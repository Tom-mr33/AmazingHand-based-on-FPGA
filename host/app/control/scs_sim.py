"""虚拟 SCS0009 舵机总线 —— 帧进帧出的设备模拟器（冒烟测试 / host 空转 / 协议参考）。

定位
====
- **帧级接口**：``transact(frame) -> bytes | None``，对应半双工总线"主发一帧、
  从回一帧"的真实行为；组包/解包全部走 :mod:`scs_protocol`，自身不做协议逻辑。
- **确定性**：所有物理演化由调用方显式 ``step(dt)`` 推进，不依赖真实时钟——
  同一帧序列 + 同一 step 序列必然得到同一响应序列，因此可作为
  STM32 侧总线实现的协议行为参考（golden model）。
- **故障注入**：掉线 / 堵转 / 过温，供抓取状态机（backend_sim / STM32 FSM）
  的保护逻辑在无硬件阶段反复演练。

模拟范围（SIM ONLY，冒烟级）
============================
- 位置：torque 使能时以 ``speed_raw`` 限速逼近 ``goal_raw``；
- 负载：外部注入 ``external_load_raw``（模拟手指夹持物体的接触力）+
  运动分量，堵转 = 外部负载顶满力矩极限且尚未到位 → 响应置 ``ERR_OVERLOAD``；
- 温度：负载积分升温、自然散热，超 ``max_temperature_limit`` →
  置 ``ERR_OVERHEAT`` 并自动切断力矩（模拟 Alarm_Shutdown）；
- 掉线：``inject_drop`` 后 ``transact`` 直接返回 None（主机侧超时语义）。

注意：速度/温度/负载的 raw 单位与真实舵机的换算系数**未实机标定**
（待定项 1/2），本模块所有常数仅用于行为演示与测试，
禁止当作物理真值引用。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable, Mapping

from app.control import scs_protocol as sp

# --------------------------------------------------------------------------
# 虚拟参数（SIM ONLY，禁止当真值）
# --------------------------------------------------------------------------

#: speed_raw 直译为 steps/s 的系数（真实单位待实机标定）
SPEED_STEPS_PER_S = 1.0
#: 空载温度与环境
AMBIENT_TEMP_C = 25.0
HEAT_RATE_C_PER_S = 0.5  # 满载持续升温速率
COOL_RATE_PER_S = 0.1  # 对环境散热速率
#: 默认力矩极限（raw，10bit 幅值）
DEFAULT_TORQUE_LIMIT_RAW = 1023
#: 出厂默认位置（中位）
DEFAULT_POSITION_RAW = 2048
#: 负载方向位（真实 Load 寄存器 bit10 为方向，虚拟机仅存幅值；实机校验时统一）
LOAD_MAX_RAW = 1023


@dataclass
class ServoState:
    """单个虚拟舵机的完整状态。"""

    servo_id: int
    model_number: int = 0x0009  # TODO(实机校验): SCS0009 官方 model raw
    position_raw: int = DEFAULT_POSITION_RAW
    speed_raw: int = 0  # 回读的当前速度
    load_raw: int = 0  # 当前负载幅值（0..1023）
    temperature_c: float = AMBIENT_TEMP_C
    voltage_x10: int = 74  # 7.4V
    torque_on: bool = False
    goal_raw: int = DEFAULT_POSITION_RAW
    goal_time_raw: int = 0  # 到达时间（0 = 不限时间，用速度限制）
    goal_speed_raw: int = 0  # 0 = 全速（真实语义：0 表示最大速度）
    max_temperature_limit_c: float = 70.0
    torque_limit_raw: int = DEFAULT_TORQUE_LIMIT_RAW
    status_return_level: int = 2  # 2 = 所有指令均响应（真实语义，待核对）
    moving: bool = False
    registered: bool = False
    #: 故障/注入
    dropped: bool = False  # 掉线注入：不响应任何帧
    external_load_raw: int = 0  # 外部注入负载（夹持阻力）
    overheat_shutdown: bool = False  # 过温切断后保持
    stalled: bool = False  # 本 step 是否堵转
    #: REG_WRITE 暂存
    _pending: tuple[int, bytes] | None = field(default=None, repr=False)

    @property
    def error_bits(self) -> int:
        """当前状态包应携带的 ERROR 位。"""
        err = 0
        if self.overheat_shutdown:
            err |= sp.ERR_OVERHEAT
        if self.stalled:
            err |= sp.ERR_OVERLOAD
        return err


def _clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


class VirtualServoBus:
    """挂在一条半双工总线上的 N 个虚拟 SCS0009。"""

    def __init__(self, servo_ids: Iterable[int] = range(1, 9)):
        self._servos: dict[int, ServoState] = {
            sid: ServoState(servo_id=sid) for sid in servo_ids
        }

    # ------------------------------------------------------------------
    # 调试 / 测试便捷接口（非总线协议）
    # ------------------------------------------------------------------

    def state(self, servo_id: int) -> ServoState:
        return self._servos[servo_id]

    @property
    def servo_ids(self) -> list[int]:
        return sorted(self._servos)

    def inject_drop(self, servo_ids: Iterable[int], drop: bool = True) -> None:
        """注入掉线故障：被注入舵机对任何帧都不再响应（主机侧超时）。"""
        for sid in servo_ids:
            self._servos[sid].dropped = drop

    def inject_external_load(
        self, load_by_id: Mapping[int, int]
    ) -> None:
        """注入外部负载（raw 0..1023），模拟手指接触/夹持物体。"""
        for sid, load in load_by_id.items():
            self._servos[sid].external_load_raw = _clamp(load, 0, LOAD_MAX_RAW)

    # ------------------------------------------------------------------
    # 时间推进（确定性动力学）
    # ------------------------------------------------------------------

    def step(self, dt_s: float) -> None:
        """推进 dt 秒虚拟时间：位置限速逼近 + 负载/温度演化 + 保护判定。"""
        assert dt_s >= 0
        for s in self._servos.values():
            s.stalled = False
            if s.overheat_shutdown or not s.torque_on:
                s.moving = False
                s.speed_raw = 0
                s.load_raw = self._coast_load(s)
                self._thermal(s, dt_s)
                continue

            delta = s.goal_raw - s.position_raw
            if delta == 0:
                s.moving = False
                s.speed_raw = 0
            else:
                # 全速语义：goal_speed_raw == 0 表示不限速（真实舵机行为）
                if s.goal_speed_raw == 0:
                    move = int(round(delta))
                else:
                    limit = s.goal_speed_raw * SPEED_STEPS_PER_S * dt_s
                    move = _clamp(int(round(delta)), -int(limit), int(limit))
                    if move == 0:
                        move = 1 if delta > 0 else -1  # 至少 1 步，避免无限挂起
                # 堵转：外部负载顶满力矩极限则推不动
                if s.external_load_raw >= s.torque_limit_raw:
                    s.stalled = True
                    s.speed_raw = 0
                else:
                    s.position_raw = _clamp(s.position_raw + move, 0, sp.POSITION_STEPS)
                    s.speed_raw = (
                        s.goal_speed_raw if s.goal_speed_raw else int(SPEED_STEPS_PER_S)
                    )
                s.moving = s.position_raw != s.goal_raw
            s.load_raw = self._load_of(s)
            self._thermal(s, dt_s)

    def _load_of(self, s: ServoState) -> int:
        """负载 = 外部注入 + 运动分量（运动越快/越重越吃力，冒烟级模型）。"""
        moving_component = 60 if s.moving else 0
        return _clamp(s.external_load_raw + moving_component, 0, LOAD_MAX_RAW)

    def _coast_load(self, s: ServoState) -> int:
        return _clamp(s.external_load_raw // 4, 0, LOAD_MAX_RAW)

    def _thermal(self, s: ServoState, dt_s: float) -> None:
        s.temperature_c += (
            (s.load_raw / LOAD_MAX_RAW) * HEAT_RATE_C_PER_S - COOL_RATE_PER_S
        ) * dt_s
        if s.temperature_c < AMBIENT_TEMP_C:
            s.temperature_c = AMBIENT_TEMP_C
        if (
            not s.overheat_shutdown
            and s.temperature_c >= s.max_temperature_limit_c
        ):
            s.overheat_shutdown = True
            s.torque_on = False  # Alarm_Shutdown 行为：自动切断力矩

    # ------------------------------------------------------------------
    # 总线事务（帧进帧出）
    # ------------------------------------------------------------------

    def transact(self, frame: bytes) -> bytes | None:
        """处理一条主机请求帧；返回响应帧或 None（广播 / 不响应）。"""
        start = sp.locate_frame(frame)
        if start < 0:
            return None  # 垃圾字节：真实从机同样不响应
        if len(frame) - start < 5:
            return None  # 半帧到达（仅帧头/ID/LEN 不全）：本帧丢弃，主机侧重试
        servo_id = frame[start + 2]
        length = frame[start + 3]
        end = start + 4 + length
        if length < 2 or len(frame) < end:
            return None
        instruction = frame[start + 4]
        params = frame[start + 5 : end - 1]
        cs = frame[end - 1]
        expect = sp.checksum(servo_id, length, instruction, *params)
        if cs != expect:
            return None  # 校验错：真实从机直接丢弃

        is_broadcast = servo_id == sp.BROADCAST_ID

        if instruction == sp.INS_PING:
            if is_broadcast:
                return None  # 广播 PING 会碰撞，模拟为不响应
            servo = self._servos.get(servo_id)
            if servo is None or servo.dropped:
                return None
            return self._respond(servo_id, servo.error_bits, b"")

        if instruction == sp.INS_SYNC_WRITE:
            self._apply_sync_write(params)
            return None  # 广播指令无响应

        if instruction == sp.INS_ACTION:
            for s in self._servos.values():
                if s._pending is not None:
                    self._apply_write(s, *s._pending)
                    s._pending = None
                    s.registered = False
            return None

        if is_broadcast:
            if instruction == sp.INS_RESET:
                # 广播复位：全部舵机恢复出厂态（真实从机语义），不响应
                for s in self._servos.values():
                    self._reset(s)
                return None
            # 广播 WRITE / REG_WRITE：执行但不响应（避免总线碰撞）
            if instruction == sp.INS_WRITE and params:
                for s in self._servos.values():
                    self._apply_write(s, params[0], bytes(params[1:]))
            elif instruction == sp.INS_REG_WRITE and params:
                for s in self._servos.values():
                    s._pending = (params[0], bytes(params[1:]))
                    s.registered = True
            return None

        # ---- 单播：定位舵机 ----
        servo = self._servos.get(servo_id)
        if servo is None or servo.dropped:
            return None  # 总线上不存在 / 已掉线

        if instruction == sp.INS_READ:
            addr, n = params[0], params[1]
            data = self._read_regs(servo, addr, n)
            return self._respond(servo_id, servo.error_bits, data)

        if instruction == sp.INS_WRITE and params:
            self._apply_write(servo, params[0], bytes(params[1:]))
            return self._maybe_ack(servo, err=servo.error_bits)

        if instruction == sp.INS_REG_WRITE and params:
            servo._pending = (params[0], bytes(params[1:]))
            servo.registered = True
            return self._maybe_ack(servo, err=servo.error_bits)

        if instruction == sp.INS_RESET:
            self._reset(servo)
            return self._maybe_ack(servo, err=0)

        # 未定义指令：置 Instruction Error 并响应（真实从机行为）
        return self._respond(servo_id, servo.error_bits | sp.ERR_INSTRUCTION, b"")

    # ------------------------------------------------------------------
    # 内部：寄存器视图 / 写分发 / 响应组帧
    # ------------------------------------------------------------------

    def _register_view(self, s: ServoState) -> dict[int, int]:
        """摊平状态为 addr -> byte（0x00..0x43），未列出的填 0。"""
        v: dict[int, int] = {a: 0 for a in range(0x44)}

        def put(addr: int, size: int, value: int) -> None:
            for i in range(size):
                v[addr + i] = (value >> (8 * i)) & 0xFF

        put(*sp.Reg.MODEL_NUMBER, s.model_number)
        v[sp.Reg.FIRMWARE_VERSION[0]] = 0x01
        v[sp.Reg.ID[0]] = s.servo_id
        v[sp.Reg.BAUD_RATE[0]] = 0x07  # TODO(实机校验): 1M 波特率对应 raw
        v[sp.Reg.MIN_ANGLE_LIMIT[0]] = 0x00
        v[sp.Reg.MAX_ANGLE_LIMIT[0]] = 0xFF  # TODO(实机校验): 行程上限 raw
        v[sp.Reg.MAX_TEMPERATURE_LIMIT[0]] = int(s.max_temperature_limit_c)
        put(*sp.Reg.MAX_TORQUE, s.torque_limit_raw)
        v[sp.Reg.STATUS_RETURN_LEVEL[0]] = s.status_return_level
        v[sp.Reg.TORQUE_SWITCH[0]] = 1 if s.torque_on else 0
        put(*sp.Reg.GOAL_POSITION, s.goal_raw)
        put(*sp.Reg.GOAL_TIME, s.goal_time_raw)
        put(*sp.Reg.RUNNING_SPEED, s.goal_speed_raw)
        put(*sp.Reg.TORQUE_LIMIT, s.torque_limit_raw)
        put(*sp.Reg.PRESENT_POSITION, s.position_raw)
        put(*sp.Reg.PRESENT_SPEED, s.speed_raw)
        put(*sp.Reg.PRESENT_LOAD, s.load_raw)
        v[sp.Reg.PRESENT_VOLTAGE[0]] = s.voltage_x10
        v[sp.Reg.PRESENT_TEMPERATURE[0]] = int(round(s.temperature_c))
        v[sp.Reg.REGISTERED_INSTRUCTION[0]] = 1 if s.registered else 0
        v[sp.Reg.MOVING[0]] = 1 if s.moving else 0
        return v

    def _read_regs(self, s: ServoState, addr: int, n: int) -> bytes:
        view = self._register_view(s)
        return bytes(view.get(addr + i, 0) for i in range(n))

    def _apply_write(self, s: ServoState, addr: int, data: bytes) -> None:
        """把 WRITE 落到状态（小端）。只读区忽略；ID 改写需重挂字典。"""
        if addr == sp.Reg.TORQUE_SWITCH[0]:
            s.torque_on = data[0] == 1  # 1=On / 2=Off / 3=Free（官方 Demo 语义）
            if s.torque_on and s.overheat_shutdown:
                s.torque_on = False  # 过温切断状态下拒绝重新使能
            return
        if addr == sp.Reg.GOAL_POSITION[0]:
            # 部分写支持：单字节写 = 只改低字节（真实舵机行为）
            s.goal_raw = (s.goal_raw & 0x0F00) | data[0] if len(data) < 2 else (data[0] | (data[1] << 8)) & sp.POSITION_STEPS
            # 连写视图：0x2A 起 6B = 位置(2B) + Goal_Time(2B) + 速度(2B)
            if len(data) >= 6:
                s.goal_time_raw = data[2] | (data[3] << 8)
                s.goal_speed_raw = data[4] | (data[5] << 8)
            return
        if addr == sp.Reg.GOAL_TIME[0]:
            s.goal_time_raw = data[0] | ((data[1] << 8) if len(data) >= 2 else 0)
            return
        if addr == sp.Reg.RUNNING_SPEED[0]:
            s.goal_speed_raw = data[0] | ((data[1] << 8) if len(data) >= 2 else 0)
            return
        if addr == sp.Reg.TORQUE_LIMIT[0]:
            s.torque_limit_raw = data[0] | ((data[1] << 8) if len(data) >= 2 else 0)
            return
        if addr == sp.Reg.MAX_TEMPERATURE_LIMIT[0]:
            s.max_temperature_limit_c = float(data[0])
            return
        if addr == sp.Reg.STATUS_RETURN_LEVEL[0]:
            s.status_return_level = data[0]
            return
        if addr == sp.Reg.ID[0] and data[0] in self._accept_new_id():
            old = s.servo_id
            s.servo_id = data[0]
            self._servos[data[0]] = self._servos.pop(old)
            return
        # 其余（EE 配置区 / 只读区）：冒烟级模拟器忽略写入

    def _accept_new_id(self) -> set[int]:
        return set(range(0, sp.MAX_ID + 1)) - set(self._servos)

    def _apply_sync_write(self, params: bytes) -> None:
        """SYNC_WRITE 参数解析：[addr, len, (id, data...) × N]。"""
        addr, per = params[0], params[1]
        body = params[2:]
        for i in range(0, len(body) - per + 1, per + 1):
            sid, data = body[i], body[i + 1 : i + 1 + per]
            servo = self._servos.get(sid)
            if servo is not None and not servo.dropped:
                self._apply_write(servo, addr, data)

    def _reset(self, s: ServoState) -> None:
        fresh = ServoState(servo_id=s.servo_id)
        s.__dict__.update(fresh.__dict__)

    def _respond(self, servo_id: int, err: int, params: bytes) -> bytes:
        length = len(params) + 2  # error + checksum
        cs = sp.checksum(servo_id, length, err, *params)
        return bytes([sp.HEADER0, sp.HEADER1, servo_id, length, err, *params, cs])

    def _maybe_ack(self, s: ServoState, err: int) -> bytes | None:
        """按 status_return_level 决定写指令是否回 ACK（真实舵机语义）。"""
        if s.status_return_level >= 2:
            return self._respond(s.servo_id, err, b"")
        return None
