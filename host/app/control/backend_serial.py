"""软件环后端 —— USB-TTL 直驱舵机总线（Pi/PC 调试与对比基准）。

- 通道：USB-TTL / 总线驱动板（树莓派或 PC USB → /dev/ttyUSB0），1Mbps 半双工；
- 协议：``scs_protocol`` 组包/解包（与 sim/mcu 后端同一份 golden model）；
- 状态机：与 backend_sim 相同逻辑的软件实现（Python 循环）；
  ``read_loop_timestamp`` 用 ``time.perf_counter_ns()`` 供抖动统计；
- 树莓派/Linux 或 Windows（COMx）运行。
"""

from __future__ import annotations

import math
import time
from typing import Iterable

from app.control import scs_protocol as sp
from app.control.servo_base import (
    N_JOINTS,
    FsmState,
    FsmSnapshot,
    GraspPlan,
    ServoBackend,
    ServoFeedback,
)

#: 关节角（度，相对中位）→ raw：真实换算依赖待定项 1 标定，先按几何满量程占位
ANGLE_RANGE_DEG = 180.0  # TODO(实机校验)：MiddlePos ± 行程，标定后出 servo_map


def deg_to_raw(deg: float, middle_deg: float) -> int:
    """占位换算：-ANGLE_RANGE..+ANGLE_RANGE → 0..4095。真值表待标定替换。"""
    frac = (deg + middle_deg + ANGLE_RANGE_DEG) / (2 * ANGLE_RANGE_DEG)
    return max(0, min(sp.POSITION_STEPS, int(round(frac * sp.POSITION_STEPS))))


class SerialBackend(ServoBackend):
    name = "serial"

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 1_000_000,
                 timeout_s: float = 0.05, middle_deg: Iterable[float] = (0.0,) * N_JOINTS,
                 servo_ids: Iterable[int] = range(1, 9)):
        self._port_cfg = port
        self._baud = baudrate
        self._timeout_s = timeout_s
        self.middle = tuple(middle_deg)
        self._ids = tuple(servo_ids)
        self._ser: object | None = None  # serial.Serial（延迟导入）
        # 软件状态机状态（与 backend_sim 逻辑一致，逐段对照）
        self._fsm: FsmState = FsmState.IDLE
        self._plan: GraspPlan | None = None
        self._state_t: float = 0.0

    # -- 串口基础 -------------------------------------------------------

    def open(self) -> None:
        try:
            import serial  # pyserial：仅真实后端需要
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("backend_serial 需要 pyserial（pip install pyserial）") from exc
        self._ser = serial.Serial(self._port_cfg, self._baud, timeout=self._timeout_s)

    def close(self) -> None:
        if self._ser is not None:
            try:
                self.set_torque(False)
            finally:
                self._ser.close()
                self._ser = None

    def _transact(self, frame: bytes, expect_reply: bool) -> sp.Response | None:
        """写请求帧，读一帧响应（半双工：发完才收）。"""
        assert self._ser is not None
        self._ser.write(frame)
        self._ser.flush()
        if not expect_reply:
            return None
        deadline = time.perf_counter() + self._timeout_s
        buf = bytearray()
        while time.perf_counter() < deadline:
            n = self._ser.in_waiting
            if n:
                buf += self._ser.read(n)
                try:
                    return sp.parse_response(bytes(buf))
                except sp.ProtocolError:
                    continue  # 半帧/坏帧：继续凑字节
        return None

    # -- ServoBackend 接口 ----------------------------------------------

    def set_torque(self, on: bool) -> None:
        for sid in self._ids:
            self._transact(sp.build_write(sid, sp.Reg.TORQUE_SWITCH[0], bytes([1 if on else 0])), True)

    def set_targets(self, angles_deg: tuple[float, ...], speed_raw: int = 0) -> None:
        for i, sid in enumerate(self._ids):
            raw = deg_to_raw(angles_deg[i], self.middle[i])
            self._transact(sp.build_write_u16(sid, sp.Reg.GOAL_POSITION[0], raw), True)
            self._transact(sp.build_write_u16(sid, sp.Reg.RUNNING_SPEED[0], speed_raw), True)

    def start_grasp_fsm(self, plan: GraspPlan) -> None:
        self._plan = plan
        self._fsm = FsmState.PREGRASP
        self._state_t = 0.0
        self.set_torque(True)
        self.set_targets(plan.pregrasp_deg, plan.speed_raw)

    def release(self) -> None:
        if self._plan is not None and self._fsm in (FsmState.CLOSING, FsmState.HOLD):
            self._fsm = FsmState.RELEASING
            self._state_t = 0.0
            assert self._plan is not None
            self.set_targets(self._plan.pregrasp_deg, self._plan.speed_raw)

    def emergency_stop(self) -> None:
        self._fsm = FsmState.PROTECT
        self._plan = None
        self.set_torque(False)

    def poll_fsm(self) -> FsmSnapshot:
        feedback = self._read_feedback()
        err_bitmap = 0
        for i, fb in enumerate(feedback):
            if not fb.online:
                err_bitmap |= 1 << (i * 3 + 1)
            elif fb.error_bits & sp.ERR_OVERLOAD:
                err_bitmap |= 1 << (i * 3)
            elif fb.error_bits & sp.ERR_OVERHEAT:
                err_bitmap |= 1 << (i * 3 + 2)
        return FsmSnapshot(
            state=self._fsm,
            busy=self._fsm not in (FsmState.IDLE, FsmState.HOLD),
            grasped=self._fsm == FsmState.HOLD,
            fault=self._fsm == FsmState.PROTECT,
            err_bitmap=err_bitmap,
            loop_timestamp=self.read_loop_timestamp(),
            feedback=feedback,
        )

    def read_loop_timestamp(self) -> int:
        return time.perf_counter_ns()

    def tick(self, dt_s: float) -> None:
        """软件环：状态机在 Python 循环里推进（与 STM32 硬件环对比的核心差异）。"""
        self._state_t += dt_s
        plan = self._plan
        if plan is None:
            return
        if self._fsm == FsmState.PREGRASP and self._state_t >= 0.3:
            self.set_targets(plan.angles_deg, plan.speed_raw)
            self._fsm, self._state_t = FsmState.CLOSING, 0.0
        elif self._fsm == FsmState.CLOSING:
            fb = self._read_feedback()
            hit = any(
                f.online and f.load_raw >= plan.torque_levels[i] * 64
                for i, f in enumerate(fb)
            )
            if hit or self._state_t >= 3.0:
                self._fsm, self._state_t = FsmState.HOLD, 0.0
        elif self._fsm == FsmState.RELEASING and self._state_t >= 0.5:
            self._fsm, self._plan = FsmState.IDLE, None

    def _read_feedback(self) -> list[ServoFeedback]:
        out: list[ServoFeedback] = []
        for i, sid in enumerate(self._ids):
            fb = ServoFeedback(servo_id=sid, online=False)
            resp = self._transact(
                sp.build_read(sid, sp.Reg.FEEDBACK_ADDR, sp.Reg.FEEDBACK_LEN), True
            )
            if resp is not None and len(resp.params) == sp.Reg.FEEDBACK_LEN:
                pos_raw = resp.param_u16(0)
                fb = ServoFeedback(
                    servo_id=sid,
                    online=True,
                    position_raw=pos_raw,
                    position_deg=sp.raw_to_rad(pos_raw) * 180.0 / math.pi - self.middle[i],
                    load_raw=resp.param_u16(4),
                    temperature_c=float(resp.params[7]),
                    voltage_x10=resp.params[6],
                    error_bits=resp.error,
                )
            out.append(fb)
        return out
