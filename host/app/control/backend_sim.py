"""模拟后端 —— VirtualServoBus + 软件版抓取状态机。

双重身份
========
1. PC 冒烟/全链路空转的默认后端（``settings.yaml: backend: sim``）；
2. **STM32 抓取 FSM 的行为 golden model**：内置的软件状态机
   （PREGRASP→CLOSING→HOLD→RELEASING→PROTECT）与状态迁移条件
   （到位/力矩阈值/超时/故障）作为后续 STM32 C 实现的行为参照。

物理换算（均为 SIM/占位，真值待标定）：
- 关节角：deg（相对中位）→ raw 走 scs_protocol.rad_to_raw；
- 力矩等级 0..15 → 负载阈值 raw = level * 64（占位线性映射）。
"""

from __future__ import annotations

import math
from typing import Iterable

from app.control import scs_protocol as sp
from app.control import scs_sim
from app.control.servo_base import (
    N_JOINTS,
    FsmState,
    FsmSnapshot,
    GraspPlan,
    ServoBackend,
    ServoFeedback,
)

#: 力矩等级 → 负载阈值 raw 的占位映射（TODO 实机校验，待定项 2）
TORQUE_LEVEL_STEP = 64
#: CLOSING 态力控判定的最小闭合时长（秒），避免瞬时负载误触发
MIN_CLOSING_S = 0.05
#: CLOSING 态超时（秒）：到时仍未保持 → timeout 置位进 HOLD/FAULT
CLOSING_TIMEOUT_S = 3.0
#: RELEASING 时长（秒）
RELEASING_S = 0.5
#: PREGRASP 时长（秒）
PREGRASP_S = 0.3


def deg_to_raw(deg: float, middle_deg: float) -> int:
    """关节角（相对中位，度）→ 舵机位置 raw。"""
    return sp.rad_to_raw(math.radians(middle_deg + deg))


def torque_level_to_raw(level: int) -> int:
    return level * TORQUE_LEVEL_STEP


class SimBackend(ServoBackend):
    """挂 VirtualServoBus 的全功能模拟后端。测试可访问 ``.bus`` 注入故障。"""

    name = "sim"

    def __init__(self, middle_deg: Iterable[float] = (0.0,) * N_JOINTS,
                 servo_ids: Iterable[int] = range(1, 9)):
        self.middle = tuple(middle_deg)
        if len(self.middle) != N_JOINTS:
            raise ValueError("middle_deg must have 8 entries")
        self.bus = scs_sim.VirtualServoBus(servo_ids)
        self._virtual_clock_us = 0
        self._fsm: FsmState = FsmState.IDLE
        self._plan: GraspPlan | None = None
        self._state_t: float = 0.0  # 当前态已持续时间
        self._closing_hold_raw: tuple[int, ...] | None = None
        self._closed_by_timeout: bool = False  # CLOSING 超时进入 HOLD（未达力阈值/未到位）

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def open(self) -> None:
        self._virtual_clock_us = 0

    def close(self) -> None:
        self.set_torque(False)

    # ------------------------------------------------------------------
    # ServoBackend 接口
    # ------------------------------------------------------------------

    def set_torque(self, on: bool) -> None:
        for sid in self.bus.servo_ids:
            self.bus.transact(sp.build_write(sid, sp.Reg.TORQUE_SWITCH[0], bytes([1 if on else 0])))

    def set_targets(self, angles_deg: tuple[float, ...], speed_raw: int = 0) -> None:
        """直接角度模式：SYNC_WRITE 一帧下发 8 关节。

        数据布局 = 0x2A 起连写 6B：位置(2B) + Goal_Time(2B, 0=速度模式) + 速度(2B)。
        """
        table = {
            sid: bytes([
                deg_to_raw(angles_deg[i], self.middle[i]) & 0xFF,
                (deg_to_raw(angles_deg[i], self.middle[i]) >> 8) & 0xFF,
                0x00, 0x00,  # Goal_Time = 0：使用速度限制（不用时间模式）
                speed_raw & 0xFF,
                (speed_raw >> 8) & 0xFF,
            ])
            for i, sid in enumerate(self.bus.servo_ids)
        }
        self.bus.transact(sp.build_sync_write(sp.Reg.GOAL_POSITION[0], 6, table))

    def start_grasp_fsm(self, plan: GraspPlan) -> None:
        """CMD=0x01：进入 PREGRASP，随后硬件（此处软件）执行闭合。"""
        self._plan = plan
        self._enter(FsmState.PREGRASP)

    def release(self) -> None:
        """CMD=0x02：RELEASING → 张开至预张位 → IDLE。"""
        if self._fsm in (FsmState.HOLD, FsmState.CLOSING):
            self._enter(FsmState.RELEASING)

    def emergency_stop(self) -> None:
        self._enter(FsmState.PROTECT)

    def poll_fsm(self) -> FsmSnapshot:
        feedback = self._read_feedback()
        err_bitmap = 0
        for i, fb in enumerate(feedback):
            if not fb.online:
                err_bitmap |= 1 << (i * 3 + 1)  # 通道 i 掉线位
            elif fb.error_bits & sp.ERR_OVERLOAD:
                err_bitmap |= 1 << (i * 3)
            elif fb.error_bits & sp.ERR_OVERHEAT:
                err_bitmap |= 1 << (i * 3 + 2)
        grasped = self._fsm == FsmState.HOLD and self._plan is not None and not self._closed_by_timeout
        timeout = self._closed_by_timeout
        return FsmSnapshot(
            state=self._fsm,
            busy=self._fsm not in (FsmState.IDLE, FsmState.HOLD),
            grasped=grasped,
            timeout=timeout,
            fault=self._fsm == FsmState.PROTECT,
            err_bitmap=err_bitmap,
            loop_timestamp=self.read_loop_timestamp(),
            feedback=feedback,
        )

    def read_loop_timestamp(self) -> int:
        return self._virtual_clock_us

    # ------------------------------------------------------------------
    # 时间推进 + 软件状态机（STM32 抓取 FSM 的行为对照）
    # ------------------------------------------------------------------

    def tick(self, dt_s: float) -> None:
        self._virtual_clock_us += int(round(dt_s * 1e6))
        self.bus.step(dt_s)
        self._state_t += dt_s
        self._run_fsm(dt_s)

    def _enter(self, state: FsmState) -> None:
        self._fsm = state
        self._state_t = 0.0
        if state == FsmState.PREGRASP:
            self._closed_by_timeout = False
        if state == FsmState.PREGRASP and self._plan is not None:
            self.set_torque(True)
            self.set_targets(self._plan.pregrasp_deg, self._plan.speed_raw)
        elif state == FsmState.RELEASING and self._plan is not None:
            self.set_targets(self._plan.pregrasp_deg, self._plan.speed_raw)
        elif state == FsmState.PROTECT:
            self.set_torque(False)
            self._plan = None
            self._closing_hold_raw = None

    def _run_fsm(self, dt_s: float) -> None:
        plan = self._plan
        if self._fsm == FsmState.PREGRASP:
            if self._state_t >= PREGRASP_S:
                assert plan is not None
                self.set_targets(plan.angles_deg, plan.speed_raw)
                self._enter(FsmState.CLOSING)

        elif self._fsm == FsmState.CLOSING:
            assert plan is not None
            # 力控判定：任一通道负载达到其力矩阈值 → 停止并保持当前位
            fb = self._read_feedback()
            hit = any(
                f.online and f.load_raw >= torque_level_to_raw(plan.torque_levels[i])
                for i, f in enumerate(fb)
            )
            reached = all(
                abs(deg_to_raw(plan.angles_deg[i], self.middle[i]) - f.position_raw) <= 4
                for i, f in enumerate(fb)
                if f.online
            )
            if hit or reached:
                self._closed_by_timeout = False
                self._closing_hold_raw = tuple(f.position_raw for f in fb)
                self._enter(FsmState.HOLD)
            elif self._state_t >= CLOSING_TIMEOUT_S:
                self._closed_by_timeout = True
                self._closing_hold_raw = tuple(f.position_raw for f in fb)
                self._enter(FsmState.HOLD)  # 超时同样停止（快照 timeout=True 区分）

        elif self._fsm == FsmState.HOLD:
            # 保持：持续重申当前锁定位置（STM32 以寄存器保持，软件环需重发）
            if self._closing_hold_raw:
                table = {
                    sid: bytes([r & 0xFF, (r >> 8) & 0xFF, 0x00, 0x00, 0xFF, 0xFF])
                    for sid, r in zip(self.bus.servo_ids, self._closing_hold_raw)
                }
                self.bus.transact(sp.build_sync_write(sp.Reg.GOAL_POSITION[0], 6, table))
            # 保护迁移：堵转/超温/掉线 → PROTECT
            fb = self._read_feedback()
            if any(not f.online for f in fb) or any(
                f.error_bits & (sp.ERR_OVERHEAT | sp.ERR_OVERLOAD) for f in fb
            ):
                self._enter(FsmState.PROTECT)

        elif self._fsm == FsmState.RELEASING:
            if self._state_t >= RELEASING_S:
                self._plan = None
                self._closing_hold_raw = None
                self._enter(FsmState.IDLE)

    # ------------------------------------------------------------------
    # 反馈
    # ------------------------------------------------------------------

    def _read_feedback(self) -> list[ServoFeedback]:
        out: list[ServoFeedback] = []
        for i, sid in enumerate(self.bus.servo_ids):
            s = self.bus.state(sid)
            online = not s.dropped
            pos_raw = s.position_raw if online else 0
            out.append(ServoFeedback(
                servo_id=sid,
                online=online,
                position_raw=pos_raw,
                position_deg=sp.raw_to_rad(pos_raw) * 180.0 / math.pi - self.middle[i],
                load_raw=s.load_raw if online else 0,
                temperature_c=s.temperature_c if online else 0.0,
                voltage_x10=s.voltage_x10 if online else 0,
                error_bits=s.error_bits if online else 0,
            ))
        return out
