"""STM32 透传后端 —— PC/树莓派指令 → UART/USB → STM32 → 1Mbps 单总线 → SCS0009。

与 backend_serial 的关系：协议层（scs_protocol golden model）完全复用，
仅传输封装不同 —— 帧长前缀协议 [len][bytes]，响应 [len][bytes] | [0]。

架构定位（2026-10-20 起的主链路）::

    Raspberry Pi（感知/策略/主状态机）
        └─ /dev/serial0 或 USB-TTL @115200
             └─ STM32F103（firmware/stm32，字节泵）
                  └─ USART1 PA9 单线半双工 @1Mbps → 8 × SCS0009

当前 FSM 仍在 Pi 上跑 Python（Phase 1）；后续可把 FSM/保护下沉到 STM32。
"""

from __future__ import annotations

import math
import time
from typing import Iterable

from app.control import scs_protocol as sp
from app.control.mcu_link import MCU_BAUD, McuLink, McuLinkError
from app.control.servo_base import (
    N_JOINTS,
    FsmState,
    FsmSnapshot,
    GraspPlan,
    ServoBackend,
    ServoFeedback,
)

TORQUE_ON = 1
TORQUE_OFF = 2


def deg_to_raw(deg: float, middle_deg: float) -> int:
    """占位换算：-180..+180 → 0..4095（真值待定项 1 实机标定）。"""
    frac = (deg + middle_deg + 180.0) / 360.0
    return max(0, min(sp.POSITION_STEPS, int(round(frac * sp.POSITION_STEPS))))


class McuBackend(ServoBackend):
    """STM32 透传后端：Pi 侧软件 FSM + 真实 SCS 总线。"""

    name = "mcu"

    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = MCU_BAUD,
        timeout_s: float = 0.05,
        middle_deg: Iterable[float] = (0.0,) * N_JOINTS,
        servo_ids: Iterable[int] = range(1, 9),
        feedback_period_s: float = 0.05,
        link: McuLink | None = None,
    ):
        self._link = link or McuLink(port, baud, timeout_s)
        self.port = port
        self.baud = int(baud)
        self._ids = tuple(int(x) for x in servo_ids)
        middle = tuple(float(x) for x in middle_deg)
        if len(middle) != len(self._ids):
            middle = middle + (0.0,) * (len(self._ids) - len(middle))
        self.middle = middle
        self._feedback_period_s = max(0.0, float(feedback_period_s))

        # FSM 状态：必须在构造时初始化，避免未 start 就 poll/release 抛 AttributeError
        self._plan: GraspPlan | None = None
        self._fsm = FsmState.IDLE
        self._state_t = 0.0
        self._timeout = False
        self._feedback: list[ServoFeedback] = [
            ServoFeedback(servo_id=sid, online=False) for sid in self._ids
        ]
        self._last_feedback_t = -1e9

    # ---- 生命周期 ----

    def open(self) -> None:
        self._link.open()

    def close(self) -> None:
        # 安全默认：先卸力，再关串口；失败不掩盖原始异常
        try:
            if self._link.is_open:
                self.set_torque(False)
        except Exception:
            pass
        finally:
            self._link.close()

    # ---- 透传事务 ----

    def _transact(self, frame: bytes, expect_reply: bool = True) -> sp.Response | None:
        try:
            raw = self._link.transact(frame, expect_reply=expect_reply)
        except McuLinkError:
            return None
        if raw is None:
            return None
        try:
            return sp.parse_response(raw)
        except sp.ProtocolError:
            return None

    # ---- ServoBackend ----

    def set_torque(self, on: bool) -> None:
        value = bytes([TORQUE_ON if on else TORQUE_OFF])
        table = {sid: value for sid in self._ids}
        self._transact(
            sp.build_sync_write(sp.Reg.TORQUE_SWITCH[0], 1, table),
            expect_reply=False,
        )

    def set_targets(self, angles_deg: tuple[float, ...], speed_raw: int = 0) -> None:
        if len(angles_deg) != len(self._ids):
            raise ValueError(f"angles_deg must have {len(self._ids)} entries")
        pos = {
            sid: deg_to_raw(angles_deg[i], self.middle[i]).to_bytes(2, "little")
            for i, sid in enumerate(self._ids)
        }
        self._transact(
            sp.build_sync_write(sp.Reg.GOAL_POSITION[0], 2, pos),
            expect_reply=False,
        )
        speed = {
            sid: (int(speed_raw) & 0xFFFF).to_bytes(2, "little") for sid in self._ids
        }
        self._transact(
            sp.build_sync_write(sp.Reg.RUNNING_SPEED[0], 2, speed),
            expect_reply=False,
        )

    def start_grasp_fsm(self, plan: GraspPlan) -> None:
        self._plan = plan
        self._fsm = FsmState.PREGRASP
        self._state_t = 0.0
        self._timeout = False
        self.set_torque(True)
        self.set_targets(plan.pregrasp_deg, plan.speed_raw)

    def release(self) -> None:
        if self._plan is None or self._fsm not in (FsmState.CLOSING, FsmState.HOLD):
            return
        self._fsm = FsmState.RELEASING
        self._state_t = 0.0
        self.set_targets(self._plan.pregrasp_deg, self._plan.speed_raw)

    def emergency_stop(self) -> None:
        self._plan = None
        self._fsm = FsmState.PROTECT
        self._timeout = False
        self.set_torque(False)

    def poll_fsm(self) -> FsmSnapshot:
        self._refresh_feedback()
        err_bitmap = 0
        offline = False
        for i, fb in enumerate(self._feedback):
            offline = offline or not fb.online
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
            timeout=self._timeout,
            fault=self._fsm == FsmState.PROTECT or offline,
            err_bitmap=err_bitmap,
            loop_timestamp=self.read_loop_timestamp(),
            feedback=list(self._feedback),
        )

    def read_loop_timestamp(self) -> int:
        return time.perf_counter_ns()

    def tick(self, dt_s: float) -> None:
        self._state_t += float(dt_s)
        plan = self._plan
        if plan is None:
            return
        if self._fsm == FsmState.PREGRASP:
            if self._state_t >= 0.3:
                self.set_targets(plan.angles_deg, plan.speed_raw)
                self._fsm = FsmState.CLOSING
                self._state_t = 0.0
        elif self._fsm == FsmState.CLOSING:
            fb = self._refresh_feedback()
            hit = any(
                f.online and f.load_raw >= plan.torque_levels[i] * 64
                for i, f in enumerate(fb)
            )
            if hit:
                self._latch_hold(plan, fb)
                self._fsm = FsmState.HOLD
                self._state_t = 0.0
            elif self._state_t >= 3.0:
                self._timeout = True
                self._latch_hold(plan, fb)
                self._fsm = FsmState.HOLD
                self._state_t = 0.0
        elif self._fsm == FsmState.HOLD:
            self._refresh_feedback()
        elif self._fsm == FsmState.RELEASING:
            if self._state_t >= 0.5:
                self._fsm = FsmState.IDLE
                self._plan = None
                self._state_t = 0.0

    def _latch_hold(self, plan: GraspPlan, fb: list[ServoFeedback]) -> None:
        """接触/超时后锁存当前实测位置为保持目标（离线通道回退到计划角）。"""
        hold = tuple(
            fb[i].position_deg if fb[i].online else plan.angles_deg[i]
            for i in range(len(self._ids))
        )
        self.set_targets(hold, plan.speed_raw)

    # ---- 反馈 ----

    def _refresh_feedback(self) -> list[ServoFeedback]:
        now = time.perf_counter()
        if now - self._last_feedback_t < self._feedback_period_s:
            return self._feedback
        self._feedback = self._read_feedback()
        self._last_feedback_t = now
        return self._feedback

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
