"""STM32 控制后端 —— Pi/PC ↔ STM32F103 实时控制协议。

与 backend_mcu（原始 SCS 透传）不同：本后端只发高层命令/收遥测，
SCS 总线调度、抓取 FSM、保护与看门狗都在 STM32 内运行。

协议见 host/app/control/stm32_protocol.py 与
docs/design/stm32_control_protocol.md。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Iterable

from app.control import scs_protocol as sp
from app.control import stm32_protocol as p
from app.control.servo_base import (
    N_JOINTS,
    FsmState,
    FsmSnapshot,
    GraspPlan,
    ServoBackend,
    ServoFeedback,
)


class Stm32LinkError(RuntimeError):
    """串口打不开、INFO 超时或链路不可用。"""


class Stm32Backend(ServoBackend):
    """STM32 实时控制后端：只发命令/收遥测，不跑软件 FSM。"""

    name = "stm32"

    def __init__(
        self,
        port: str = "/dev/serial0",
        baud: int = 115200,
        timeout_s: float = 0.2,
        heartbeat_s: float = 0.1,
        telemetry_stale_s: float = 1.0,
        middle_deg: Iterable[float] = (0.0,) * N_JOINTS,
        servo_ids: Iterable[int] = range(1, 9),
        serial_factory=None,
    ):
        self.port = port
        self.baud = int(baud)
        self.timeout_s = float(timeout_s)
        self.heartbeat_s = float(heartbeat_s)
        self.telemetry_stale_s = float(telemetry_stale_s)
        self._ids = tuple(int(x) for x in servo_ids)
        middle = tuple(float(x) for x in middle_deg)
        if len(middle) != len(self._ids):
            middle = middle + (0.0,) * (len(self._ids) - len(middle))
        self.middle = middle

        self._factory = serial_factory
        self._ser = None
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._write_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._seq_lock = threading.Lock()
        self._seq = 0
        self._parser = p.FrameParser()
        self._info: p.Info | None = None
        self._info_event = threading.Event()
        self._telemetry: p.Telemetry | None = None
        self._last_rx = 0.0
        self._last_hb = 0.0
        self._last_error: tuple[int, int] | None = None
        self._raw_response: bytes | None = None
        self._raw_event = threading.Event()
        self._open_raw = self._deg8_to_raw((0.0,) * len(self._ids))

    # ---- 生命周期 ----

    def open(self) -> None:
        if self._ser is not None:
            return
        factory = self._factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise Stm32LinkError("backend_stm32 需要 pyserial") from exc
            factory = serial.Serial
        try:
            self._ser = factory(self.port, self.baud, timeout=self.timeout_s)
        except Exception as exc:
            raise Stm32LinkError(f"无法打开串口 {self.port}: {exc}") from exc

        self._stop.clear()
        self._info_event.clear()
        self._reader = threading.Thread(target=self._reader_loop, name="stm32-reader", daemon=True)
        self._reader.start()
        self._last_rx = time.monotonic()
        self._last_hb = self._last_rx
        try:
            self._send(p.build_hello(self._next_seq()))
        except Exception as exc:
            self.close()
            raise Stm32LinkError(f"发送 HELLO 失败: {exc}") from exc
        if not self._info_event.wait(1.0):
            self.close()
            raise Stm32LinkError("STM32 INFO 超时（检查固件/串口/波特率）")

    def close(self) -> None:
        if self._ser is None:
            return
        try:
            self._send(p.build_command(self._next_seq(), p.CMD_STOP))
        except Exception:
            pass
        self._stop.set()
        try:
            self._ser.close()
        except Exception:
            pass
        if self._reader is not None:
            self._reader.join(timeout=1.0)
            self._reader = None
        self._ser = None
        with self._state_lock:
            self._telemetry = None
            self._info = None

    # ---- ServoBackend ----

    def set_torque(self, on: bool) -> None:
        self._send(p.build_set_torque(self._next_seq(), on))

    def set_targets(self, angles_deg: tuple[float, ...], speed_raw: int = 0) -> None:
        if len(angles_deg) != len(self._ids):
            raise ValueError(f"angles_deg must have {len(self._ids)} entries")
        self._send(p.build_direct_targets(self._next_seq(), self._deg8_to_raw(angles_deg), speed_raw))

    def start_grasp_fsm(self, plan: GraspPlan) -> None:
        pregrasp = self._deg8_to_raw(plan.pregrasp_deg)
        close = self._deg8_to_raw(plan.angles_deg)
        thresholds = tuple(min(0xFFFF, int(t) * 64) for t in plan.torque_levels)
        self._open_raw = pregrasp
        self._send(p.build_command(
            self._next_seq(), p.CMD_GRASP,
            speed_raw=plan.speed_raw,
            torque_levels=tuple(plan.torque_levels),
            pregrasp_raw=pregrasp,
            close_raw=close,
            load_threshold_raw=thresholds,
            closing_timeout_ms=3000,
        ))

    def release(self) -> None:
        self._send(p.build_command(
            self._next_seq(), p.CMD_RELEASE,
            pregrasp_raw=self._open_raw,
            close_raw=self._open_raw,
            load_threshold_raw=(0,) * 8,
        ))

    def emergency_stop(self) -> None:
        self._send(p.build_command(self._next_seq(), p.CMD_STOP))

    def poll_fsm(self) -> FsmSnapshot:
        with self._state_lock:
            t = self._telemetry
            last = self._last_rx
        if t is None or time.monotonic() - last > self.telemetry_stale_s:
            return FsmSnapshot(
                state=FsmState.PROTECT,
                fault=True,
                err_bitmap=p.FAULT_WATCHDOG,
                loop_timestamp=0,
                feedback=[ServoFeedback(servo_id=sid, online=False) for sid in self._ids],
            )
        state = FsmState(t.state) if 0 <= t.state <= 5 else FsmState.PROTECT
        feedback = []
        for i, sid in enumerate(self._ids):
            online = t.online[i]
            raw = t.pos_raw[i]
            feedback.append(ServoFeedback(
                servo_id=sid,
                online=online,
                position_raw=raw,
                position_deg=sp.raw_to_rad(raw) * 180.0 / math.pi - self.middle[i],
                load_raw=t.load_raw[i],
                temperature_c=float(t.temp_c[i]),
                error_bits=t.error_bits[i],
            ))
        return FsmSnapshot(
            state=state,
            busy=t.moving or state in (FsmState.PREGRASP, FsmState.CLOSING, FsmState.RELEASING),
            grasped=t.grasped,
            timeout=t.timeout,
            fault=bool(t.fault),
            err_bitmap=t.fault,
            loop_timestamp=t.tick_ms,
            feedback=feedback,
        )

    def read_loop_timestamp(self) -> int:
        with self._state_lock:
            return self._telemetry.tick_ms if self._telemetry is not None else 0

    def tick(self, dt_s: float) -> None:
        now = time.monotonic()
        if now - self._last_hb >= self.heartbeat_s and self._ser is not None:
            self._last_hb = now
            try:
                self._send(p.build_heartbeat(self._next_seq()))
            except Exception:
                pass

    # ---- 调试/标定 ----

    def raw_scs(self, frame: bytes, timeout_s: float = 0.2) -> bytes | None:
        with self._state_lock:
            self._raw_response = None
        self._raw_event.clear()
        self._send(p.build_raw_scs(self._next_seq(), frame))
        if not self._raw_event.wait(timeout_s):
            return None
        with self._state_lock:
            return self._raw_response

    # ---- 内部 ----

    def _next_seq(self) -> int:
        with self._seq_lock:
            self._seq = (self._seq + 1) & 0xFF
            return self._seq

    def _send(self, frame: bytes) -> None:
        if self._ser is None:
            raise Stm32LinkError("链路未打开")
        with self._write_lock:
            self._ser.write(frame)
            self._ser.flush()

    def _deg8_to_raw(self, angles: tuple[float, ...]) -> tuple[int, ...]:
        out = []
        for i in range(len(self._ids)):
            frac = (angles[i] + self.middle[i] + 180.0) / 360.0
            out.append(max(0, min(4095, int(round(frac * 4095)))))
        return tuple(out)

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._ser.read(128)
            except Exception:
                break
            if not data:
                continue
            for frame in self._parser.feed(data):
                self._handle(frame)

    def _handle(self, frame: p.Frame) -> None:
        with self._state_lock:
            self._last_rx = time.monotonic()
        try:
            if frame.type == p.MSG_INFO:
                info = p.parse_info(frame.payload)
                with self._state_lock:
                    self._info = info
                self._info_event.set()
            elif frame.type == p.MSG_TELEMETRY:
                t = p.parse_telemetry(frame.payload)
                with self._state_lock:
                    self._telemetry = t
            elif frame.type == p.MSG_ERROR:
                err = p.parse_error(frame.payload)
                with self._state_lock:
                    self._last_error = err
            elif frame.type == p.MSG_RAW_SCS:
                with self._state_lock:
                    self._raw_response = frame.payload
                self._raw_event.set()
        except p.ProtocolError:
            return
