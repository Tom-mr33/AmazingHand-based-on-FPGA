"""backend_stm32 无硬件测试：FakeSerial 模拟 INFO/TELEMETRY。"""

from __future__ import annotations

import struct
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app.control import stm32_protocol as p  # noqa: E402
from app.control.backend_stm32 import Stm32Backend  # noqa: E402
from app.control.servo_base import FsmState, GraspPlan  # noqa: E402


def info_payload() -> bytes:
    return struct.pack("<BBBBHH", 1, 1, 0, 8, 20, 500)


def telemetry_payload(state: int, fault: int = 0, flags: int = 0) -> bytes:
    return struct.pack(
        "<BBBBBI8H8H8B8BHHH",
        state, fault, flags, 0xFF, 0, 123456,
        *([2048] * 8), *([0] * 8), *([25] * 8), *([0] * 8),
        1, 1, 0,
    )


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self._in = bytearray()
        self._lock = threading.Lock()
        self._parser = p.FrameParser()
        self.writes: list[bytes] = []
        self.closed = False

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        for frame in self._parser.feed(bytes(data)):
            self._on_frame(frame)
        return len(data)

    def _enqueue(self, frame: bytes) -> None:
        with self._lock:
            self._in += frame

    def _on_frame(self, frame: p.Frame) -> None:
        if frame.type == p.MSG_HELLO:
            self._enqueue(p.encode_frame(p.MSG_INFO, 0, info_payload()))
        elif frame.type == p.MSG_HEARTBEAT:
            self._enqueue(p.encode_frame(p.MSG_TELEMETRY, 0,
                                          telemetry_payload(p.STATE_IDLE, flags=p.FLAG_TORQUE)))
        elif frame.type == p.MSG_COMMAND:
            cmd = p.parse_command(frame.payload).cmd
            if cmd == p.CMD_GRASP:
                self._enqueue(p.encode_frame(p.MSG_TELEMETRY, 0,
                                              telemetry_payload(p.STATE_CLOSING, flags=p.FLAG_TORQUE | p.FLAG_MOVING)))
            elif cmd == p.CMD_STOP:
                self._enqueue(p.encode_frame(p.MSG_TELEMETRY, 0,
                                              telemetry_payload(p.STATE_PROTECT, p.FAULT_STOP)))
            elif cmd == p.CMD_RELEASE:
                self._enqueue(p.encode_frame(p.MSG_TELEMETRY, 0,
                                              telemetry_payload(p.STATE_RELEASING, flags=p.FLAG_TORQUE | p.FLAG_MOVING)))
        elif frame.type == p.MSG_DIRECT_TARGETS:
            self._enqueue(p.encode_frame(p.MSG_TELEMETRY, 0,
                                          telemetry_payload(p.STATE_IDLE, flags=p.FLAG_TORQUE)))
        elif frame.type == p.MSG_SET_TORQUE:
            self._enqueue(p.encode_frame(p.MSG_TELEMETRY, 0,
                                          telemetry_payload(p.STATE_IDLE, flags=0)))
        elif frame.type == p.MSG_RAW_SCS:
            self._enqueue(p.encode_frame(p.MSG_RAW_SCS, 0, b"\xAA"))

    def read(self, n: int) -> bytes:
        with self._lock:
            out = bytes(self._in[:n])
            del self._in[:n]
        if not out:
            time.sleep(0.001)
        return out

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        with self._lock:
            self._in.clear()

    def close(self) -> None:
        self.closed = True


class Stm32BackendTests(unittest.TestCase):
    def setUp(self):
        self.fake = FakeSerial()
        self.backend = Stm32Backend(serial_factory=lambda *a, **k: self.fake)
        self.backend.open()

    def tearDown(self):
        self.backend.close()

    def wait_state(self, state, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snap = self.backend.poll_fsm()
            if snap.state == state:
                return snap
            time.sleep(0.005)
        return self.backend.poll_fsm()

    def test_info_and_heartbeat(self):
        self.backend._last_hb = time.monotonic() - 1.0
        self.backend.tick(0.0)
        snap = self.wait_state(FsmState.IDLE)
        self.assertEqual(snap.state, FsmState.IDLE)
        self.assertFalse(snap.fault)
        self.assertEqual(len(snap.feedback), 8)
        self.assertTrue(all(f.online for f in snap.feedback))

    def test_grasp_and_stop(self):
        self.backend.start_grasp_fsm(GraspPlan(angles_deg=(0.0,) * 8, pregrasp_deg=(0.0,) * 8))
        snap = self.wait_state(FsmState.CLOSING)
        self.assertEqual(snap.state, FsmState.CLOSING)
        self.assertTrue(snap.busy)
        self.backend.emergency_stop()
        snap = self.wait_state(FsmState.PROTECT)
        self.assertTrue(snap.fault)
        self.assertEqual(snap.err_bitmap & p.FAULT_STOP, p.FAULT_STOP)

    def test_stale_telemetry_fault(self):
        self.backend._last_hb = time.monotonic() - 1.0
        self.backend.tick(0.0)
        self.wait_state(FsmState.IDLE)
        self.backend._last_rx = time.monotonic() - 10.0
        snap = self.backend.poll_fsm()
        self.assertTrue(snap.fault)
        self.assertEqual(snap.err_bitmap, p.FAULT_WATCHDOG)

    def test_raw_scs(self):
        self.assertEqual(self.backend.raw_scs(b"\x01"), b"\xAA")


if __name__ == "__main__":
    unittest.main(verbosity=2)
