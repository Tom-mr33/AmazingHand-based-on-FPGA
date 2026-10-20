"""无硬件测试：McuLink 帧封装 + McuBackend FSM/初始化。

不依赖 pyserial、不连硬件：用 FakeSerial / FakeLink 注入。
运行： python host/tests/test_mcu_link.py
"""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app.control import scs_protocol as sp  # noqa: E402
from app.control.backend_mcu import McuBackend  # noqa: E402
from app.control.mcu_link import McuLink, McuLinkError  # noqa: E402
from app.control.servo_base import FsmState, GraspPlan  # noqa: E402


def _response_frame(
    servo_id: int = 1,
    params: bytes = bytes([0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x2A]),
) -> bytes:
    length = len(params) + 2
    cs = sp.checksum(servo_id, length, 0, *params)
    return bytes([0xFF, 0xFF, servo_id, length, 0, *params, cs])


class FakeSerial:
    """最小 pyserial 替身：收到 [len][frame] 后返回预设响应。"""

    def __init__(self, *args, **kwargs):
        self.writes: list[bytes] = []
        self._buf = bytearray()

    def write(self, data: bytes) -> int:
        self.writes.append(bytes(data))
        if not data:
            return 0
        n = data[0]
        frame = data[1 : 1 + n]
        if n == 1 and frame == b"\x00":
            resp = b""
        else:
            resp = _response_frame()
        self._buf = bytearray(bytes([len(resp)]) + resp)
        return len(data)

    def flush(self) -> None:
        pass

    def reset_input_buffer(self) -> None:
        pass

    def read(self, n: int) -> bytes:
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def close(self) -> None:
        pass


class FakeLink:
    def __init__(self, responses=None, auto_response: bool = True):
        self.responses = list(responses or [])
        self.auto_response = auto_response
        self.frames: list[bytes] = []
        self.opened = False

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    @property
    def is_open(self) -> bool:
        return self.opened

    def transact(self, frame: bytes, expect_reply: bool = True):
        self.frames.append(bytes(frame))
        if not expect_reply:
            return None
        if self.responses:
            return self.responses.pop(0)
        return _response_frame() if self.auto_response else None


class McuLinkTests(unittest.TestCase):
    def setUp(self):
        self._old_serial = sys.modules.get("serial")
        sys.modules["serial"] = types.SimpleNamespace(Serial=FakeSerial)
        self.link = McuLink("FAKE", timeout_s=0.2)
        self.link.open()

    def tearDown(self):
        self.link.close()
        if self._old_serial is None:
            sys.modules.pop("serial", None)
        else:
            sys.modules["serial"] = self._old_serial

    def test_framed_request_and_response(self):
        resp = self.link.transact(sp.build_ping(1))
        self.assertIsNotNone(resp)
        parsed = sp.parse_response(resp)
        self.assertEqual(parsed.servo_id, 1)
        self.assertEqual(self.link._ser.writes[0][0], len(sp.build_ping(1)))

    def test_zero_length_means_no_response(self):
        self.assertIsNone(self.link.transact(b"\x00"))

    def test_reject_oversize_frame(self):
        with self.assertRaises(McuLinkError):
            self.link.transact(bytes(65))


class McuBackendTests(unittest.TestCase):
    def make_backend(self):
        link = FakeLink()
        backend = McuBackend(link=link, middle_deg=(0.0,) * 8,
                             servo_ids=range(1, 9), feedback_period_s=0.0)
        return backend, link

    def test_poll_before_start_is_safe(self):
        backend, _ = self.make_backend()
        snap = backend.poll_fsm()
        self.assertEqual(snap.state, FsmState.IDLE)
        self.assertFalse(snap.fault)
        self.assertEqual(len(snap.feedback), 8)

    def test_start_pre_grasp_closing_timeout(self):
        backend, link = self.make_backend()
        plan = GraspPlan(angles_deg=(0.0,) * 8)
        backend.start_grasp_fsm(plan)
        self.assertEqual(backend._fsm, FsmState.PREGRASP)
        self.assertTrue(link.frames)  # torque + pregrasp frames sent
        backend.tick(0.31)
        self.assertEqual(backend._fsm, FsmState.CLOSING)
        backend.tick(3.1)
        self.assertEqual(backend._fsm, FsmState.HOLD)
        self.assertTrue(backend.poll_fsm().timeout)

    def test_offline_feedback_sets_fault(self):
        link = FakeLink(auto_response=False)
        backend = McuBackend(link=link, middle_deg=(0.0,) * 8,
                             servo_ids=range(1, 9), feedback_period_s=0.0)
        snap = backend.poll_fsm()
        self.assertTrue(snap.fault)
        self.assertTrue(any(not f.online for f in snap.feedback))

    def test_release_and_estop(self):
        backend, _ = self.make_backend()
        backend.start_grasp_fsm(GraspPlan(angles_deg=(0.0,) * 8))
        backend.tick(0.31)
        backend.release()
        self.assertEqual(backend._fsm, FsmState.RELEASING)
        backend.tick(0.6)
        self.assertEqual(backend._fsm, FsmState.IDLE)
        backend.emergency_stop()
        self.assertEqual(backend._fsm, FsmState.PROTECT)
        self.assertTrue(backend.poll_fsm().fault)


if __name__ == "__main__":
    unittest.main(verbosity=2)
