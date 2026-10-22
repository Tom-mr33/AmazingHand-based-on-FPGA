"""STM32 控制协议编解码测试（无硬件）。

运行： python host/tests/test_stm32_protocol.py
"""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from app.control import stm32_protocol as p  # noqa: E402


class CrcTests(unittest.TestCase):
    def test_known_vector(self):
        self.assertEqual(p.crc16_ccitt(b"123456789"), 0x29B1)

    def test_empty(self):
        self.assertEqual(p.crc16_ccitt(b""), 0xFFFF)


class FrameTests(unittest.TestCase):
    def test_roundtrip(self):
        raw = p.encode_frame(p.MSG_HEARTBEAT, 7)
        frames = p.FrameParser().feed(raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, p.MSG_HEARTBEAT)
        self.assertEqual(frames[0].seq, 7)
        self.assertEqual(frames[0].payload, b"")

    def test_fragmented_feed(self):
        raw = p.encode_frame(p.MSG_HELLO, 1, bytes([p.VERSION]))
        parser = p.FrameParser()
        self.assertEqual(parser.feed(raw[:3]), [])
        self.assertEqual(parser.feed(raw[3:]), [p.Frame(p.MSG_HELLO, 1, bytes([p.VERSION]))])

    def test_leading_garbage(self):
        raw = p.encode_frame(p.MSG_HEARTBEAT, 2)
        frames = p.FrameParser().feed(b"\x00\xAA\x11" + raw)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].seq, 2)

    def test_bad_crc_dropped(self):
        raw = bytearray(p.encode_frame(p.MSG_HEARTBEAT, 3))
        raw[-1] ^= 0xFF
        self.assertEqual(p.FrameParser().feed(bytes(raw)), [])

    def test_oversize_payload(self):
        with self.assertRaises(p.ProtocolError):
            p.encode_frame(p.MSG_RAW_SCS, 0, bytes(65))


class PayloadTests(unittest.TestCase):
    def test_info(self):
        raw = struct.pack("<BBBBHH", 1, 1, 2, 8, 20, 500)
        info = p.parse_info(raw)
        self.assertEqual((info.proto, info.n_servos, info.telemetry_ms), (1, 8, 20))

    def test_command_roundtrip(self):
        frame = p.build_command(
            seq=9, cmd=p.CMD_GRASP, speed_raw=300,
            torque_levels=(7,) * 8,
            pregrasp_raw=tuple(range(100, 108)),
            close_raw=tuple(range(200, 208)),
            load_threshold_raw=(384,) * 8,
            closing_timeout_ms=3000,
        )
        frames = p.FrameParser().feed(frame)
        self.assertEqual(len(frames), 1)
        self.assertEqual(len(frames[0].payload), p.COMMAND_PAYLOAD_SIZE)
        cmd = p.parse_command(frames[0].payload)
        self.assertEqual(cmd.cmd, p.CMD_GRASP)
        self.assertEqual(cmd.pregrasp_raw, tuple(range(100, 108)))
        self.assertEqual(cmd.close_raw, tuple(range(200, 208)))
        self.assertEqual(cmd.closing_timeout_ms, 3000)

    def test_command_bad_torque(self):
        with self.assertRaises(p.ProtocolError):
            p.build_command(0, p.CMD_GRASP, torque_levels=(16,) * 8)

    def test_telemetry_roundtrip(self):
        pos = tuple(range(2048, 2056))
        load = tuple(range(0, 8))
        temp = tuple(range(20, 28))
        err = (0,) * 8
        payload = struct.pack(
            "<BBBBBI8H8H8B8BHHH",
            p.STATE_CLOSING, p.FAULT_OVERLOAD, p.FLAG_TORQUE | p.FLAG_MOVING,
            0x0F, 42, 123456,
            *pos, *load, *temp, *err, 7, 99, 12,
        )
        self.assertEqual(len(payload), p.TELEMETRY_PAYLOAD_SIZE)
        t = p.parse_telemetry(payload)
        self.assertEqual(t.state_name, "CLOSING")
        self.assertTrue(t.torque_enabled)
        self.assertTrue(t.moving)
        self.assertEqual(t.pos_raw, pos)
        self.assertEqual(t.online_mask, 0x0F)
        self.assertEqual(t.fault_names(), ["overload"])
        self.assertEqual(t.command_count, 7)
        self.assertEqual(t.max_age_ms, 12)

    def test_config_and_direct(self):
        cfg = p.build_config(1, watchdog_ms=500, telemetry_ms=20)
        frames = p.FrameParser().feed(cfg)
        self.assertEqual(frames[0].type, p.MSG_CONFIG)
        direct = p.build_direct_targets(2, (2048,) * 8, speed_raw=300)
        frames = p.FrameParser().feed(direct)
        self.assertEqual(frames[0].type, p.MSG_DIRECT_TARGETS)
        self.assertEqual(len(frames[0].payload), 18)
        speed, targets = p.parse_direct_targets(frames[0].payload)
        self.assertEqual(speed, 300)
        self.assertEqual(targets, (2048,) * 8)

    def test_raw_scs(self):
        frame = p.build_raw_scs(3, b"\xFF\xFF\x01\x02\x01\xFB")
        frames = p.FrameParser().feed(frame)
        self.assertEqual(frames[0].payload, b"\xFF\xFF\x01\x02\x01\xFB")


if __name__ == "__main__":
    unittest.main(verbosity=2)
