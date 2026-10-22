"""STM32 控制协议 v1（Pi/PC ↔ STM32F103）。

帧格式：AA 55 | VER | TYPE | SEQ | LEN | PAYLOAD | CRC16_LE
CRC16-CCITT（poly 0x1021, init 0xFFFF）覆盖 VER..PAYLOAD。

协议与 docs/design/stm32_control_protocol.md 一致；固件侧 C 实现见
firmware/stm32/Core/{Inc,Src}/ah_protocol.*。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

SOF0 = 0xAA
SOF1 = 0x55
SOF = bytes([SOF0, SOF1])
VERSION = 1
MAX_PAYLOAD = 64

# ---- 消息类型 ----
MSG_HELLO = 0x01
MSG_INFO = 0x02
MSG_COMMAND = 0x03
MSG_TELEMETRY = 0x04
MSG_RAW_SCS = 0x05
MSG_HEARTBEAT = 0x06
MSG_CONFIG = 0x07
MSG_ERROR = 0x08
MSG_DIRECT_TARGETS = 0x09
MSG_SET_TORQUE = 0x0A

# ---- 命令 ----
CMD_OPEN = 1
CMD_GRASP = 2
CMD_HOLD = 3
CMD_RELEASE = 4
CMD_STOP = 5
CMD_RESET = 6

# ---- 状态 ----
STATE_IDLE = 0
STATE_PREGRASP = 1
STATE_CLOSING = 2
STATE_HOLD = 3
STATE_RELEASING = 4
STATE_PROTECT = 5
STATE_NAMES = ("IDLE", "PREGRASP", "CLOSING", "HOLD", "RELEASING", "PROTECT")

# ---- 故障位 ----
FAULT_WATCHDOG = 1 << 0
FAULT_STOP = 1 << 1
FAULT_OFFLINE = 1 << 2
FAULT_SEQ = 1 << 3
FAULT_OVERLOAD = 1 << 4
FAULT_OVERTEMP = 1 << 5
FAULT_CRC = 1 << 6
FAULT_NAMES = {
    FAULT_WATCHDOG: "watchdog",
    FAULT_STOP: "stop",
    FAULT_OFFLINE: "offline",
    FAULT_SEQ: "seq",
    FAULT_OVERLOAD: "overload",
    FAULT_OVERTEMP: "overtemp",
    FAULT_CRC: "crc",
}

# ---- 遥测 flags ----
FLAG_TORQUE = 1 << 0
FLAG_MOVING = 1 << 1
FLAG_TIMEOUT = 1 << 2
FLAG_GRASPED = 1 << 3

COMMAND_PAYLOAD_SIZE = 61
TELEMETRY_PAYLOAD_SIZE = 63
CONFIG_PAYLOAD_SIZE = 12
INFO_PAYLOAD_SIZE = 8

_CMD_FMT = "<BH8B8H8H8HH"
_TELEM_FMT = "<BBBBBI8H8H8B8BHHH"
_CONFIG_FMT = "<6H"
_INFO_FMT = "<BBBBHH"


class ProtocolError(ValueError):
    """帧格式/长度/CRC 错误。"""


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT-FALSE：poly 0x1021，init 0xFFFF，非反射。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Frame:
    type: int
    seq: int
    payload: bytes


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    if not 0 <= msg_type <= 0xFF:
        raise ProtocolError(f"bad type {msg_type}")
    if not 0 <= seq <= 0xFF:
        raise ProtocolError(f"bad seq {seq}")
    if len(payload) > MAX_PAYLOAD:
        raise ProtocolError(f"payload too long: {len(payload)}")
    body = bytes([VERSION, msg_type, seq, len(payload)]) + payload
    crc = crc16_ccitt(body)
    return SOF + body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class FrameParser:
    """流式解析；容忍分片、前导垃圾与坏帧。"""

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self._buf += data
        out: list[Frame] = []
        while True:
            idx = self._buf.find(SOF)
            if idx < 0:
                if len(self._buf) > 1:
                    del self._buf[:-1]
                break
            if idx:
                del self._buf[:idx]
            if len(self._buf) < 6:
                break
            length = self._buf[5]
            total = length + 8
            if length > MAX_PAYLOAD:
                del self._buf[:2]
                continue
            if len(self._buf) < total:
                break
            raw = bytes(self._buf[:total])
            body = raw[2:6 + length]
            got = raw[6 + length] | (raw[7 + length] << 8)
            if crc16_ccitt(body) != got:
                del self._buf[:2]
                continue
            out.append(Frame(type=raw[3], seq=raw[4], payload=raw[6:6 + length]))
            del self._buf[:total]
        return out


# ---------------- payload 编解码 ----------------

@dataclass(frozen=True)
class Info:
    proto: int
    fw_major: int
    fw_minor: int
    n_servos: int
    telemetry_ms: int
    watchdog_ms: int


@dataclass(frozen=True)
class CommandData:
    cmd: int
    speed_raw: int
    torque_levels: tuple[int, ...]
    pregrasp_raw: tuple[int, ...]
    close_raw: tuple[int, ...]
    load_threshold_raw: tuple[int, ...]
    closing_timeout_ms: int


@dataclass(frozen=True)
class Telemetry:
    state: int
    fault: int
    flags: int
    online_mask: int
    seq_ack: int
    tick_ms: int
    pos_raw: tuple[int, ...]
    load_raw: tuple[int, ...]
    temp_c: tuple[int, ...]
    error_bits: tuple[int, ...]
    command_count: int
    feedback_count: int
    max_age_ms: int

    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state] if 0 <= self.state < len(STATE_NAMES) else f"?{self.state}"

    @property
    def torque_enabled(self) -> bool:
        return bool(self.flags & FLAG_TORQUE)

    @property
    def moving(self) -> bool:
        return bool(self.flags & FLAG_MOVING)

    @property
    def timeout(self) -> bool:
        return bool(self.flags & FLAG_TIMEOUT)

    @property
    def grasped(self) -> bool:
        return bool(self.flags & FLAG_GRASPED)

    @property
    def online(self) -> tuple[bool, ...]:
        return tuple(bool(self.online_mask & (1 << i)) for i in range(8))

    def fault_names(self) -> list[str]:
        return [name for bit, name in FAULT_NAMES.items() if self.fault & bit]


def build_hello(seq: int = 0) -> bytes:
    return encode_frame(MSG_HELLO, seq, bytes([VERSION]))


def build_heartbeat(seq: int) -> bytes:
    return encode_frame(MSG_HEARTBEAT, seq)


def build_set_torque(seq: int, on: bool) -> bytes:
    return encode_frame(MSG_SET_TORQUE, seq, bytes([1 if on else 0]))


def build_direct_targets(seq: int, target_raw: tuple[int, ...], speed_raw: int = 0) -> bytes:
    _check_raw8(target_raw)
    if not 0 <= speed_raw <= 0xFFFF:
        raise ProtocolError("speed_raw out of range")
    return encode_frame(MSG_DIRECT_TARGETS, seq, struct.pack("<H8H", speed_raw, *target_raw))


def parse_direct_targets(payload: bytes) -> tuple[int, tuple[int, ...]]:
    if len(payload) != 18:
        raise ProtocolError(f"DIRECT_TARGETS payload {len(payload)} != 18")
    values = struct.unpack("<H8H", payload)
    return values[0], tuple(values[1:])


def build_config(
    seq: int,
    watchdog_ms: int = 500,
    telemetry_ms: int = 20,
    slot_ms: int = 2,
    pregrasp_ms: int = 300,
    closing_timeout_ms: int = 3000,
    release_ms: int = 500,
) -> bytes:
    values = (watchdog_ms, telemetry_ms, slot_ms, pregrasp_ms, closing_timeout_ms, release_ms)
    if any(not 0 <= v <= 0xFFFF for v in values):
        raise ProtocolError("config value out of u16 range")
    return encode_frame(MSG_CONFIG, seq, struct.pack(_CONFIG_FMT, *values))


def build_raw_scs(seq: int, scs_frame: bytes) -> bytes:
    if not 1 <= len(scs_frame) <= MAX_PAYLOAD:
        raise ProtocolError(f"raw SCS frame length {len(scs_frame)}")
    return encode_frame(MSG_RAW_SCS, seq, scs_frame)


def build_command(
    seq: int,
    cmd: int,
    speed_raw: int = 0,
    torque_levels: tuple[int, ...] = (7,) * 8,
    pregrasp_raw: tuple[int, ...] = (2048,) * 8,
    close_raw: tuple[int, ...] = (2048,) * 8,
    load_threshold_raw: tuple[int, ...] = (384,) * 8,
    closing_timeout_ms: int = 3000,
) -> bytes:
    if cmd not in (CMD_OPEN, CMD_GRASP, CMD_HOLD, CMD_RELEASE, CMD_STOP, CMD_RESET):
        raise ProtocolError(f"bad cmd {cmd}")
    if not 0 <= speed_raw <= 0xFFFF:
        raise ProtocolError("speed_raw out of range")
    if len(torque_levels) != 8 or any(not 0 <= v <= 15 for v in torque_levels):
        raise ProtocolError("torque_levels must be 8 x 0..15")
    _check_raw8(pregrasp_raw)
    _check_raw8(close_raw)
    _check_raw8(load_threshold_raw)
    if not 0 <= closing_timeout_ms <= 0xFFFF:
        raise ProtocolError("closing_timeout_ms out of range")
    payload = struct.pack(
        _CMD_FMT, cmd, speed_raw, *torque_levels,
        *pregrasp_raw, *close_raw, *load_threshold_raw, closing_timeout_ms,
    )
    assert len(payload) == COMMAND_PAYLOAD_SIZE
    return encode_frame(MSG_COMMAND, seq, payload)


def parse_info(payload: bytes) -> Info:
    if len(payload) != INFO_PAYLOAD_SIZE:
        raise ProtocolError(f"INFO payload {len(payload)} != {INFO_PAYLOAD_SIZE}")
    return Info(*struct.unpack(_INFO_FMT, payload))


def parse_command(payload: bytes) -> CommandData:
    if len(payload) != COMMAND_PAYLOAD_SIZE:
        raise ProtocolError(f"COMMAND payload {len(payload)} != {COMMAND_PAYLOAD_SIZE}")
    values = struct.unpack(_CMD_FMT, payload)
    return CommandData(
        cmd=values[0],
        speed_raw=values[1],
        torque_levels=tuple(values[2:10]),
        pregrasp_raw=tuple(values[10:18]),
        close_raw=tuple(values[18:26]),
        load_threshold_raw=tuple(values[26:34]),
        closing_timeout_ms=values[34],
    )


def parse_telemetry(payload: bytes) -> Telemetry:
    if len(payload) != TELEMETRY_PAYLOAD_SIZE:
        raise ProtocolError(f"TELEMETRY payload {len(payload)} != {TELEMETRY_PAYLOAD_SIZE}")
    v = struct.unpack(_TELEM_FMT, payload)
    i = 0
    state, fault, flags, online_mask, seq_ack, tick_ms = v[0:6]
    i = 6
    pos = tuple(v[i:i + 8]); i += 8
    load = tuple(v[i:i + 8]); i += 8
    temp = tuple(v[i:i + 8]); i += 8
    err = tuple(v[i:i + 8]); i += 8
    cmd_count, fb_count, age = v[i:i + 3]
    return Telemetry(
        state=state, fault=fault, flags=flags, online_mask=online_mask,
        seq_ack=seq_ack, tick_ms=tick_ms, pos_raw=pos, load_raw=load,
        temp_c=temp, error_bits=err, command_count=cmd_count,
        feedback_count=fb_count, max_age_ms=age,
    )


def parse_error(payload: bytes) -> tuple[int, int]:
    if len(payload) != 2:
        raise ProtocolError(f"ERROR payload {len(payload)} != 2")
    return payload[0], payload[1]


def _check_raw8(values: tuple[int, ...]) -> None:
    if len(values) != 8 or any(not 0 <= v <= 0xFFFF for v in values):
        raise ProtocolError("expected 8 x u16 raw values")
