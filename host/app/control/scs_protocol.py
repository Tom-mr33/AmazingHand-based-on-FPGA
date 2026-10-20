"""Feetech SCS 系列串行协议 —— 组包/解包/校验和（golden model）。

本项目定位
==========
这是全项目控制链路的**协议行为参考模型（golden model）**：

- ``host/app/control/backend_serial.py`` : 用它组包/解包真实舵机（USB-TTL）
- ``host/app/control/backend_mcu.py``    : 经 STM32 透传访问 1Mbps SCS 总线
- ``host/app/control/backend_sim.py``    : 用它驱动 ``scs_sim.VirtualServoBus``
- ``firmware/stm32``                     : 字节泵（协议语义仍在本文件）
- ``tools/blink_servo.py`` / ``probe_servos.py`` : 复用组包函数

协议帧格式（SCS 系列，与 Dynamixel protocol 1.0 同构）::

    Instruction Packet: FF FF ID LENGTH INSTRUCTION PARAM... CHECKSUM
    Status Packet:      FF FF ID LENGTH ERROR      PARAM... CHECKSUM

    LENGTH = LENGTH 字段之后到帧尾的字节数 = len(PARAM) + 2（指令/错误 + 校验和）
    CHECKSUM = ~(ID + LENGTH + INSTRUCTION/ERROR + PARAM...) & 0xFF

已知手工算例（本文件冒烟测试的锚点）::

    WRITE(ID=1, addr=0x2A, data=00 08)
      -> FF FF 01 05 03 2A 00 08 C4     (01+05+03+2A+00+08=0x3B, ~0x3B=0xC4)
    READ (ID=1, addr=0x3A, n=6)
      -> FF FF 01 04 02 3A 06 B8        (01+04+02+3A+06=0x47, ~0x47=0xB8)

约定
====
- 所有多字节寄存器均为 **小端**（低字节在前），与 Feetech 官方一致。
- 地址表集中在 :class:`Reg`，个别地址以 "TODO(实机校验)" 标注：
  无实机前按 SCS 系列公开资料填写，实机用 ``probe_servos.py`` / ``blink_servo.py``
  核对后定稿（待定项 1/2 同批解决）。
- 单位换算与 rustypot ``Scs0009PyController`` 行为对齐：
  位置 raw 0..4095 对应 -pi..+pi（12bit 磁编码）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

# --------------------------------------------------------------------------
# 协议常量
# --------------------------------------------------------------------------

HEADER0 = 0xFF
HEADER1 = 0xFF
BROADCAST_ID = 0xFE  # 广播地址（SYNC_WRITE 目标；广播 PING 在真实总线会碰撞）

# 指令码
INS_PING = 0x01
INS_READ = 0x02
INS_WRITE = 0x03
INS_REG_WRITE = 0x04
INS_ACTION = 0x05
INS_RESET = 0x06
INS_SYNC_WRITE = 0x83

# 状态包 ERROR 位（bit 编号，与 Dynamixel 1.0 / Feetech SCS 一致）
ERR_INSTRUCTION = 1 << 0  # 未定义指令
ERR_OVERLOAD = 1 << 1  # 负载超限 / 堵转（力控状态机的关键信号）
ERR_CHECKSUM = 1 << 2  # （主机侧）收包校验错
ERR_RANGE = 1 << 3  # 指令值超行程
ERR_OVERHEAT = 1 << 4  # 过温（保护逻辑关键信号）
ERR_ANGLE_LIMIT = 1 << 5  # 超角度限制
ERR_INPUT_VOLTAGE = 1 << 6  # 输入电压异常

# 协议极限
MAX_ID = 0xFC  # 0xFD 保留、0xFE 广播、0xFF 帧头禁用
MIN_FRAME_LEN = 6  # FF FF ID 02 ERR CS（PING 响应最短帧）


class ProtocolError(ValueError):
    """帧解析失败（长度非法 / 校验和错误 / 无帧头）。"""


# --------------------------------------------------------------------------
# 寄存器地址表（SCS0009）
# --------------------------------------------------------------------------


class Reg:
    """SCS0009 寄存器地址与宽度（字节）。

    TODO(实机校验)：下表按 Feetech SCS 系列公开资料与上游 rustypot
    行为填写；上板后用 FD 调试工具逐项核对，修订只改本类。
    """

    # ---- EEPROM ----
    MODEL_NUMBER = (0x00, 2)
    FIRMWARE_VERSION = (0x02, 1)
    ID = (0x03, 1)
    BAUD_RATE = (0x04, 1)
    RETURN_DELAY_TIME = (0x05, 1)
    MIN_ANGLE_LIMIT = (0x07, 2)
    MAX_ANGLE_LIMIT = (0x09, 2)
    MAX_TEMPERATURE_LIMIT = (0x0B, 1)
    MAX_VOLTAGE_LIMIT = (0x0C, 1)
    MIN_VOLTAGE_LIMIT = (0x0D, 1)
    MAX_TORQUE = (0x0E, 2)
    STATUS_RETURN_LEVEL = (0x10, 1)
    ALARM_LED = (0x11, 1)
    ALARM_SHUTDOWN = (0x12, 1)

    # ---- RAM ----
    TORQUE_SWITCH = (0x28, 1)  # 官方 Demo: 1=On / 2=Off / 3=Free
    GOAL_POSITION = (0x2A, 2)
    GOAL_TIME = (0x2C, 2)
    RUNNING_SPEED = (0x2E, 2)
    TORQUE_LIMIT = (0x30, 2)
    PRESENT_POSITION = (0x3A, 2)
    PRESENT_SPEED = (0x3C, 2)
    PRESENT_LOAD = (0x3E, 2)
    PRESENT_VOLTAGE = (0x40, 1)
    PRESENT_TEMPERATURE = (0x41, 1)
    REGISTERED_INSTRUCTION = (0x42, 1)
    MOVING = (0x43, 1)

    # 连读视图：一次 READ 取回位置/速度/负载/电压/温度
    FEEDBACK_ADDR = PRESENT_POSITION[0]  # 0x3A
    FEEDBACK_LEN = 8  # pos(2)+speed(2)+load(2)+voltage(1)+temp(1) = 0x3A..0x41


# --------------------------------------------------------------------------
# 单位换算（与 rustypot Scs0009PyController 对齐，待实机校验）
# --------------------------------------------------------------------------

POSITION_STEPS = 4095  # 12bit
POSITION_RANGE_RAD = 2.0 * math.pi  # raw 全量程对应角度


def rad_to_raw(rad: float) -> int:
    """弧度 [-pi, +pi] -> 位置 raw [0, 4095]（四舍五入并夹紧）。"""
    raw = round((rad + math.pi) / POSITION_RANGE_RAD * POSITION_STEPS)
    return max(0, min(POSITION_STEPS, raw))


def raw_to_rad(raw: int) -> float:
    """位置 raw [0, 4095] -> 弧度 [-pi, +pi]。"""
    raw = max(0, min(POSITION_STEPS, raw))
    return raw / POSITION_STEPS * POSITION_RANGE_RAD - math.pi


# --------------------------------------------------------------------------
# 组包
# --------------------------------------------------------------------------


def checksum(servo_id: int, length: int, *body: int) -> int:
    """CHECKSUM = ~(ID + LENGTH + body...) & 0xFF。"""
    total = (servo_id + length + sum(body)) & 0xFF
    return ~total & 0xFF


def _frame(servo_id: int, instruction: int, params: bytes) -> bytes:
    """按帧格式组装 Instruction Packet。"""
    if not 0 <= servo_id <= 0xFF:
        raise ValueError(f"servo_id out of range: {servo_id}")
    length = len(params) + 2  # instruction + checksum
    cs = checksum(servo_id, length, instruction, *params)
    return bytes([HEADER0, HEADER1, servo_id, length, instruction, *params, cs])


def build_ping(servo_id: int) -> bytes:
    return _frame(servo_id, INS_PING, b"")


def build_read(servo_id: int, addr: int, length: int) -> bytes:
    """READ 指令：params = [addr, length]。"""
    if not 1 <= length <= 255:
        raise ValueError(f"read length out of range: {length}")
    return _frame(servo_id, INS_READ, bytes([addr, length]))


def build_write(servo_id: int, addr: int, data: bytes) -> bytes:
    """WRITE 指令：params = [addr, data...]。"""
    if not data:
        raise ValueError("write data must be non-empty")
    return _frame(servo_id, INS_WRITE, bytes([addr, *data]))


def build_reg_write(servo_id: int, addr: int, data: bytes) -> bytes:
    """REG_WRITE 指令（配合 ACTION 广播触发）。"""
    if not data:
        raise ValueError("write data must be non-empty")
    return _frame(servo_id, INS_REG_WRITE, bytes([addr, *data]))


def build_action() -> bytes:
    """ACTION 广播：一次性触发所有已 REG_WRITE 的动作。"""
    return _frame(BROADCAST_ID, INS_ACTION, b"")


def build_sync_write(addr: int, data_len: int, table: Mapping[int, bytes]) -> bytes:
    """SYNC_WRITE 广播（0x83）：一条指令写多舵机，**无响应**。

    帧内 params = [addr, data_len, (id, data...) * N]，
    其中 ``data_len`` 为每舵机数据字节数（须一致）。
    这是批量下发 8 舵机目标的核心指令。
    """
    if not table:
        raise ValueError("sync_write table must be non-empty")
    params = bytearray([addr, data_len])
    for servo_id in sorted(table):
        data = table[servo_id]
        if len(data) != data_len:
            raise ValueError(
                f"servo {servo_id}: data len {len(data)} != declared {data_len}"
            )
        params += bytes([servo_id, *data])
    # 注意：SYNC_WRITE 固定发广播 ID，Length 按广播 ID 计算
    length = len(params) + 2
    cs = checksum(BROADCAST_ID, length, INS_SYNC_WRITE, *params)
    return bytes([HEADER0, HEADER1, BROADCAST_ID, length, INS_SYNC_WRITE, *params, cs])


# 便捷封装：写 16 位小端寄存器
def build_write_u16(servo_id: int, addr: int, value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"u16 out of range: {value}")
    return build_write(servo_id, addr, bytes([value & 0xFF, (value >> 8) & 0xFF]))


# --------------------------------------------------------------------------
# 解包
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Response:
    """解析后的 Status Packet。"""

    servo_id: int
    error: int  # ERR_* 位或
    params: bytes

    def param_u16(self, index: int) -> int:
        """取第 index 个 16 位小端参数。"""
        return self.params[index] | (self.params[index + 1] << 8)


def locate_frame(buf: bytes) -> int:
    """返回第一个合法帧头 0xFF 0xFF 的下标；找不到返回 -1。"""
    for i in range(len(buf) - 1):
        if buf[i] == HEADER0 and buf[i + 1] == HEADER1:
            return i
    return -1


def parse_response(buf: bytes) -> Response:
    """解析 Status Packet（容忍前导垃圾字节，如 UART 缓冲残渣）。

    校验和错误 / 帧长非法抛 :class:`ProtocolError`。
    """
    start = locate_frame(buf)
    if start < 0:
        raise ProtocolError("no frame header found")
    if len(buf) - start < MIN_FRAME_LEN:
        raise ProtocolError("frame too short")
    servo_id = buf[start + 2]
    length = buf[start + 3]
    end = start + 4 + length  # FF FF ID LEN + LEN 字节
    if length < 2 or len(buf) < end:
        raise ProtocolError(f"bad length {length} for {len(buf) - start} bytes")
    error = buf[start + 4]
    params = bytes(buf[start + 5 : end - 1])
    cs = buf[end - 1]
    expect = checksum(servo_id, length, error, *params)
    if cs != expect:
        raise ProtocolError(
            f"checksum mismatch: got 0x{cs:02X}, expect 0x{expect:02X}"
        )
    return Response(servo_id=servo_id, error=error, params=params)


def is_broadcast(frame: bytes) -> bool:
    """判断 Instruction Packet 是否为广播帧（ID=0xFE）。"""
    return len(frame) > 2 and frame[2] == BROADCAST_ID
