"""STM32 透传链路（PC/树莓派 ↔ STM32F103）。

固件见 firmware/stm32：STM32 把 PC 下发的原始 SCS 帧泵到 1 Mbps 单线
半双工总线，并把舵机响应原样回传。本模块只负责 PC 侧的帧长前缀传输：

    下行： [len u8][len 字节 SCS 请求帧]      len = 1..64
    上行： [len u8][len 字节 SCS 响应]        len = 0 表示总线静默

SCS 协议语义仍由 app.control.scs_protocol 负责，STM32 固件零协议逻辑。
"""

from __future__ import annotations

import time

# 与 firmware/stm32/Core/Src/main.c 保持一致
MCU_BAUD = 115_200
FRAME_MAX = 64
RESPONSE_MAX = 80


class McuLinkError(RuntimeError):
    """串口打不开、帧超长或响应不完整。"""


class McuLink:
    """帧长前缀传输层；不解析 SCS 协议。"""

    def __init__(self, port: str, baud: int = MCU_BAUD, timeout_s: float = 0.05):
        self.port = port
        self.baud = int(baud)
        self.timeout_s = float(timeout_s)
        self._ser = None

    def open(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise McuLinkError("backend_mcu 需要 pyserial：pip install pyserial") from exc
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
        except Exception as exc:
            raise McuLinkError(f"无法打开串口 {self.port}: {exc}") from exc
        time.sleep(0.2)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    @property
    def is_open(self) -> bool:
        return self._ser is not None

    def __enter__(self) -> "McuLink":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def transact(self, frame: bytes, expect_reply: bool = True) -> bytes | None:
        """发送一条 SCS 帧；返回原始响应帧，或 None（无响应/静默）。"""
        if self._ser is None:
            raise McuLinkError("链路未打开")
        if not 1 <= len(frame) <= FRAME_MAX:
            raise McuLinkError(f"SCS 帧长度非法：{len(frame)}（1..{FRAME_MAX}）")

        self._ser.reset_input_buffer()
        self._ser.write(bytes([len(frame)]) + frame)
        self._ser.flush()
        if not expect_reply:
            return None

        head = self._read_exact(1)
        if head is None:
            return None
        n = head[0]
        if n == 0:
            return None
        if n > RESPONSE_MAX:
            raise McuLinkError(f"STM32 响应长度非法：{n}")

        body = self._read_exact(n)
        if body is None or len(body) != n:
            raise McuLinkError(f"STM32 响应不完整：{0 if body is None else len(body)}/{n}")
        return body

    def _read_exact(self, n: int) -> bytes | None:
        assert self._ser is not None
        buf = bytearray()
        deadline = time.perf_counter() + self.timeout_s
        while len(buf) < n:
            chunk = self._ser.read(n - len(buf))
            if chunk:
                buf += chunk
            if time.perf_counter() > deadline:
                break
        return bytes(buf) if len(buf) == n else None
