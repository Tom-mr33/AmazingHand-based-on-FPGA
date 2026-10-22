"""舵机总线探测 —— 拓扑/存活/型号/回读（实物到手第一件事）。

支持两种下行通道：

1. --via mcu  (默认，树莓派/PC + STM32 透传固件)
   Pi/USB-TTL @115200  --[len][SCS frame]-->  STM32  --1Mbps 半双工-->  舵机
   接线：总线 DATA -> STM32 PA9（HDSEL 单线）；GND 共地；舵机 5V 独立供电。

2. --via raw  (USB-TTL / FD 调试板直连总线)
   PC @1Mbps 直接收发 SCS 帧；TX 串 1kΩ 合并半双工（或使用总线驱动板）。

扫描 ID 1..20，报告存活、型号/固件、位置/负载/温度回读，并给出拓扑判定：
  · 每线 1 个 ID = 8 路独立
  · 每线 2 个 ID = 4 路（每指 1 路，2 舵机并联）
  · 每线 8 个 ID = 单总线（官方默认拓扑）

用法::

    python tools/probe_servos.py --port /dev/serial0 --via mcu
    python tools/probe_servos.py --port /dev/ttyUSB0 --via raw
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "host"))

from app.control import scs_protocol as sp  # noqa: E402
from app.control.mcu_link import McuLink  # noqa: E402

SCAN_IDS = range(1, 21)


class RawTransport:
    """USB-TTL/FD 直连总线：裸 SCS 帧 @1Mbps。"""

    def __init__(self, port: str, baud: int = 1_000_000):
        import serial

        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def transact(self, frame: bytes) -> bytes | None:
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        time.sleep(len(frame) * 10e-6)  # 半双工回听/换向
        deadline = time.perf_counter() + 0.05
        buf = bytearray()
        while time.perf_counter() < deadline:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
            elif buf:
                break
        return bytes(buf) or None


class McuTransport:
    """STM32 透传：帧长前缀 @115200。"""

    def __init__(self, port: str, baud: int = 115_200):
        self.link = McuLink(port, baud=baud, timeout_s=0.05)
        self.link.open()

    def close(self) -> None:
        self.link.close()

    def transact(self, frame: bytes) -> bytes | None:
        return self.link.transact(frame)


def ping(t, sid: int) -> dict | None:
    resp = t.transact(sp.build_ping(sid))
    if resp is None:
        return None
    try:
        r = sp.parse_response(resp)
    except sp.ProtocolError:
        return None
    return {
        "model": r.param_u16(0) if len(r.params) >= 2 else 0,
        "fw": r.params[2] if len(r.params) >= 3 else 0,
    }


def read_feedback(t, sid: int) -> bytes | None:
    resp = t.transact(sp.build_read(sid, sp.Reg.FEEDBACK_ADDR, sp.Reg.FEEDBACK_LEN))
    if resp is None:
        return None
    try:
        r = sp.parse_response(resp)
    except sp.ProtocolError:
        return None
    return r.params


def main() -> int:
    parser = argparse.ArgumentParser(description="SCS0009 总线探测")
    parser.add_argument("--port", required=True, help="Pi /dev/serial0、USB-TTL /dev/ttyUSB0、Windows COMx")
    parser.add_argument("--via", choices=["mcu", "raw"], default="mcu",
                        help="mcu=STM32 透传（默认）；raw=USB-TTL 直连")
    parser.add_argument("--baud", type=int, default=None,
                        help="默认：mcu=115200，raw=1000000")
    args = parser.parse_args()

    baud = args.baud or (115_200 if args.via == "mcu" else 1_000_000)
    try:
        t = McuTransport(args.port, baud) if args.via == "mcu" else RawTransport(args.port, baud)
    except Exception as exc:
        print(f"打开串口失败：{exc}")
        return 1

    print(f"port={args.port} via={args.via} baud={baud}")
    print("扫描 ID 1..20（PING）...")
    alive: dict[int, dict] = {}
    try:
        for sid in SCAN_IDS:
            info = ping(t, sid)
            if info:
                alive[sid] = info
                print(f"  ID {sid:2d}: 存活  型号raw={info['model']} 固件={info['fw']}")
        if not alive:
            print("未发现任何舵机——检查：①共地 ②PA9/DATA ③独立 5V ④ID/波特率")
            return 1

        ids = sorted(alive)
        print(f"\n存活 {len(ids)} 个：{ids}")
        if len(ids) == 1:
            topo = "单舵机（测试单指）"
        elif len(ids) == 2:
            topo = "2 舵机/线 → 每指 1 路（4 总线拓扑）"
        elif len(ids) >= 8:
            topo = "8 舵机/线 → 单总线拓扑（官方默认）"
        else:
            topo = f"{len(ids)} 舵机/线（非标准划分，记录待定）"
        print(f"拓扑判定（本线）：{topo}")

        print("\n位置/负载/温度回读（3 次）：")
        for _ in range(3):
            row = []
            for sid in ids:
                fb = read_feedback(t, sid)
                if fb and len(fb) >= 8:
                    pos = fb[0] | (fb[1] << 8)
                    load = fb[4] | (fb[5] << 8)
                    row.append(f"#{sid}:pos={pos:4d} load={load:3d} T={fb[7]}C")
            print("  " + " | ".join(row))
            time.sleep(0.3)
    finally:
        t.close()

    print("\n下一步：tools/blink_servo.py --via mcu 单舵机摆动；再逐舵机标定。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
