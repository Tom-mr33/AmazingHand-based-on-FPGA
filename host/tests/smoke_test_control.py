"""控制链路冒烟测试 —— 协议层 golden model + 虚拟舵机总线。

运行方式（无任何第三方依赖）::

    python host/tests/smoke_test_control.py

也可用 pytest 收集::

    pytest host/tests/smoke_test_control.py

用例分四组：
1. 协议组包/解包 —— 字节级锚点（帧内含手工算好的校验和，防"错得一致"）
2. 协议解析鲁棒性 —— 前导垃圾 / 坏校验和 / 截断帧
3. 虚拟总线闭环 —— PING / 写目标/回读位置 / 限速 / SYNC_WRITE 广播
4. 故障注入 —— 掉线 / 堵转 / 过温切断（grasp_fsm 保护逻辑的行为预期）

这些用例同时是 SCS 协议与虚拟总线的回归基线。
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # host/

from app.control import scs_protocol as sp  # noqa: E402
from app.control import scs_sim  # noqa: E402

H = bytes([0xFF, 0xFF])

# ---- 便捷组包（Reg 元组为 (addr, size)，显式取 addr 防解包歧义） ----


def _w1(sid: int, reg: tuple[int, int], val: int) -> bytes:
    """写 1 字节寄存器。"""
    return sp.build_write(sid, reg[0], bytes([val]))


def _w2(sid: int, reg: tuple[int, int], val: int) -> bytes:
    """写 16 位小端寄存器。"""
    return sp.build_write_u16(sid, reg[0], val)


def _torque_on(sid: int, on: bool = True) -> bytes:
    return _w1(sid, sp.Reg.TORQUE_SWITCH, 1 if on else 0)


# ==========================================================================
# 1. 协议组包：字节级锚点
# ==========================================================================


def test_checksum_known_vector() -> None:
    """手工算例：01+05+03+2A+00+08 = 0x3B -> ~0x3B = 0xC4。"""
    assert sp.checksum(0x01, 0x05, 0x03, 0x2A, 0x00, 0x08) == 0xC4


def test_build_write_bytes() -> None:
    """WRITE goal_position=2048 (0x0800 小端) 的完整帧。"""
    expect = H + bytes([0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4])
    assert sp.build_write(1, 0x2A, bytes([0x00, 0x08])) == expect
    assert sp.build_write_u16(1, 0x2A, 2048) == expect


def test_build_read_bytes() -> None:
    """READ 0x3A 起 6 字节的完整帧（01+04+02+3A+06 = 0x47 -> 0xB8）。"""
    expect = H + bytes([0x01, 0x04, 0x02, 0x3A, 0x06, 0xB8])
    assert sp.build_read(1, 0x3A, 6) == expect


def test_build_sync_write_bytes() -> None:
    """SYNC_WRITE 两舵机 goal=2048 的完整帧（广播 ID、14 字节、CS=0x35）。"""
    frame = sp.build_sync_write(
        0x2A, 2, {1: bytes([0x00, 0x08]), 2: bytes([0x00, 0x08])}
    )
    expect = H + bytes(
        [0xFE, 0x0A, 0x83, 0x2A, 0x02, 0x01, 0x00, 0x08, 0x02, 0x00, 0x08, 0x35]
    )
    assert frame == expect
    assert sp.is_broadcast(frame)


def test_units_roundtrip() -> None:
    """rad <-> raw 往返误差不超过半步。"""
    half_step = math.pi / sp.POSITION_STEPS
    for rad in (-math.pi, -1.0, 0.0, 0.5, math.pi):
        assert abs(sp.raw_to_rad(sp.rad_to_raw(rad)) - rad) <= half_step + 1e-9


# ==========================================================================
# 2. 协议解析鲁棒性
# ==========================================================================


def test_parse_response_ok() -> None:
    """PING 响应：FF FF 01 02 00 CS，CS = ~(01+02+00) = ~03 = 0xFC。"""
    resp = sp.parse_response(H + bytes([0x01, 0x02, 0x00, 0xFC]))
    assert resp.servo_id == 1
    assert resp.error == 0
    assert resp.params == b""


def test_parse_response_with_params() -> None:
    """READ 响应 2 字节小端参数 0x0800 = 2048。"""
    frame = H + bytes([0x01, 0x04, 0x00, 0x00, 0x08, sp.checksum(1, 4, 0, 0, 8)])
    resp = sp.parse_response(frame)
    assert resp.param_u16(0) == 2048


def test_parse_response_garbage_prefix() -> None:
    """UART 缓冲残渣：前导垃圾字节应被跳过。"""
    noise = bytes([0x00, 0xAB, 0xFF, 0x00, 0xDE])
    good = H + bytes([0x02, 0x02, 0x00, sp.checksum(2, 2, 0)])
    resp = sp.parse_response(noise + good)
    assert resp.servo_id == 2


def test_parse_response_bad_checksum() -> None:
    frame = bytearray(H + bytes([0x01, 0x02, 0x00, 0xFC]))
    frame[-1] ^= 0xFF
    try:
        sp.parse_response(bytes(frame))
        raise AssertionError("expected ProtocolError")
    except sp.ProtocolError:
        pass


def test_parse_response_truncated() -> None:
    for bad in (b"", b"\xff\xff", H + bytes([0x01, 0x10, 0x00])):  # 长度字段超帧
        try:
            sp.parse_response(bad)
            raise AssertionError("expected ProtocolError")
        except sp.ProtocolError:
            pass


def test_parse_response_no_header() -> None:
    try:
        sp.parse_response(bytes([1, 2, 3, 4, 5, 6]))
        raise AssertionError("expected ProtocolError")
    except sp.ProtocolError:
        pass


# ==========================================================================
# 3. 虚拟总线闭环
# ==========================================================================


def _fresh_bus() -> scs_sim.VirtualServoBus:
    return scs_sim.VirtualServoBus()


def test_ping_roundtrip() -> None:
    bus = _fresh_bus()
    resp = sp.parse_response(bus.transact(sp.build_ping(3)))
    assert resp.servo_id == 3
    assert resp.error == 0


def test_ping_nonexistent_servo() -> None:
    bus = _fresh_bus()
    assert bus.transact(sp.build_ping(99)) is None


def test_torque_then_move_then_read_position() -> None:
    """写 goal=3000 -> 推进时间 -> 回读位置逼近目标。"""
    bus = _fresh_bus()
    ack = sp.parse_response(bus.transact(_torque_on(1)))
    assert ack.error == 0
    bus.transact(_w2(1, sp.Reg.GOAL_POSITION, 3000))
    for _ in range(1000):
        bus.step(0.01)  # 共 10s 虚拟时间
    resp = sp.parse_response(
        bus.transact(sp.build_read(1, *sp.Reg.PRESENT_POSITION))
    )
    assert resp.error == 0
    pos = resp.param_u16(0)
    assert abs(pos - 3000) <= 2, f"position not converged: {pos}"


def test_speed_limit_enforced() -> None:
    """goal_speed_raw=10 时，1s 内移动不得超过 ~10 步。"""
    bus = _fresh_bus()
    bus.transact(_torque_on(1))
    bus.transact(_w2(1, sp.Reg.RUNNING_SPEED, 10))
    bus.transact(_w2(1, sp.Reg.GOAL_POSITION, 3048))  # delta=1000
    bus.step(1.0)
    assert bus.state(1).position_raw <= 2048 + 12


def test_feedback_block_read() -> None:
    """力控时隙连读 0x3A 起 8 字节：pos/speed/load/volt/temp 齐全。"""
    bus = _fresh_bus()
    bus.state(1).torque_on = True
    bus.state(1).goal_raw = 2500
    bus.step(0.1)
    resp = sp.parse_response(
        bus.transact(sp.build_read(1, sp.Reg.FEEDBACK_ADDR, sp.Reg.FEEDBACK_LEN))
    )
    assert len(resp.params) == sp.Reg.FEEDBACK_LEN
    pos = resp.param_u16(0)
    load = resp.param_u16(4)
    temp = resp.params[7]
    assert 2048 < pos <= 2500
    assert 0 <= load <= scs_sim.LOAD_MAX_RAW
    assert scs_sim.AMBIENT_TEMP_C - 1 <= temp <= 100


def test_sync_write_updates_all_goals() -> None:
    """1kHz 时隙核心指令：一条帧更新 8 舵机 goal，且无响应。"""
    bus = _fresh_bus()
    goals = {i: 500 * i for i in range(1, 9)}
    frame = sp.build_sync_write(
        sp.Reg.GOAL_POSITION[0], 2, {i: bytes([g & 0xFF, g >> 8]) for i, g in goals.items()}
    )
    assert len(frame) == 32, f"sync_write 8-servo frame should be 32B, got {len(frame)}"
    assert bus.transact(frame) is None
    for sid, g in goals.items():
        assert bus.state(sid).goal_raw == g, f"servo {sid} goal not applied"


def test_broadcast_write_no_response() -> None:
    bus = _fresh_bus()
    frame = sp.build_write(sp.BROADCAST_ID, sp.Reg.TORQUE_SWITCH[0], bytes([1]))
    assert bus.transact(frame) is None
    assert all(bus.state(i).torque_on for i in bus.servo_ids)


def test_reg_write_action_pair() -> None:
    """REG_WRITE 暂存不生效、ACTION 广播统一触发（用非默认值严格验证）。"""
    bus = _fresh_bus()
    bus.transact(_torque_on(1))
    bus.transact(
        sp.build_reg_write(2, sp.Reg.GOAL_POSITION[0], bytes([0xB8, 0x0B]))  # 3000
    )
    assert bus.state(2).registered
    assert bus.state(2).goal_raw == 2048  # ACTION 前不生效（默认值未被改写）
    bus.transact(sp.build_action())
    assert not bus.state(2).registered
    assert bus.state(2).goal_raw == 3000  # ACTION 后生效


def test_truncated_header_frame_no_crash() -> None:
    """半帧到达（UART 常见）：任何长度不足的帧头帧都安全丢弃，不抛异常。"""
    bus = _fresh_bus()
    for bad in (b"\xff\xff", b"\xff\xff\x01", b"\xff\xff\x01\x05", b"\xff\xff\x01\x05\x03"):
        assert bus.transact(bad) is None, f"should drop {bad!r}"


def test_broadcast_reset_all() -> None:
    """广播 RESET：全部舵机恢复出厂态（力矩断开、目标回中位），无响应。"""
    bus = _fresh_bus()
    bus.transact(_torque_on(1))
    bus.transact(_w2(2, sp.Reg.GOAL_POSITION, 3000))
    frame = sp._frame(sp.BROADCAST_ID, sp.INS_RESET, b"")
    assert bus.transact(frame) is None
    for sid in bus.servo_ids:
        s = bus.state(sid)
        assert s.goal_raw == scs_sim.DEFAULT_POSITION_RAW, f"servo {sid} goal not reset"
        assert not s.torque_on, f"servo {sid} torque not released"


def test_bad_checksum_frame_dropped() -> None:
    bus = _fresh_bus()
    frame = bytearray(sp.build_ping(1))
    frame[-1] ^= 0x55
    assert bus.transact(bytes(frame)) is None


# ==========================================================================
# 4. 故障注入（grasp_fsm 保护逻辑的行为预期）
# ==========================================================================


def test_drop_fault_no_response() -> None:
    """掉线注入：READ/PING 一律无响应（主机侧超时语义）。"""
    bus = _fresh_bus()
    bus.inject_drop([1])
    assert bus.transact(sp.build_ping(1)) is None
    assert bus.transact(sp.build_read(1, sp.Reg.PRESENT_POSITION[0], 2)) is None
    assert bus.transact(sp.build_ping(2)) is not None  # 其他舵机不受影响


def test_overload_stall_sets_error_bit() -> None:
    """堵转：外力顶满力矩极限 -> 位置推不动 + ERR_OVERLOAD 置位。"""
    bus = _fresh_bus()
    bus.transact(_torque_on(1))
    bus.inject_external_load({1: scs_sim.DEFAULT_TORQUE_LIMIT_RAW})
    bus.transact(_w2(1, sp.Reg.GOAL_POSITION, 3000))
    bus.step(0.1)
    assert bus.state(1).stalled
    resp = sp.parse_response(bus.transact(sp.build_read(1, sp.Reg.PRESENT_POSITION[0], 2)))
    assert resp.error & sp.ERR_OVERLOAD
    assert resp.param_u16(0) == 2048  # 位置纹丝未动


def test_overheat_shutdown() -> None:
    """过温：温度超限 -> 自动切断力矩 + ERR_OVERHEAT，且拒绝重新使能。"""
    bus = _fresh_bus()
    bus.transact(_torque_on(1))
    bus.inject_external_load({1: 900})
    bus.transact(_w1(1, sp.Reg.MAX_TEMPERATURE_LIMIT, 26))
    for _ in range(20):
        bus.step(0.5)  # 10s，足够越过 26C 阈值
    s = bus.state(1)
    assert s.temperature_c >= 26.0
    assert s.overheat_shutdown and not s.torque_on
    resp = sp.parse_response(bus.transact(sp.build_read(1, sp.Reg.PRESENT_TEMPERATURE[0], 1)))
    assert resp.error & sp.ERR_OVERHEAT
    # 过温后重新使能必须被拒绝（安全）
    bus.transact(_torque_on(1))
    assert not bus.state(1).torque_on


def test_torque_off_no_motion() -> None:
    """力矩关闭：goal 再远也不动。"""
    bus = _fresh_bus()
    bus.transact(_w2(1, sp.Reg.GOAL_POSITION, 3000))
    bus.step(1.0)
    assert bus.state(1).position_raw == 2048


def test_reset_restores_defaults() -> None:
    bus = _fresh_bus()
    bus.state(1).goal_raw = 100
    bus.transact(_torque_on(1))
    bus.transact(sp._frame(1, sp.INS_RESET, b""))
    assert bus.state(1).goal_raw == scs_sim.DEFAULT_POSITION_RAW


# ==========================================================================
# 入口
# ==========================================================================


def main() -> int:
    tests = [
        (name, fn)
        for name, fn in sorted(globals().items())
        if name.startswith("test_") and callable(fn)
    ]
    failures = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"  FAIL  {name}: {type(exc).__name__}: {exc}")
    total = len(tests)
    print(f"\n{total - failures}/{total} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
