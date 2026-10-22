"""舵机后端抽象基类 —— 上层只认接口，换硬件只改 settings.yaml 一行。

后端（2026-10-20 起主链路为 mcu）::

    backend_sim.py     虚拟舵机（PC 冒烟/全链路空转，确定性 tick）
    backend_serial.py  USB-TTL 软件环（真实舵机，调试/对比基准）
    backend_mcu.py     树莓派/PC → STM32F103 原始 SCS 透传（bring-up）
    backend_stm32.py   STM32 内部 FSM/保护，Pi 只发高层命令（正式控制）

各后端接受**同一份 :class:`GraspPlan` 输入**，行为差异只来自控制环本身；
``read_loop_timestamp()`` 提供各环的本地时间基准用于抖动统计。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum

#: 关节数：4 指 × (屈曲, 外展)，舵机 ID 1..8 官方映射
N_JOINTS = 8


class FsmState(IntEnum):
    """抓取状态编码（与 backend_sim / STM32 FSM 一致）。"""

    IDLE = 0
    PREGRASP = 1
    CLOSING = 2
    HOLD = 3
    RELEASING = 4
    PROTECT = 5


@dataclass(frozen=True)
class GraspPlan:
    """策略层 → 控制层的抓取计划（三后端共同的输入契约）。

    angles_deg / pregrasp_deg 均为 8 关节目标角（度，**相对各舵机机械中位**
    的偏移，与官方 Demo 的 MiddlePos+Angle 约定一致；中位值由标定产出）。
    """

    angles_deg: tuple[float, ...]  # 长度 N_JOINTS：闭合抓取目标
    pregrasp_deg: tuple[float, ...] = (-35.0, 35.0, -35.0, 35.0, -35.0, 35.0, -35.0, 35.0)
    speed_raw: int = 0  # 0 = 全速（真实舵机语义）
    torque_levels: tuple[int, ...] = (7,) * N_JOINTS  # 0..15 级/通道
    mode: str = "power"  # power / pinch / lateral
    label: str = ""  # 来源标注（类别名），仅日志用

    def __post_init__(self) -> None:
        if len(self.angles_deg) != N_JOINTS:
            raise ValueError(f"angles_deg must have {N_JOINTS} entries")
        if len(self.pregrasp_deg) != N_JOINTS:
            raise ValueError(f"pregrasp_deg must have {N_JOINTS} entries")
        if len(self.torque_levels) != N_JOINTS:
            raise ValueError(f"torque_levels must have {N_JOINTS} entries")
        for t in self.torque_levels:
            if not 0 <= t <= 15:
                raise ValueError(f"torque level out of range 0..15: {t}")


@dataclass(frozen=True)
class ServoFeedback:
    """单舵机回读快照（对应 MEAS_i / LOAD_i / TEMP 寄存器）。"""

    servo_id: int
    online: bool
    position_deg: float = 0.0
    position_raw: int = 0
    load_raw: int = 0
    temperature_c: float = 0.0
    voltage_x10: int = 0
    error_bits: int = 0


@dataclass
class FsmSnapshot:
    """poll_fsm() 返回的控制器全景。"""

    state: FsmState = FsmState.IDLE
    busy: bool = False
    grasped: bool = False
    timeout: bool = False
    fault: bool = False
    err_bitmap: int = 0  # 每位一通道：bit0 堵转 / bit1 掉线 / bit2 超温（细则待定项定稿）
    loop_timestamp: int = 0  # 环时间戳（F5 抖动统计用）
    feedback: list[ServoFeedback] = field(default_factory=list)


class ServoBackend(ABC):
    """三后端统一接口。构造不打开硬件，open() 才做资源获取。"""

    name: str = "base"

    @abstractmethod
    def open(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

    @abstractmethod
    def set_torque(self, on: bool) -> None:
        """全体舵机力矩使能/断开。"""

    @abstractmethod
    def set_targets(self, angles_deg: tuple[float, ...], speed_raw: int = 0) -> None:
        """直接角度模式：逐关节下发目标。"""

    @abstractmethod
    def start_grasp_fsm(self, plan: GraspPlan) -> None:
        """状态机模式：下发目标/力矩/时序并启动抓取 FSM。

        mcu/serial/sim 后端：由 Pi/PC 侧等价软件状态机执行；
        Phase 2 计划把该 FSM 下沉到 STM32。
        """

    @abstractmethod
    def release(self) -> None:
        """CMD=0x02：张开放置。"""

    @abstractmethod
    def poll_fsm(self) -> FsmSnapshot:
        """读 STATUS/FSM_STATE/ERR_BITMAP/MEAS/LOAD/TEMP。"""

    @abstractmethod
    def emergency_stop(self) -> None:
        """急停（CTRL.estop）：立即断力矩进入保护。"""

    @abstractmethod
    def read_loop_timestamp(self) -> int:
        """环时间戳：sim=虚拟时钟 µs，serial/mcu=perf_counter ns。"""

    def tick(self, dt_s: float) -> None:
        """时间推进钩子：仅 sim 后端有实际行为（确定性动力学），其余为空。"""

    def __enter__(self) -> "ServoBackend":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()
