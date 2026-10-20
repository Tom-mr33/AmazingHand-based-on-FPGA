# Amazing Hand AI — 树莓派 + STM32 人机协作智能辅助手

> **2026-10-20**：项目只保留 **树莓派（感知/策略/UI） + STM32F103C8T6
> （1 Mbps SCS 舵机总线）** 主链路；原 FPGA/Z19 代码、文档、相机驱动与赛事材料已删除。
> 架构基线见 [docs/design/architecture_mcu_rpi.md](docs/design/architecture_mcu_rpi.md)。

目标流程：语音指定目标 → 树莓派端侧视觉识别/估计尺寸 → 引导人手就位 →
STM32 实时总线驱动 8 × SCS0009 完成抓取/保持/释放。

## 系统架构

~~~
树莓派（Linux / Python）
  语音命令词 -> YOLO 检测 + 单目尺寸 -> 抓取策略 -> 主状态机/UI
        │  UART 115200（帧长前缀 + 原始 SCS 帧）
        ▼
STM32F103C8T6（字节泵；Phase 2 接管抓取 FSM/保护/看门狗）
        │  1 Mbps 单线半双工
        ▼
8 × Feetech SCS0009 -> AmazingHand
~~~

设计原则：**模糊性交给 Pi（AI/策略），确定性交给 STM32（固定时序/保护）**。
不再需要 Vivado、FMC、Z19 相机驱动或 FPGA 工具链。

## 当前状态（2026-10-20）

| 模块 | 状态 | 证据/边界 |
|---|---|---|
| 树莓派 + STM32 传输层 | 代码就绪 | `mcu_link.py` / `backend_mcu.py`；7 项无硬件单测通过 |
| STM32 透传固件 | 代码就绪，未实机 | `firmware/stm32/`；**尚未编译/烧录**，首次上电需自检 |
| Pi 侧软件 FSM | 代码就绪 | `backend_mcu` 跑 `grasp_fsm` 等价逻辑；115200 限制 ≤100 Hz |
| SCS 协议/单舵机工具 | 代码就绪，未实机 | `blink_servo.py --via mcu`、`probe_servos.py --via mcu` |
| 真实语音/视觉接入 | 未完成 | 现有 `kws.py`/`vision` 仍为骨架/Mock 入口 |
| 实物标定/抓取 | 未开始 | 待单舵机 ID/方向/中位标定后再整手 |

> 诚实边界：当前没有真实舵机、真实相机或真实抓取验收；不要用 PC 模拟结果替代。

## 目录结构

~~~
firmware/stm32/   STM32F103C8T6 SCS 总线透传固件（HAL/CMake）
host/             树莓派/Python：控制后端、策略、视觉、语音、配置、测试
tools/            PC/Pi 工具：单舵机摆动、总线探测、模型导出、标定
sim/              PC 端 MuJoCo 数字孪生（不装到树莓派）
data/             标定结果与实测指标
docs/             架构、接口、开发日志
~~~

## 快速开始

### PC 模拟链路（不接硬件）

~~~sh
python host/tests/smoke_test_control.py
python host/tests/smoke_test_backend.py
python host/app/main.py --mock-voice "抓起香蕉,放下"
~~~

MuJoCo 实时演示见 [sim/LIVE_CONTROL.md](sim/LIVE_CONTROL.md)。

### 树莓派 + STM32（Phase 1）

1. 硬件：Pi、STM32F103C8T6、8 × SCS0009、**独立 5V 电源**、USB 相机、USB 麦克风。
2. 接线：Pi UART TX/RX ↔ STM32 PA3/PA2；STM32 PA9 ↔ 舵机 DATA；共地；
   舵机 5V 独立供电。完整表见
   [firmware/stm32/README.md](firmware/stm32/README.md)。
3. 刷固件：ARM 工具链编译后 ST-Link 烧录（见固件 README）。
4. 单舵机验证：

   ~~~sh
   python tools/blink_servo.py --port /dev/serial0 --via mcu --id 1
   python tools/probe_servos.py --port /dev/serial0 --via mcu
   ~~~

5. 配置 `host/config/settings.yaml`：`backend: mcu`、`mcu.port: /dev/serial0`。
6. 跑主流程（当前仅 mock 语音入口）：

   ~~~sh
   python host/app/main.py --backend mcu --mock-voice "抓起香蕉,放下"
   ~~~

Pi 依赖：`pip install -r requirements-pi.txt`。系统建议 Raspberry Pi OS 64-bit
Bookworm（Python 3.11）；旧版 Python 3.9 不支持部分类型语法。

## 开发状态

- [x] 树莓派 + STM32 传输层/后端/单测
- [x] STM32 透传固件工程（待实机编译/烧录）
- [ ] 8 舵机 ID/方向/中位标定（下一优先级）
- [ ] STM32 内部 FSM/保护/看门狗（Phase 2）
- [ ] Pi 真实语音/视觉接入
- [ ] 整手抓取/保持/释放与指标实测
- [x] 删除 FPGA/Z19 代码、文档、相机驱动与赛事材料（2026-10-20）

进度详见 [docs/log/](docs/log/)。采购清单见 [docs/procurement.md](docs/procurement.md)
（已按 Pi+STM32 路线更新）。

## 许可证

项目代码默认 Apache-2.0，以各文件声明为准。STM32 HAL/CMSIS 保留 ST 原始许可；
第三方依赖保留原有版权/许可，不统一重授权为 Apache。
机械结构继承 [AmazingHand](https://github.com/pollen-robotics/AmazingHand) 的 CC-BY 4.0。

## 致谢

- [Pollen Robotics — AmazingHand](https://github.com/pollen-robotics/AmazingHand)
- [Feetech SCS 系列](https://www.feetechrc.com/)
