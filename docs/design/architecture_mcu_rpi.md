# STM32F103 + 树莓派 架构（2026-10-20 起的主链路）

> 决策：项目只保留 **树莓派（感知/策略/UI） + STM32F103（实时舵机总线）**；
> 原 FPGA/Z19 代码、文档、相机驱动与赛事材料已删除。本文是唯一架构基线。

## 1. 目标与分工

| 层 | 硬件 | 职责 |
|---|---|---|
| 高层 | Raspberry Pi（Linux） | 语音命令词、YOLO 检测 + 单目尺寸、抓取策略、主状态机、引导/UI、日志 |
| 实时层 | STM32F103C8T6 | 1 Mbps SCS 单总线、8 × SCS0009 收发、反馈轮询；Phase 2 起接管抓取 FSM/保护 |
| 执行器 | 8 × Feetech SCS0009 | AmazingHand 四指两自由度 |

设计原则：**模糊性交给 Pi（AI/策略），确定性交给 STM32（固定时序/保护）**。
不再需要 Vivado、FMC、Z19 相机驱动或 FPGA 工具链。

## 2. 数据流

~~~
麦克风/USB 相机（Pi）
   -> 语音命令词 / YOLO 检测 / 尺寸估计
   -> grasp_selector 查表得到 GraspPlan
   -> Pi 主状态机（IDLE/RECOGNIZING/GUIDING/GRASPING/HOLDING/RELEASING/FAULT）
   -> UART 115200：帧长前缀 + 原始 SCS 帧
   -> STM32F103 字节泵（Phase 1） / 抓取 FSM + 保护（Phase 2）
   -> 1 Mbps 半双工单总线
   -> 8 × SCS0009
~~~

## 3. 硬件连接

### 3.1 Pi ↔ STM32

| Pi 侧 | STM32 侧 | 说明 |
|---|---|---|
| GPIO14 / UART TX（`/dev/serial0`） | PA3 / USART2_RX | 115200 8N1 |
| GPIO15 / UART RX | PA2 / USART2_TX | 115200 8N1 |
| GND | GND | 必须共地 |
| 5V（可选） | 5V | STM32 可独立供电；不建议从 Pi 取舵机电流 |

如果不用 Pi UART，可用 USB-TTL（CH340/CP2102）接 PA2/PA3，设备名变成 `/dev/ttyUSB0`。
树莓派需在 `raspi-config` 打开串口硬件、关闭串口登录，再确认 `/dev/serial0` 可用。

### 3.2 STM32 ↔ 舵机总线

| 信号 | STM32 | 舵机 3-pin 口 | 说明 |
|---|---|---|---|
| DATA | PA9（USART1，HDSEL 单线半双工，1 Mbps） | 白线 | 可串 100Ω 保护；不需要 1kΩ 合并 |
| GND | GND | 黑线 | **必须与独立 5V 电源共地** |
| V+ | 不接 | 红线 | **独立 5V 电源**，禁止从 Pi/STM32/USB 取电 |

PA9 是 5V 容忍（FT）引脚；若 3.3V 发送电平导致舵机响应不稳，在 PA9 与总线间加
3.3V↔5V 电平转换（DFRobot DFR0844 / TXS0108E，VCCA=3.3V、VCCB=5V）。

### 3.3 感知外设

| 外设 | 接口 | 说明 |
|---|---|---|
| USB 摄像头 | UVC `/dev/video0` | 用 OpenCV 直接读 UVC 相机 |
| USB 麦克风 | ALSA | Vosk 命令词识别 |
| 显示 | HDMI / 浏览器 | 演示界面可跑在 Pi 本机或 PC 浏览器 |

## 4. 通信协议（Pi ↔ STM32，Phase 1）

当前固件是**字节泵**，不解析 SCS：

~~~
下行： [len u8][len 字节原始 SCS 请求帧]    len = 1..64
上行： [len u8][len 字节 SCS 响应帧]        len = 0 = 总线静默（无响应/离线）
~~~

- SCS 帧由 `host/app/control/scs_protocol.py` 生成与解析；
- 串口参数 115200 8N1；
- 每个事务一次请求/一次响应；超时 50 ms；
- 115200 是瓶颈：控制更新 ≤100 Hz，只适合探测/标定/低速动作；1 kHz 环留给 Phase 2。

Phase 2 计划（尚未实现）把 FSM/保护下沉到 STM32，定义独立的二进制控制协议：
命令（open/grasp/hold/release/stop/reset）、8 路目标 raw、反馈/状态/故障/计数器；
Pi 只发高层计划并显示遥测。协议冻结前不实现。

## 5. 代码映射

| 功能 | 文件 |
|---|---|
| 传输层（帧长前缀，bring-up） | `host/app/control/mcu_link.py` |
| Pi 侧后端（软件 FSM，bring-up） | `host/app/control/backend_mcu.py` |
| STM32 控制协议（Pi 侧） | `host/app/control/stm32_protocol.py` |
| STM32 控制后端（正式） | `host/app/control/backend_stm32.py` |
| STM32 固件协议模块 | `firmware/stm32/Core/{Inc,Src}/ah_protocol.*` |
| 控制协议设计 | `docs/design/stm32_control_protocol.md` |
| 主状态机/后端工厂 | `host/app/main.py` |
| 配置 | `host/config/settings.yaml`（`mcu:` 段） |
| STM32 固件 | `firmware/stm32/`（HAL 工程，main.c 为字节泵） |
| 单舵机测试 | `tools/blink_servo.py --via mcu` |
| 总线探测 | `tools/probe_servos.py --via mcu` |
| 协议 golden model | `host/app/control/scs_protocol.py` |
| 抓取策略 | `host/app/strategy/` |
| 视觉/语音 | `host/app/vision/`、`host/app/voice/` |
| PC 数字孪生（可选） | `sim/`（MuJoCo，只在 PC 上跑） |

## 6. 分阶段实施

### Phase 1（当前代码，目标：单舵机→整手标定）
- STM32 字节泵 + 树莓派 UART；
- Pi 上 `backend_mcu.py` 跑软件 FSM；
- 工具：blink_servo、probe_servos；
- 验收：8 个 ID 全通；单舵机方向/中位标定；整手张开/闭合/停止。

### Phase 2（进行中，目标：实时性与安全）
- [x] 协议/主机后端/固件 C 编解码（`stm32_protocol.py`、`backend_stm32.py`、`ah_protocol.*`）
- [ ] `main.c` 接入 1 ms 调度 + SCS 总线 + FSM（下一步）
- 把 `grasp_fsm` 逻辑从 Python 下沉到 STM32 C；
- STM32 内部固定周期轮询 + 负载接触判定 + 超时/掉线/过温/过载保护；
- STM32 独立看门狗：Pi 断连 500 ms 内卸力/保护；
- Pi↔STM32 改成状态/遥测协议，Pi 只发高层命令；
- 验收：停止/断连/超时路径、1 kHz 级调度与抖动数据。

### Phase 3（目标：完整演示）
- Pi 接入真实语音、YOLO、测尺寸、ArUco；
- 演示：语音指定物体→检测→引导→抓取→保持→释放；
- 指标：端到端延迟、每类成功率、软件/STM32 环对比。

## 7. 安全边界（必须遵守）

1. **Phase 1 没有独立安全兜底**：Pi 崩溃/断线时，舵机保持最后目标；必须有人看护，
   并准备硬件急停（直接断舵机 5V）。
2. 舵机 5V 独立供电、共地、串保险丝；禁止从 Pi/STM32/USB 取电。
3. 先单舵机、后整手；先小角度、后全行程。
4. 上电前用万用表确认 5V 极性与共地；不要热插拔总线。
5. Phase 2 的 STM32 看门狗/FSM 未完成前，不进行无人值守演示。

## 8. 赛事与交付

原比赛已取消；FPGA/Z19 代码、文档、相机驱动与赛事材料已删除。本仓库只保留
树莓派 + STM32 主链路。

## 9. 树莓派部署步骤（Phase 1）

~~~sh
# 1) 系统：Raspberry Pi OS 64-bit（Bookworm，Python 3.11）
sudo raspi-config   # Interface -> Serial -> 启用硬件串口，关闭串口登录
ls -l /dev/serial0

# 2) 代码与依赖
git clone https://github.com/Tom-mr33/AmazingHand-based-on-stm32.git
cd AmazingHand-based-on-stm32
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements-pi.txt

# 3) 先验证串口/舵机（单舵机 + 独立 5V + 共地）
python tools/blink_servo.py --port /dev/serial0 --via mcu --id 1
python tools/probe_servos.py --port /dev/serial0 --via mcu

# 4) 跑主流程（Phase 1 还没有把真实语音/视觉接进 main.py，先用 mock 命令验证状态机）
python host/app/main.py --backend mcu --mock-voice "抓起香蕉,放下"
~~~

配置：`host/config/settings.yaml` 改 `backend: mcu`，按实际串口改 `mcu.port`。
PC 上仍可用 `--backend sim` 或 `--mujoco` 做离线验证。

## 10. 已知限制

- STM32 固件来自旧分支原型，**尚未在实机编译/烧录**；首次构建可能需要修 HAL/时钟细节。
- Pi 上的真实语音/视觉尚未接入 `main.py`（当前只有 mock 输入）。
- 115200 链路吞吐低；整手 8 路反馈轮询较慢。
- SCS0009 端序/寄存器/负载解码仍未实机核验（待定项 1/2）。
- 无独立安全兜底（见第 7 节）。
