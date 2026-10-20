# firmware/stm32 — STM32F103C8T6 SCS 总线透传固件

树莓派（或 PC）作为上层控制器，STM32F103C8T6 作为 **1 Mbps SCS 总线网关**：

```
Raspberry Pi / PC
  └─ UART 115200（Pi /dev/serial0 或 USB-TTL）──> USART2 PA2/PA3
       └─ STM32F103C8T6
            └─ USART1 PA9 单线半双工 1 Mbps ──> 8 × SCS0009（树状并联单总线）
```

本固件是**字节泵**：SCS 帧格式、校验和、寄存器语义全部在
`host/app/control/scs_protocol.py`；固件只负责把 PC 下发的原始 SCS 帧
转发到总线，并把舵机响应原样回传。当前**不包含**抓取 FSM、力控保护或
独立看门狗，这些在 Phase 1 仍由树莓派上的 Python 后端执行。

> 本目录取回自分支 `feature/sim-model-rtl@903b462`（2026-10-06 原型），
> 尚未在实机上编译/烧录/验证。首次使用请按下面步骤构建并自检。

## 接线（必须遵守）

| 信号 | 从 | 到 |
|---|---|---|
| 总线 DATA（白） | 手部最终 3-pin 口 | STM32 **PA9**（HDSEL 单线；可串 100Ω 保护，不需要 1kΩ 合并） |
| 总线 GND（黑） | 手部最终 3-pin 口 | 独立 5V 电源负极 **且** STM32 GND（共地，必须） |
| 总线 V+（红） | 手部最终 3-pin 口 | 独立 5V 电源正极（**禁止**从树莓派/STM32/USB 取电） |
| 上行 TX | Pi GPIO14 (TXD) 或 USB-TTL TX | STM32 **PA3**（USART2_RX） |
| 上行 RX | Pi GPIO15 (RXD) 或 USB-TTL RX | STM32 **PA2**（USART2_TX） |
| 电源地 | Pi/PC GND | STM32 GND（与舵机电源共地） |

- 树莓派用 `/dev/serial0` 时需在 `raspi-config` 打开串口、关闭串口登录；
  若用 USB-TTL，则设备名是 `/dev/ttyUSB0`（Linux）或 `COMx`（Windows）。
- PA9 在 STM32F103 上是 5V 容忍（FT）引脚，可直接接收 5V 总线；STM32 发送
  高电平为 3.3V，若舵机响应不稳定，可在 PA9 与总线之间加 5V 电平转换
  （DFRobot DFR0844 / TXS0108E，VCCA=3.3V、VCCB=5V）。
- 共地、独立 5V 供电、串保险丝是硬要求，详见 `docs/design/architecture_mcu_rpi.md`。

## 构建（Ubuntu / 树莓派，arm-none-eabi-gcc）

```sh
sudo apt install -y cmake gcc-arm-none-eabi
cd firmware/stm32
cmake --preset Debug
cmake --build build/Debug
# 产物：build/Debug/ah_passthrough.elf
```

## 烧录（ST-Link V2 SWD）

```sh
STM32_Programmer_CLI -c port=SWD freq=4000 \
  -w build/Debug/ah_passthrough.elf -v -rst
```

SWD 四线：SWDIO / SWCLK / GND / 3V3。板载 USB-C 仅供电，不能当 USB CDC 用。

## 帧协议（PC ↔ STM32，115200 8N1）

```
PC -> STM32:  [len u8][len 字节原始 SCS 请求帧]      len = 1..64
STM32 -> PC:  [len u8][len 字节 SCS 响应]            len = 0 表示总线静默（无响应/离线）
```

- 下行帧由 PC 的 `scs_protocol.build_*` 生成；
- 固件在 USART1 上以 HDSEL 半双工发送，丢弃回听 echo，再以 3 ms 静默窗口
  收集响应；
- 115200 的 PC 链路是瓶颈：**控制更新率 ≤ 100 Hz**，不适合 1 kHz 实时环或
  抖动对比；1 kHz 环后续由 STM32 内部 FSM/调度实现，或改用更高带宽的 USB 总线适配器。

## 上电自检（先单舵机，后整手）

```sh
# 单舵机摆动（PC 或树莓派）
python tools/blink_servo.py --port /dev/serial0 --via mcu --id 1

# 扫 ID / 拓扑 / 回读
python tools/probe_servos.py --port /dev/serial0 --via mcu
```

先只接 1 个舵机、独立 5V、共地；确认 ID/方向/中位后再接整手。

## 当前限制（Phase 1）

1. **无独立安全兜底**：上位机崩溃/断线时，舵机保持最后目标；必须先做
   硬件急停（断舵机电源）并在有人看护下测试。
2. 无 STM32 内部抓取 FSM/力控保护；`backend_mcu.py` 在 Pi 上跑软件 FSM。
3. 115200 链路吞吐有限；整手 8 路反馈轮询较慢。
4. 未做 SCS0009 实机协议核验（端序/寄存器/负载解码仍待标定）。
5. 本原型尚未在实机编译/烧录；P0 优先级是先编译 + 单舵机 PING。

## 故障排查

| 现象 | 排查顺序 |
|---|---|
| probe 全部无响应 | 共地 → 5V 供电 → PA9/DATA 接线 → 舵机 ID 1-8 → 波特率 |
| 响应偶发校验错 | GND 走线、杜邦线过长、1 Mbps 信号完整性；可先降速/缩短线 |
| Pi 串口打不开 | `raspi-config` 是否启用串口、`/dev/serial0` 权限、是否被串口登录占用 |
| USB-TTL 设备名不对 | `ls /dev/ttyUSB*` / `dmesg`；Windows 看设备管理器 |
| 编译报错 | 确认 arm-none-eabi-gcc 版本与 CMakePresets；HAL/CMSIS 文件是否完整 |

## 许可

STM32 HAL/CMSIS 文件保留 ST 原始版权与许可；本仓库新增代码按项目 Apache-2.0。
