# STM32 控制 AmazingHand 协议与状态机

> 目标：STM32F103C8T6 作为**实时控制器**，负责 1 Mbps SCS 总线调度、抓取 FSM、
> 力控/超时/掉线/过温保护与遥测；树莓派只发高层命令、收遥测、做视觉/策略/UI。
> 这是对原 FPGA（servo_bus_ctrl + grasp_fsm + axi_lite_regs）的 C 语言替代。

## 1. 分工

| 层 | 运行位置 | 职责 |
|---|---|---|
| 高层 | Raspberry Pi / PC | 语音、YOLO、测尺寸、抓取策略、主状态机、UI、标定参数 |
| 实时层 | STM32F103 | SCS 帧收发、2 ms 时隙调度、8 舵机反馈轮询、抓取 FSM、保护、看门狗、遥测 |
| 执行器 | 8 × SCS0009 | 单总线 1 Mbps 半双工 |

Pi 不直接发 SCS 帧（除调试用 RAW_SCS）；STM32 不使用 Python 状态机。
Pi↔STM32 链路 115200 8N1，只承载高层命令/遥测，不再受 1 Mbps 总线时序影响。

## 2. 帧格式（Pi ↔ STM32）

固定二进制帧，小端：

~~~
偏移  0     1     2      3      4     5      6..5+LEN   6+LEN  7+LEN
    0xAA  0x55  VER=1  TYPE   SEQ   LEN    PAYLOAD     CRC_L  CRC_H
~~~

- LEN = 0..64，帧长 = LEN + 8；
- CRC16-CCITT（poly 0x1021，init 0xFFFF，非反射）覆盖
  `VER TYPE SEQ LEN PAYLOAD`，低字节在前；
- 解析器在坏 CRC/非法 LEN 时丢弃并重新找 0xAA 0x55；
- 每帧最多 72 B，115200 下最坏约 6.3 ms，遥测 20 ms 周期留足余量。

## 3. 消息类型

| TYPE | 方向 | 名称 | Payload |
|---|---|---|---|
| 0x01 | Pi→STM32 | HELLO | u8 host_ver |
| 0x02 | STM32→Pi | INFO | u8 proto, u8 fw_major, u8 fw_minor, u8 n_servos, u16 telemetry_ms, u16 watchdog_ms |
| 0x03 | Pi→STM32 | COMMAND | 见 §4，61 B |
| 0x04 | STM32→Pi | TELEMETRY | 见 §5，63 B |
| 0x05 | Pi→STM32 | RAW_SCS | 原始 SCS 请求帧（≤64 B） |
| 0x05 | STM32→Pi | RAW_SCS | 原始 SCS 响应（0 B = 总线静默） |
| 0x06 | Pi→STM32 | HEARTBEAT | 空，刷新看门狗 |
| 0x07 | Pi→STM32 | CONFIG | u16 watchdog_ms,u16 telemetry_ms,u16 slot_ms,u16 pregrasp_ms,u16 closing_timeout_ms,u16 release_ms |
| 0x08 | STM32→Pi | ERROR | u8 code, u8 arg |
| 0x09 | Pi→STM32 | DIRECT_TARGETS | u16 speed_raw + 8 × u16 target_raw（直接角度模式） |
| 0x0A | Pi→STM32 | SET_TORQUE | u8 on（0/1） |

## 4. COMMAND payload（61 B）

~~~
u8  cmd              1=OPEN 2=GRASP 3=HOLD 4=RELEASE 5=STOP 6=RESET
u16 speed_raw
u8  torque_levels[8] 0..15
u16 pregrasp_raw[8]  OPEN/GRASP/RELEASE 用
u16 close_raw[8]     GRASP 用
u16 load_threshold_raw[8] 接触判定阈值
u16 closing_timeout_ms
~~~

- 所有位置/阈值均为 SCS raw（0..4095），与 Pi 侧标定表换算；
- STM32 不做度→raw 换算，只执行 raw 目标；
- OPEN = 力矩开 + 下发 pregrasp_raw；GRASP = PREGRASP→CLOSING→HOLD；
- RELEASE = 下发 pregrasp_raw + 释放计时；STOP = 断力矩并锁存 PROTECT；
- RESET = 清锁存、断力矩、回 IDLE（不恢复旧动作）。

## 5. TELEMETRY payload（63 B）

~~~
u8  state          0 IDLE,1 PREGRASP,2 CLOSING,3 HOLD,4 RELEASING,5 PROTECT
u8  fault          位图：1 watchdog,2 stop,4 offline,8 seq/非法,16 overload,32 overtemp,64 crc
u8  flags          bit0 torque, bit1 moving, bit2 timeout, bit3 grasped
u8  online_mask
u8  seq_ack
u32 tick_ms
u16 pos_raw[8]
u16 load_raw[8]
u8  temp_c[8]
u8  error_bits[8]
u16 command_count
u16 feedback_count
u16 max_age_ms
~~~

## 6. STM32 状态机

状态与 `backend_sim.FsmState` 一致：IDLE/PREGRASP/CLOSING/HOLD/RELEASING/PROTECT。

| 状态 | 行为 | 迁移 |
|---|---|---|
| IDLE | 力矩按命令；可接受 OPEN/GRASP/DIRECT | GRASP→PREGRASP；STOP→PROTECT |
| PREGRASP | 下发 pregrasp_raw，等 pregrasp_ms | 计时到→CLOSING；超时/故障→PROTECT |
| CLOSING | 下发 close_raw；每 2 ms 轮询负载 | 任一通道 load ≥ threshold→锁存实测位置→HOLD；closing_timeout→HOLD(timeout)；掉线/过温/急停→PROTECT |
| HOLD | 保持锁存位置；持续监督 | RELEASE→RELEASING；STOP/故障→PROTECT |
| RELEASING | 下发 pregrasp_raw，等 release_ms | 计时到→IDLE；故障→PROTECT |
| PROTECT | 断力矩、锁存故障 | 仅 RESET→IDLE |

保护规则：
- 连续 500 ms 未收到 Pi 任意合法帧 → 断力矩 + PROTECT（watchdog）；
- 任一舵机离线 → fault，PROTECT；
- SCS 返回过温/过载/角度错误 → fault，PROTECT；
- STOP 立即断力矩并锁存，不接受 OPEN/GRASP/HOLD 直到 RESET；
- RESET 不恢复旧动作、不自动上力矩。

## 7. SCS 总线调度

- 1 ms tick；每 2 ms 一个时隙；
- t0：SYNC_WRITE 8 舵机目标位置（32 B ≈ 320 µs）；
- t1/t2：READ 2 个反馈（8 B 请求 + 14 B 响应 ≈ 220 µs/路），8 路轮询周期 8 ms；
- 剩余时间给电源/温度/错误统计；
- 事件事务（速度/力矩）在下一个空闲时隙发送；
- 超时/错误有界，不能阻塞下一时隙；反馈 age 超限报 fault。

## 8. Pi 侧后端

新文件 `host/app/control/backend_stm32.py`（`ServoBackend` 实现）：

- `open()`：打开串口、发 HELLO、等 INFO（1 s），失败直接报错，不回退软件 FSM；
- 后台读线程解析 TELEMETRY/INFO/ERROR，保存最新快照；
- `start_grasp_fsm(plan)` → COMMAND GRASP（度→raw 用标定中位）；
- `set_targets` → DIRECT_TARGETS；`release` → COMMAND RELEASE；
  `emergency_stop` → COMMAND STOP；`set_torque` → SET_TORQUE；
- `poll_fsm()` → 最新 TELEMETRY；超过 1 s 无遥测 → fault；
- `tick()` 每 100 ms 发 HEARTBEAT；
- 不实现 HIL/字节泵回退。

`backend_mcu.py`（原始 SCS 透传）保留用于**首次 bring-up/标定**；
`backend_stm32.py` 用于正式控制。

## 9. 分阶段

1. **Phase 1（已有）**：STM32 字节泵 + Pi 软件 FSM；烧录、单舵机、8 ID、标定。
2. **Phase 2（本方案）**：STM32 协议 + SCS 调度 + FSM/保护/看门狗；Pi `backend_stm32`。
3. **Phase 3**：Pi 真实语音/视觉接入；演示与指标。

## 10. bring-up 顺序

1. 先烧现有 byte-pump 固件，`blink_servo --via mcu` 单舵机；
2. `probe_servos --via mcu` 扫 8 ID，记录型号/方向/中位；
3. 烧 control 固件，发 HELLO/INFO；`backend_stm32` 连接；
4. 单舵机 DIRECT_TARGETS / SET_TORQUE；
5. GRASP/RELEASE/STOP/RESET；断开 Pi 验证 500 ms watchdog；
6. 整手标定后做抓取。

## 11. 当前实现状态（2026-10-20）

- [x] Pi 侧协议编解码：`host/app/control/stm32_protocol.py`（13 项单测）
- [x] Pi 侧后端：`host/app/control/backend_stm32.py`（4 项 FakeSerial 单测）
- [x] STM32 侧可移植协议：`firmware/stm32/Core/{Inc,Src}/ah_protocol.*`
  （gcc 主机测试 `AH_PROTOCOL_TEST_OK`）
- [ ] `main.c` 接入 1 ms 调度 + SCS 总线 + FSM/看门狗
- [ ] 实机编译/烧录/单舵机/8 ID/标定
- [ ] Pi 视觉/语音接入

## 12. 安全边界

- 5V 独立供电、共地、保险丝；PA9 5V 容忍，必要时 3.3V↔5V 电平转换；
- 首次上电只接单舵机；急停 = 直接断舵机 5V；
- STOP/掉线/看门狗必须断力矩；PROTECT 只能 RESET 解除；
- 无硬件实测前，不声称抓取/保护已验收。
