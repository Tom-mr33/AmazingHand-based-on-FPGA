# Host → MuJoCo 实时控制

复用 `GraspSelector`、`SimBackend` 抓取状态机及协议总线，替换原来的线性模拟动力学，
实际调用 `mujoco.mj_step`，从 `qpos` / `actuator_force` 返回反馈。输入来自当次按键、
按钮或本地 API，没有自动播放序列。选择 banana/apple/bottle 使用仓库同名抓取策略；
当前展示手部姿态控制，场景未放置可抓物体，不声称物体已被抓稳。

> MuJoCo 只在 PC 上运行，不安装到树莓派。2026-10-20 起主链路为树莓派 + STM32。

## 启动

从仓库根目录运行（Windows / Python 3.12）：

~~~powershell
git submodule update --init --depth 1 AmazingHand
uv venv --python 3.12 .venv
uv pip install --python .venv/Scripts/python.exe -r sim/requirements-live.txt
.venv/Scripts/python.exe host/app/main.py --mujoco
~~~

`.gitmodules` 补齐已有模型子模块 URL；版本仍固定在原 gitlink `3e824107`，不改上游模型。
没有 uv 时可用 `py -3.12 -m venv .venv` 和 `.venv/Scripts/python.exe -m pip install -r sim/requirements-live.txt`。
不需要安装整个板端 `requirements.txt`。

按 **1/2/3** 选择香蕉/苹果/瓶子；**O** 张开、**G** 抓取、**H** 保持当前位置、
**R** 释放、**S** 停止、**E** 显式复位，**Q/Esc** 或窗口 X 退出。鼠标按钮也可操作；
拖动场景旋转，滚轮缩放。初始保持不动，只有输入命令才开始运动。

面板显示控制状态、实测角度、实际执行器控制量和仿真负载。
STOP 清除旧目标、禁用执行器并暂停物理状态，拒绝运动命令直到复位；复位不重放旧目标。
主循环超期 500 ms 或物理步进异常会停止。关窗/正常结束会关闭后端，SIGINT/SIGTERM 请求退出。
此处停机是模拟器语义，不代表物理机械手断力矩已验收。

## 在线输入与验收

窗口运行时，另一个 PowerShell 可发送指令、读取实际反馈：

~~~powershell
Invoke-RestMethod http://127.0.0.1:8767/command -Method Post -ContentType application/json -Body '{"action":"grasp","category":"apple"}'
Invoke-RestMethod http://127.0.0.1:8767/state
~~~

API 仅绑定 loopback，返回 202 代表已入队；实际接受/拒绝结果见 `/state` 的 `events`。
可用 `--headless` 运行相同真实物理/API而不显示窗口；`--duration 20 --snapshot tmp/panel.png`
可限时退出并保存末帧（snapshot 仅 GUI）。后台 API 不应暴露给局域网。

本机验证：5 项真实 MuJoCo 控制测试、原 backend 10 项和 sim 5 项通过；
独立 GUI 进程通过 API 完成张开→抓取→保持→释放及停止/拒绝运动/复位，
末轮实测关节变化约 48.2°、停止后位置不变且执行器输出为零，正常退出码 0。
证据见 [docs/evidence/2026-10-17/issue10/README.md](../docs/evidence/2026-10-17/issue10/README.md)。

~~~powershell
.venv/Scripts/python.exe host/tests/test_mujoco_live.py
~~~

角度按模型零位/弧度驱动，显示度；只在 SIM 适配中将预张开角改成模型零位，原抓取表不变。
raw 沿用旧 golden profile（±π 映射 0..4095），负载 `abs(actuator_force)*100` 饱和至 1023，
速度 raw3 在此演示为 60°/s。以上均非 SCS0009 实物标定。
