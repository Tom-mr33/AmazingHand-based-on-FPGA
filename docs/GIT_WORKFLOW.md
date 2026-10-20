# AmazingHand 团队 Git 工作流

适用仓库：`Tom-mr33/AmazingHand-based-on-FPGA`。所有成员与 Codex 均遵守：
**最新 main → 独立分支 → 开发与验证 → 提交 → 推送当前分支 → PR → 其他队员 Review / Approve → 人工合并。**

## 1. 首次获取项目

先检查目标目录；只在目录不存在或为空时 clone，非空目录不得删除、覆盖或移动。

```powershell
git clone https://github.com/Tom-mr33/AmazingHand-based-on-FPGA.git F:\Amazing_Hand
Set-Location F:\Amazing_Hand
git status
git remote -v
git branch -a
```

origin 必须是 `https://github.com/Tom-mr33/AmazingHand-based-on-FPGA.git`。
认证失败时完成自己的 GitHub 身份认证，不更换地址、不借用他人的 Token。

本规范文件经过 PR 合并后，每位队员在各自 clone 中执行：

```sh
git branch --show-current
git config --local core.hooksPath .githooks
git config --get core.hooksPath
```

最后一条应输出 `.githooks`。此设置不会随 clone 自动继承，也不会修改全局配置。
在 Linux/macOS 上还需确认 hook 有执行权限，必要时在工作分支执行
`chmod +x .githooks/pre-push`。脚本使用 LF、无 BOM，兼容 Windows Git Bash。

### 当前 Windows 主机的代理

浏览器的系统代理不等于 Git 已配置代理。此次恢复下载使用系统现有 HTTP 代理
`http://127.0.0.1:7897`；仅当前仓库保存了 `http.proxy`，没有更改全局 Git 配置。
该地址是本机配置，不要求其他成员照抄。代理软件必须运行；更换端口时先核实，
再在确认当前分支后更新当前仓库配置。不要在命令、日志或文档中写入代理密码或 Token。

首次 clone 如需代理，可使用仅本次命令生效的配置（端口须以自己的配置为准）：

```powershell
git -c http.proxy=http://127.0.0.1:7897 clone https://github.com/Tom-mr33/AmazingHand-based-on-FPGA.git F:\Amazing_Hand
```

## 2. 每次开始工作

先查看现有状态；有未提交改动时保留现场，不直接切换 main，不自动 stash、提交或丢弃。
确认工作区干净后执行：

```sh
git status
git branch --show-current
git checkout main
git branch --show-current
git pull --ff-only origin main
```

必须使用 `--ff-only`。如果失败、分叉或无法快进，停止并完整报告，不自行 merge、
rebase、reset。每次 Git 写操作之前都必须确认当前分支。

## 3. 创建工作分支

从刚同步的 main 创建一个分支，例如：

```sh
git branch --show-current
git checkout -b feature/servo-calibration
git branch --show-current
```

命名使用 `feature/<功能名称>`、`fix/<问题名称>`、`docs/<文档名称>` 或
`chore/<维护名称>`，建议用英文短横线命名。确认已离开 main 后才能编辑文件。
本次规范建立分支为 `chore/dev-workflow-setup`。

## 4. 开发

修改前执行 `git status`、`git branch --show-current`，阅读已有实现、`AGENTS.md`
与相关设计文档，说明修改目的，只做当前任务范围内的改动。结构、接口、代码和参数
变更记入当日 `docs/log/`。不要盲改 CubeMX/HAL 生成文件，不误忽略源码、CMake、
链接脚本/启动文件或关键配置，不擅自删除固件工程文件。

## 5. 本地检查与提交

运行项目已有且适用于改动的测试、仿真或构建；没有可执行检查时说明原因，不声称通过。
提交前检查所有改动和未跟踪文件，排除临时产物与密钥：

```sh
git status
git branch --show-current
git diff
# 下面的路径仅是示例；逐项替换为本次审核过的真实文件。
git add docs/GIT_WORKFLOW.md
git diff --cached
git branch --show-current
git commit -m "docs: document team git workflow"
```

功能提交可以使用 `feat: xxx`，修复使用 `fix: xxx`，维护使用 `chore: xxx`。
优先 `git add <明确的文件>`，不要无脑 `git add .`。
**Codex 只有获得用户明确提交指令后才能执行 add/commit；本轮不提交。**

## 6. 只推送当前工作分支

推送前检查分支、状态、origin、hook 配置及将推送的提交。首次推送可查看相对 main
的提交；已有上游时查看相对该上游的未推送提交：

```sh
git branch --show-current
git status
git remote -v
git config --get core.hooksPath
git log --oneline origin/main..HEAD
# 已有上游时可用：git log --oneline "@{upstream}..HEAD"
```

只有当前确实位于示例分支时才执行以下命令；使用自己的真实分支名替换示例：

```sh
git branch --show-current
git push -u origin feature/servo-calibration
```

**Codex 只有获得用户明确推送指令后才能 push；本轮不推送。**
不推送其他分支，不使用 `--all`、`--mirror`、`--no-verify` 或绕过 hook 的配置。

## 7. Pull Request、Review 与合并

1. 在 GitHub 创建 PR：base 为 `main`，compare 为自己的工作分支。
2. 说明问题、改动、验证结果、限制及相关硬件影响。
3. 邀请其他团队成员 Code Review，按反馈继续修改和验证。
4. 必须由其他队员明确 **Approve**；自己检查或留言不能代替审批。
5. 满足团队要求后，由有权限的团队成员人工合并到 main。
6. Codex 不得自行 merge、启用自动合并、绕过审批或删除远程分支。

本轮不创建 PR。以后也应在用户授权的任务范围内创建 PR。

## 8. 合并后与下一项任务

确认本地改动已妥善处理，再同步 main，然后为新任务创建新分支：

```sh
git status
git branch --show-current
git checkout main
git branch --show-current
git pull --ff-only origin main
git branch --show-current
git checkout -b feature/next-task
```

不要继续在已经合并的旧分支上开发新功能；Codex 不自行删除远程分支。

## 9. 明确禁止的操作

禁止直接推送 main，包括：

```sh
git push origin main
git push --force origin main
git push origin HEAD:main
```

禁止直接在 main 开发、提交，禁止绕过 PR 或其他队员审批。
未经明确授权，不执行 `git reset --hard`、`git clean -fd`、`git clean -fdx`、
`git push --force`、`git push --force-with-lease`、rebase 后强推等破坏历史或现场的操作。

## 10. 本地 hook 与 GitHub 保护

`.githooks/pre-push` 检查推送输入的**远程目标 ref**，任何指向 `refs/heads/main`
的更新均拒绝，包含从别的分支推往 main 以及删除 main。正常工作分支可以通过此项检查。
拒绝时提示：

```text
ERROR: Direct push to main is forbidden.
Please push a feature branch and create a Pull Request.
```

不要为了测试而真正向 main push。检查脚本内容、LF 格式、shell 语法和
`core.hooksPath` 即可。

本地 hook 只防误操作，不能强制其他 clone 或 GitHub 网页上的 Review。建议仓库
管理员配置 main 的分支保护或 Ruleset：必须 PR、至少一位其他成员 Approve、禁止
强推及删除，并限制绕过。远程规则的配置状态需另行核实，本轮没有修改 GitHub 设置。
