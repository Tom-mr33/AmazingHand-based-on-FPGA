# AmazingHand Codex Development Rules

## Project

- Repository: Tom-mr33/AmazingHand-based-on-FPGA
- Project: AmazingHand (STM32F103 + Raspberry Pi)
- Main hardware: STM32F103C8T6 (SCS bus) + Raspberry Pi (perception/strategy/UI)
- FPGA/Z19 route: removed 2026-10-20 (competition canceled; FPGA artifacts deleted)

These team rules apply to every future Codex agent working in this repository.
Read `docs/GIT_WORKFLOW.md`, the current development log, and relevant design
files before working. Preserve existing work and do not invent missing hardware
or software implementation from design documents.

## Main Branch Protection

main is a protected development baseline.

NEVER:

- develop or commit directly on main
- push directly to main, including pushing another local branch to remote main
- force push main
- merge code into main without a Pull Request
- bypass team review or another team member's Approve
- automatically merge Pull Requests (Codex must not merge PRs)
- delete remote branches automatically

Before every new task, inspect `git status` and `git branch --show-current`.
If uncommitted work exists, preserve it and stop to report the situation; do not
stash, discard, commit, or carry it to main without user authorization.
With a clean working tree, synchronize main first:

```sh
git checkout main
git pull --ff-only origin main
```

If synchronization fails, diverges, or cannot fast-forward, stop and report the
full error. Do not merge, rebase, or reset to resolve it automatically.
Then create a new branch from the updated main:

```sh
git checkout -b feature/<name>
# Alternatives:
git checkout -b fix/<name>
git checkout -b docs/<name>
git checkout -b chore/<name>
```

Use one appropriate command, not all four. Confirm the new branch before editing.

## Pull Request Rule

All changes must follow:

latest main → new working branch → development → local validation → git diff
review → commit → push working branch → Pull Request → review by another team
member → Approve → merge by an authorized human team member.

Codex must NEVER merge a PR, enable automatic merging, bypass review, or delete a
remote branch. Do not create a PR unless the user authorizes it.

## Before Editing

Before modifying any project file:

1. Run `git status`.
2. Run `git branch --show-current`.
3. Verify the branch is a named working branch and is NOT main.
4. Inspect the existing implementation and applicable instructions.
5. Explain intended changes.
6. Avoid unrelated modifications.

If the branch is main, detached, invalid, or unclear: STOP. Do not modify project
files. Normal checkout/pull operations for the required main synchronization are
allowed; feature edits and commits on main are not.

Before ANY Git write operation, check the current branch again. Do not change
branches or refs to conceal or bypass these rules. Preserve existing files and
history. Do not overwrite a nonempty clone destination.

## Before Commit

Before creating a commit:

1. Run `git status` and `git branch --show-current`.
2. Run `git diff`, inspect untracked files, and review `git diff --cached` if staged.
3. Inspect every changed file and confirm the branch is NOT main.
4. Confirm there are no generated temporary files or secrets.
5. Run available tests/build checks appropriate to the changes; report unavailable checks.
6. Summarize changes.
7. Stage explicit paths, preferably not `git add .`.

Do not commit unless explicitly requested by the user.

## Before Push

Before pushing:

1. Verify the branch name and confirm it is NOT main.
2. Run `git status` and verify the intended origin repository.
3. Show the commits that will be pushed (relative to the upstream if it exists,
   otherwise relative to origin/main).
4. Push only the current working branch to the same named remote branch.
5. Confirm `git config --get core.hooksPath` returns `.githooks`.

Never execute:

```sh
git push origin main
git push --force origin main
```

Do not push unless explicitly requested by the user. Do not use `--no-verify`,
disable hooks, change hooksPath, or use an alternate refspec to bypass protection.
Do not use broad pushes such as `--all` or `--mirror`.

## Local Guard and Team Enforcement

The tracked `.githooks/pre-push` rejects every update targeting
`refs/heads/main`, including deletion and explicit refspecs from other branches.
Activate it in each clone with repository-local configuration:

```sh
git config --local core.hooksPath .githooks
```

Local hooks are an accidental-push guard, not GitHub branch protection. Ask the
repository administrator to enforce PRs, at least one approval from another team
member, and no force pushes/deletions on main through repository rules. Do not
change remote repository settings without authorization.

## STM32 / Raspberry Pi Project Rules

This repository contains STM32F103 firmware (HAL/CMake), Raspberry Pi host code
(Python), C/C++, Linux, AI/computer vision, and robotic hand control. The FPGA/Z19
route was removed on 2026-10-20; do not recreate or reference it.

Before editing firmware, determine whether files are source, CubeMX/HAL generated
files, build products, or configuration. Prefer application source (`Core/Src/main.c`)
and reproducible CMake configuration. Do not blindly edit generated HAL/CMSIS files
or delete firmware project files.

Do not ignore necessary source, CMake, linker/startup files, or critical project
configuration. Review existing `.gitignore` and suggest changes before broad
updates. Only obvious logs, caches, and temporary build files may be cautiously
added without compromising reproducibility.

Read `docs/design/architecture_mcu_rpi.md` before Pi↔STM32 interface changes.
Record authorized structure, interface, code, or parameter changes in
`docs/log/YYYY-MM-DD.md`. Distinguish implemented features from planned features in
reports. Do not install board-targeted dependencies into a PC environment without
checking compatibility.

## Safety

Never execute destructive Git commands without explicit user authorization:

```sh
git reset --hard
git clean -fd
git clean -fdx
git push --force
git push --force-with-lease
# Also prohibited: rebase followed by force push
```

Do not remove, move, or overwrite existing user files to make a task succeed.
Do not alter global Git configuration, substitute repository URLs, use someone
else's token, or bypass access permissions. Never print credentials.
If unsure, stop and ask with the concrete issue and preserved state.

Unless explicitly authorized, do not modify business code, STM32 firmware,
Raspberry Pi applications, AI models, or hand-control code during workflow/setup tasks.
