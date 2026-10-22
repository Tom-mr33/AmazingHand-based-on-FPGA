#!/bin/bash

# init-vscode.sh
# 为 STM32 CubeMX CMake 项目生成 VSCode 配置文件（tasks.json 等）
# 用法：./init-vscode.sh [-f] [项目名]
#   -f    强制覆盖已存在的 tasks.json

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
VSCODE_DIR="$PROJECT_DIR/.vscode"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 解析参数
FORCE_OVERWRITE=false
PROJECT_NAME=""

while [ $# -gt 0 ]; do
    case "$1" in
        -f|--force)
            FORCE_OVERWRITE=true
            shift
            ;;
        -h|--help)
            echo "用法：./init-vscode.sh [-f] [项目名]"
            echo "  -f, --force    强制覆盖已存在的 tasks.json"
            echo "  项目名         指定项目名（默认从 CMakeLists.txt 自动提取）"
            exit 0
            ;;
        -*)
            echo -e "${RED}未知选项：$1${NC}"
            echo "用法：./init-vscode.sh [-f] [项目名]"
            exit 1
            ;;
        *)
            PROJECT_NAME="$1"
            shift
            ;;
    esac
done

# 从 CMakeLists.txt 中读取项目名
read_project_name() {
    local cmake_file="$PROJECT_DIR/CMakeLists.txt"
    if [ ! -f "$cmake_file" ]; then
        echo -e "${RED}错误：当前目录下找不到 CMakeLists.txt${NC}"
        return 1
    fi

    local name
    name=$(grep -E '^[[:space:]]*set\([[:space:]]*CMAKE_PROJECT_NAME[[:space:]]+' "$cmake_file" | sed -E 's/.*set\([[:space:]]*CMAKE_PROJECT_NAME[[:space:]]+([^[:space:])]+).*/\1/' | head -n 1)

    if [ -z "$name" ]; then
        echo -e "${YELLOW}警告：无法从 CMakeLists.txt 自动提取项目名${NC}"
        return 1
    fi

    echo "$name"
}

# 获取项目名：命令行参数优先，否则自动读取
if [ -n "$PROJECT_NAME" ]; then
    echo -e "${GREEN}使用命令行参数指定的项目名：$PROJECT_NAME${NC}"
else
    echo "尝试从 CMakeLists.txt 自动提取项目名..."
    if PROJECT_NAME=$(read_project_name); then
        echo -e "${GREEN}自动提取到项目名：$PROJECT_NAME${NC}"
    else
        echo -e "${RED}无法自动提取项目名，请手动指定：./init-vscode.sh <项目名>${NC}"
        exit 1
    fi
fi

# 检查 ELF 文件路径是否合理
ELF_PATH="build/Debug/${PROJECT_NAME}.elf"

# 创建 .vscode 目录
mkdir -p "$VSCODE_DIR"

# 生成 tasks.json
TASKS_FILE="$VSCODE_DIR/tasks.json"

if [ -f "$TASKS_FILE" ]; then
    if [ "$FORCE_OVERWRITE" = true ]; then
        cp "$TASKS_FILE" "$TASKS_FILE.bak.$(date +%Y%m%d%H%M%S)"
        echo "已备份原文件"
    else
        echo -e "${YELLOW}$TASKS_FILE 已存在${NC}"
        read -rp "是否覆盖？(y/N): " confirm
        if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
            echo "已取消，保留现有文件。"
            exit 0
        fi
        cp "$TASKS_FILE" "$TASKS_FILE.bak.$(date +%Y%m%d%H%M%S)"
        echo "已备份原文件"
    fi
fi

cat > "$TASKS_FILE" <<EOF
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build STM32 (Debug)",
            "type": "cmake",
            "command": "build",
            "preset": "Debug",
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": []
        },
        {
            "label": "Flash STM32 via ST-Link",
            "type": "shell",
            "command": "bash",
            "args": [
                "-c",
                "source ~/.bashrc && STM32_Programmer_CLI -c port=SWD freq=4000 -w \${workspaceFolder}/${ELF_PATH} -v -rst"
            ],
            "options": {
                "cwd": "\${workspaceFolder}"
            },
            "dependsOn": "Build STM32 (Debug)",
            "group": "build",
            "problemMatcher": []
        }
    ]
}
EOF

echo -e "${GREEN}已生成：$TASKS_FILE${NC}"

# 生成 c_cpp_properties.json（如果不存在）
CPP_PROPS_FILE="$VSCODE_DIR/c_cpp_properties.json"
if [ ! -f "$CPP_PROPS_FILE" ]; then
    cat > "$CPP_PROPS_FILE" <<EOF
{
    "configurations": [
        {
            "name": "STM32",
            "compileCommands": "\${workspaceFolder}/build/Debug/compile_commands.json"
        }
    ]
}
EOF
    echo -e "${GREEN}已生成：$CPP_PROPS_FILE${NC}"
else
    echo -e "${YELLOW}$CPP_PROPS_FILE 已存在，跳过${NC}"
fi

echo ""
echo -e "${GREEN}配置完成！${NC}"
echo "你可以在 VSCode 中使用："
echo "  Ctrl+Shift+P -> Tasks: Run Task -> Flash STM32 via ST-Link"
echo "或者直接按 Ctrl+Shift+B 运行默认构建任务。"
