#!/usr/bin/env bash
# teleop_birth 一键启动：进入交互式终端主菜单（uv 管理环境）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if ! command -v uv >/dev/null 2>&1; then
  echo "未找到 uv。请先安装: https://docs.astral.sh/uv/"
  exit 1
fi

if [[ ! -d .venv ]] || [[ ! -x .venv/bin/teleop-birth && ! -x .venv/bin/python ]]; then
  echo "首次启动或环境不完整，正在 uv sync ..."
  uv sync --all-groups
fi

if [[ ! -x .venv/bin/teleop-birth ]]; then
  uv sync --all-groups
fi

# 仿真用 menagerie（缺失则浅克隆；体积大，不进 git）
bash tools/scripts/bootstrap_third_party.sh

exec uv run teleop-birth "$@"
