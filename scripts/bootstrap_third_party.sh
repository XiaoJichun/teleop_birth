#!/usr/bin/env bash
# 拉取仿真依赖 mujoco_menagerie + TACTO 触觉库（不提交到 git）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MENAGERIE="$ROOT/third_party/mujoco_menagerie"
URL="${MUJOCO_MENAGERIE_URL:-https://github.com/google-deepmind/mujoco_menagerie.git}"
TACTO="$ROOT/third_party/tacto"
TACTO_URL="${TACTO_URL:-https://github.com/facebookresearch/tacto.git}"

mkdir -p "$ROOT/third_party"

if [[ -d "$MENAGERIE/.git" ]] || [[ -f "$MENAGERIE/universal_robots_ur5e/ur5e.xml" ]]; then
  echo "mujoco_menagerie 已就绪: $MENAGERIE"
else
  echo "克隆 mujoco_menagerie → $MENAGERIE"
  git clone --depth 1 "$URL" "$MENAGERIE"
  echo "完成。"
fi

if [[ -f "$TACTO/tacto/__init__.py" ]]; then
  echo "tacto 已就绪: $TACTO"
else
  echo "克隆 TACTO → $TACTO"
  echo "  来源: https://github.com/facebookresearch/tacto"
  git clone --depth 1 "$TACTO_URL" "$TACTO"
  echo "完成。"
fi

echo "可选依赖: uv sync --extra tacto"