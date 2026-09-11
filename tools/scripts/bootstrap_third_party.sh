#!/usr/bin/env bash
# 拉取仿真依赖 mujoco_menagerie（不提交到 git）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MENAGERIE="$ROOT/third_party/mujoco_menagerie"
URL="${MUJOCO_MENAGERIE_URL:-https://github.com/google-deepmind/mujoco_menagerie.git}"

mkdir -p "$ROOT/third_party"

if [[ -d "$MENAGERIE/.git" ]] || [[ -f "$MENAGERIE/universal_robots_ur5e/ur5e.xml" ]]; then
  echo "mujoco_menagerie 已就绪: $MENAGERIE"
  exit 0
fi

echo "克隆 mujoco_menagerie → $MENAGERIE"
git clone --depth 1 "$URL" "$MENAGERIE"
echo "完成。"
