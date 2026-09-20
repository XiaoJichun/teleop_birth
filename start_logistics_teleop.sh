#!/usr/bin/env bash
# 一键：物流双 Arm-6A Pico 遥操（入站/翻面/出站场景）
# Pico：XRoboToolkit → 选本机 IP → Send ON → 长按 Grip
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if ! command -v uv >/dev/null 2>&1; then
  echo "未找到 uv"
  exit 1
fi

mkdir -p logs/mcap/artixon

if [[ ! -x .venv/bin/python ]]; then
  uv sync --all-groups
fi

# 物流触觉（TACTO）+ 采集依赖
bash scripts/bootstrap_third_party.sh
uv sync --extra tacto --extra collect >/dev/null

# 若 SDK 未装，自动安装（无 sudo）
if ! uv run python -c "import xrobotoolkit_sdk" 2>/dev/null; then
  echo "首次使用：自动安装 XRobo（本地解压，无需 sudo）..."
  bash scripts/xrobo/install_xrobo.sh
fi

CFG=configs/setup/xrobo.yaml
EXTRACT=$(uv run python -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('$CFG').read_text()); print(c['pc_service']['extract_dir'])")
SVC_REL=$(uv run python -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('$CFG').read_text()); print(c['pc_service']['service_rel_dir'])")
VENV_SUB=$(uv run python -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('$CFG').read_text()); print(c['pybind']['venv_lib_subdir'])")
SVC_DIR="$ROOT/$EXTRACT/$SVC_REL"
export LD_LIBRARY_PATH="$ROOT/.venv/$VENV_SUB:$SVC_DIR:$SVC_DIR/lib:$SVC_DIR/SDK/x64:${LD_LIBRARY_PATH:-}"
export LC_ALL="${LC_ALL:-C.UTF-8}"

bash scripts/xrobo/ensure_xrobo_service.sh

echo
echo "========================================================================"
echo "  物流分拣 · Pico 遥操（ArtiXon Arm-6A ×2）"
echo "========================================================================"
echo "  1. 打开 App: XRoboToolkit"
echo "  2. 选本机 IP（热点优先 10.42.0.1）:"
hostname -I 2>/dev/null | tr ' ' '\n' | awk 'NF{print "     · "$1}'
echo "  3. WORKING → 打开 Send"
echo "  4. 长按该侧 Grip ≥0.2s 遥操；Trigger=夹爪；Y/B/Menu=回零"
echo "  5. 夹爪靠近快递盒并闭合即可抓起"
echo "  6. 指垫 DIGIT 触觉窗口（TACTO）：夹紧时可见接触图"
echo "========================================================================"
echo

echo "启动物流场景遥操..."
exec uv run python scripts/robots/artixon_teleop.py "$@"
