#!/usr/bin/env bash
# 一键：启动 PC Service + MuJoCo 双臂仿真遥操
# Pico 端只需：打开 XRoboToolkit → 选本机 IP → Send ON → 握 Grip
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if ! command -v uv >/dev/null 2>&1; then
  echo "未找到 uv"
  exit 1
fi

mkdir -p logs

if [[ ! -x .venv/bin/python ]]; then
  uv sync --all-groups
fi

# 仿真模型（menagerie）
bash scripts/bootstrap_third_party.sh

# 若 SDK 未装，自动安装（无 sudo）
if ! uv run python -c "import xrobotoolkit_sdk" 2>/dev/null; then
  echo "首次使用：自动安装 XRobo（本地解压，无需 sudo）..."
  bash scripts/xrobo/install_xrobo.sh
fi

# 导出动态库路径
CFG=configs/setup/xrobo.yaml
EXTRACT=$(uv run python -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('$CFG').read_text()); print(c['pc_service']['extract_dir'])")
SVC_REL=$(uv run python -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('$CFG').read_text()); print(c['pc_service']['service_rel_dir'])")
VENV_SUB=$(uv run python -c "import yaml; from pathlib import Path; c=yaml.safe_load(Path('$CFG').read_text()); print(c['pybind']['venv_lib_subdir'])")
SVC_DIR="$ROOT/$EXTRACT/$SVC_REL"
export LD_LIBRARY_PATH="$ROOT/.venv/$VENV_SUB:$SVC_DIR:$SVC_DIR/lib:$SVC_DIR/SDK/x64:${LD_LIBRARY_PATH:-}"
export LC_ALL="${LC_ALL:-C.UTF-8}"

bash scripts/xrobo/ensure_xrobo_service.sh

# 打印本机 IP，方便 Pico 选择
echo
echo "========================================================================"
echo "  Pico / XRoboToolkit 操作"
echo "========================================================================"
echo "  1. 打开 App: XRoboToolkit"
echo "  2. 在设备列表选本机 IP（热点优先 10.42.0.1；同网段选真实网卡 IP）:"
hostname -I 2>/dev/null | tr ' ' '\n' | awk 'NF{print "     · "$1}'
echo "  3. 状态变成 WORKING → 打开 Send"
echo "  4. 长按该侧 Grip ≥0.35s 开始遥操（松开停止）；左右各自独立"
echo "  5. 左手 Y/Menu 或 右手 B/Menu = 该侧手臂回零"
echo "  6. Trigger = 夹爪"
echo "========================================================================"
echo

# 读取 one_click overrides
mapfile -t OVERRIDES < <(uv run python - <<'PY'
import yaml
from pathlib import Path
c = yaml.safe_load(Path("configs/setup/xrobo.yaml").read_text(encoding="utf-8"))
for item in c["one_click_pico_sim"]["overrides"]:
    print(item)
PY
)

echo "启动仿真遥操..."
exec uv run python run_teleop.py "${OVERRIDES[@]}"
