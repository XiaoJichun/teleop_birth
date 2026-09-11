#!/usr/bin/env bash
# 确保本地解压的 RoboticsServiceProcess 在运行
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG="$ROOT/configs/xrobo_setup.yaml"

read_cfg() {
  local key="$1"
  local py=""
  if [[ -x "$ROOT/.venv/bin/python" ]]; then
    py="$ROOT/.venv/bin/python"
  else
    py="$(command -v python3)"
  fi
  "$py" -c '
import sys, yaml
from pathlib import Path
cfg = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
cur = cfg
for k in sys.argv[2].split("."):
    cur = cur[k]
print(cur)
' "$CFG" "$key"
}

service_running() {
  # Linux truncates /proc/*/comm to 15 chars → "RoboticsService"
  pgrep -f '[R]oboticsServiceProcess' >/dev/null 2>&1 \
    || pgrep -x 'RoboticsService' >/dev/null 2>&1
}

EXTRACT="$ROOT/$(read_cfg pc_service.extract_dir)"
SVC_DIR="$EXTRACT/$(read_cfg pc_service.service_rel_dir)"
PROC="$(read_cfg pc_service.process_name)"
VENV_LIB="$ROOT/.venv/$(read_cfg pybind.venv_lib_subdir)"

if [[ ! -x "$SVC_DIR/$PROC" ]]; then
  echo "未找到 PC Service，先运行: bash tools/scripts/install_xrobo.sh"
  exit 1
fi

export LD_LIBRARY_PATH="$SVC_DIR:$SVC_DIR/lib:$SVC_DIR/SDK/x64:$VENV_LIB:${LD_LIBRARY_PATH:-}"
export QT_PLUGIN_PATH="$SVC_DIR/plugins/:${QT_PLUGIN_PATH:-}"
export QT_QML_PATH="$SVC_DIR/qml/:${QT_QML_PATH:-}"
export LC_ALL="${LC_ALL:-C.UTF-8}"
export LANG="${LANG:-C.UTF-8}"

mkdir -p "$ROOT/logs" "$HOME/.local/share/PICOBusinessSuitData/log" 2>/dev/null || true

if service_running; then
  echo "PC Service 已在运行"
  exit 0
fi

echo "启动 PC Service: $SVC_DIR/$PROC"
cd "$SVC_DIR"
nohup "./$PROC" >"$ROOT/logs/xrobo_pc_service.log" 2>&1 &
sleep 1.5
if service_running; then
  echo "PC Service 已启动（日志: logs/xrobo_pc_service.log）"
else
  echo "PC Service 启动失败，见 logs/xrobo_pc_service.log"
  tail -n 40 "$ROOT/logs/xrobo_pc_service.log" || true
  exit 1
fi
