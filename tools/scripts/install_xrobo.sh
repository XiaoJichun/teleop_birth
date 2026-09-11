#!/usr/bin/env bash
# 本地安装 XRobo（PC Service + adb + APK + pybind SDK），无需 sudo
# - Python 包走 uv（.venv）
# - adb / PC Service 等非 Python 依赖落在 third_party/，由本脚本管理
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

CFG="$ROOT/configs/xrobo_setup.yaml"
if [[ ! -f "$CFG" ]]; then
  echo "缺少 $CFG"
  exit 1
fi

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

if ! command -v uv >/dev/null 2>&1; then
  echo "未找到 uv"
  exit 1
fi
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then
  echo "首次同步环境..."
  uv sync --all-groups
fi

DEB_URL="$(read_cfg pc_service.deb_url)"
DEB_NAME="$(read_cfg pc_service.deb_filename)"
DL_DIR="$ROOT/$(read_cfg pc_service.download_dir)"
EXTRACT="$ROOT/$(read_cfg pc_service.extract_dir)"
SVC_REL="$(read_cfg pc_service.service_rel_dir)"
SVC_DIR="$EXTRACT/$SVC_REL"
APK_URL="$(read_cfg pico_apk.apk_url)"
APK_NAME="$(read_cfg pico_apk.apk_filename)"
PT_URL="$(read_cfg adb.platform_tools_url)"
PT_ZIP="$(read_cfg adb.zip_filename)"
PT_DIR="$ROOT/$(read_cfg adb.install_dir)"
SHIM_DIR="$ROOT/$(read_cfg adb.shim_dir)"
PYBIND_DIR="$ROOT/$(read_cfg pybind.local_dir)"
PYBIND_REPO="$(read_cfg pybind.repo_url)"
VENV_LIB="$ROOT/.venv/$(read_cfg pybind.venv_lib_subdir)"

mkdir -p "$DL_DIR" "$EXTRACT" "$SHIM_DIR"

echo "========== [1/5] 下载并解压 PC Service（无需 sudo） =========="
DEB_PATH="$DL_DIR/$DEB_NAME"
if [[ ! -f "$DEB_PATH" ]]; then
  curl -L --fail -o "$DEB_PATH" "$DEB_URL"
fi
if [[ ! -x "$SVC_DIR/RoboticsServiceProcess" ]]; then
  echo "解压到 $EXTRACT ..."
  dpkg-deb -x "$DEB_PATH" "$EXTRACT"
fi
test -x "$SVC_DIR/RoboticsServiceProcess"
echo "PC Service 就绪: $SVC_DIR"

echo "========== [2/5] 安装 adb（platform-tools，不走 uv） =========="
ADB_BIN="$PT_DIR/adb"
if [[ ! -x "$ADB_BIN" ]]; then
  ZIP_PATH="$DL_DIR/$PT_ZIP"
  if [[ ! -f "$ZIP_PATH" ]]; then
    curl -L --fail -o "$ZIP_PATH" "$PT_URL"
  fi
  echo "解压 platform-tools → $(dirname "$PT_DIR") ..."
  mkdir -p "$(dirname "$PT_DIR")"
  # zip 内顶层目录名为 platform-tools/
  unzip -o -q "$ZIP_PATH" -d "$(dirname "$PT_DIR")"
fi
test -x "$ADB_BIN"
ln -sfn "$ADB_BIN" "$SHIM_DIR/adb"
export PATH="$SHIM_DIR:$PT_DIR:$PATH"
echo "adb: $("$ADB_BIN" version | head -n 1)"
echo "PATH 入口: $SHIM_DIR/adb"

echo "========== [3/5] 下载 Pico APK（USB 连接时自动安装） =========="
APK_PATH="$DL_DIR/$APK_NAME"
if [[ ! -f "$APK_PATH" ]]; then
  curl -L --fail -o "$APK_PATH" "$APK_URL"
fi
echo "APK: $APK_PATH"
if "$ADB_BIN" get-state 2>/dev/null | grep -q device; then
  echo "检测到头显，自动安装 APK..."
  "$ADB_BIN" install -r "$APK_PATH" || true
else
  echo "（可选）头显开开发者模式并 USB 连接后执行:"
  echo "  $SHIM_DIR/adb install -r \"$APK_PATH\""
  echo "  或: bash tools/scripts/install_xrobo.sh   # 会再次尝试自动安装"
fi

echo "========== [4/5] 编译安装 xrobotoolkit_sdk 到 uv 环境 =========="
uv pip install pybind11
if [[ ! -d "$PYBIND_DIR/.git" ]]; then
  git clone --depth 1 "$PYBIND_REPO" "$PYBIND_DIR"
fi
mkdir -p "$PYBIND_DIR/lib" "$PYBIND_DIR/include" "$VENV_LIB"
cp -f "$SVC_DIR/SDK/include/PXREARobotSDK.h" "$PYBIND_DIR/include/"
cp -f "$SVC_DIR/SDK/x64/libPXREARobotSDK.so" "$PYBIND_DIR/lib/"
cp -f "$SVC_DIR/SDK/x64/libPXREARobotSDK.so" "$VENV_LIB/"
# nlohmann 若缺失则下载单头文件
if [[ ! -f "$PYBIND_DIR/include/nlohmann/json.hpp" ]]; then
  mkdir -p "$PYBIND_DIR/include/nlohmann"
  curl -L --fail -o "$PYBIND_DIR/include/nlohmann/json.hpp" \
    https://raw.githubusercontent.com/nlohmann/json/v3.11.3/single_include/nlohmann/json.hpp
fi

BUILD_DIR="$ROOT/third_party/xrobo_pybind_build"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
PYBIND11_DIR="$("$ROOT/.venv/bin/python" -m pybind11 --cmakedir)"
cmake -S "$PYBIND_DIR" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_EXECUTABLE="$ROOT/.venv/bin/python" \
  -Dpybind11_DIR="$PYBIND11_DIR"
cmake --build "$BUILD_DIR" -j"$(nproc)"
SITE="$("$ROOT/.venv/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
cp -f "$BUILD_DIR"/xrobotoolkit_sdk.cpython-*-linux-gnu.so "$SITE/"

echo "========== [5/5] 验证 =========="
export LD_LIBRARY_PATH="$VENV_LIB:$SVC_DIR/SDK/x64:${LD_LIBRARY_PATH:-}"
# 确保服务在跑再 init
bash "$ROOT/tools/scripts/ensure_xrobo_service.sh"
uv run python -c "import xrobotoolkit_sdk as x; x.init(); print('xrobotoolkit_sdk OK'); x.close()"
echo
echo "安装完成。"
echo "  · Python SDK : uv / .venv"
echo "  · adb        : $SHIM_DIR/adb  （configs/xrobo_setup.yaml → adb.*）"
echo "  · 一键仿真   : ./start_pico_sim.sh"
echo "  · 或菜单     : ./start.sh →「一键：Pico 仿真遥操」"
