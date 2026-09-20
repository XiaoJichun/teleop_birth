#!/usr/bin/env bash
# teleop_birth 一键启动：进入交互式终端主菜单（uv 管理环境）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if ! command -v uv >/dev/null 2>&1; then
  echo "未找到 uv。请先安装: https://docs.astral.sh/uv/"
  exit 1
fi

# 场景工坊需要 PyQt6（已列入主依赖）；一并装上采集/可视化 extras
_sync() {
  uv sync --all-groups --extra viz --extra collect
}

if [[ ! -d .venv ]] || [[ ! -x .venv/bin/teleop-birth && ! -x .venv/bin/python ]]; then
  echo "首次启动或环境不完整，正在 uv sync ..."
  _sync
fi

if [[ ! -x .venv/bin/teleop-birth ]]; then
  _sync
fi

# 缺 PyQt6 时补装（旧环境可能只有 --all-groups）
if ! .venv/bin/python -c "import PyQt6" >/dev/null 2>&1; then
  echo "检测到缺少 PyQt6（场景工坊依赖），正在补装 ..."
  _sync
fi

# 缺采集依赖时补装（自动数据采集写 MCAP）
if ! .venv/bin/python -c "import mcap; from mcap_protobuf.writer import Writer; from foxglove_schemas_protobuf.CompressedImage_pb2 import CompressedImage" >/dev/null 2>&1; then
  echo "检测到缺少采集依赖（mcap/foxglove），正在补装: uv sync --extra collect ..."
  uv sync --extra collect || true
fi

# Qt xcb 系统库（缺 libxcb-cursor0 会导致整个进程 abort）
if ! bash scripts/check_qt_system_deps.sh; then
  echo ""
  echo "可继续启动终端菜单，但「搭建场景」PyQt 工坊会失败，直到安装上述 apt 包。"
  echo "装好后重新 bash start.sh 即可。"
  echo ""
fi

# 中文字体（缺 CJK 时 PyQt 中文乱码）
if ! fc-list :lang=zh >/dev/null 2>&1 || [[ -z "$(fc-list :lang=zh 2>/dev/null | head -1)" ]]; then
  echo "提示: 系统未检测到中文字体。已依赖 mplfonts 内嵌字体；仍建议安装:"
  echo "  sudo apt install -y fonts-wqy-microhei fonts-noto-cjk"
fi

# Qt 中文输入（输入框内直接输出汉字）
bash scripts/setup_ibus_pinyin.sh || true
export QT_IM_MODULE=ibus
export XMODIFIERS=@im=ibus
export GTK_IM_MODULE=ibus
unset IBUS_USE_PORTAL || true
if [[ -z "${LANG:-}" || "${LANG}" == "C" || "${LANG}" == "POSIX" || "${LANG}" != zh_CN* ]]; then
  if locale -a 2>/dev/null | grep -qiE '^zh_CN\.(utf8|UTF-8)$'; then
    export LANG=zh_CN.UTF-8
  else
    export LANG=C.UTF-8
  fi
fi
export LC_ALL="$LANG"
export LC_CTYPE="$LANG"
if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]] && command -v dbus-launch >/dev/null 2>&1; then
  eval "$(dbus-launch --sh-syntax)"
fi
if command -v ibus >/dev/null 2>&1; then
  _ibus_addr="$(ibus address 2>/dev/null || true)"
  [[ -n "${_ibus_addr}" ]] && export IBUS_ADDRESS="${_ibus_addr}"
fi
echo "中文输入: 工坊内点绿色中文词条，或「启用中文输入法」后在框内直接输入汉字"

# 仿真用 menagerie（缺失则浅克隆；体积大，不进 git）
bash scripts/bootstrap_third_party.sh

exec uv run teleop-birth "$@"
