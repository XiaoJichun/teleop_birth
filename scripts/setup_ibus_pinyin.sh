#!/usr/bin/env bash
# 确保系统中文输入法可在 PyQt 输入框内直接输出汉字
set -euo pipefail

export DISPLAY="${DISPLAY:-:0}"

# 空 LANG/POSIX，或非中文 locale → 强制 zh_CN，否则 Qt 中文输入法不稳定
if [[ -z "${LANG:-}" || "${LANG}" == "C" || "${LANG}" == "POSIX" || "${LANG}" != zh_CN* ]]; then
  if locale -a 2>/dev/null | grep -qiE '^zh_CN\.(utf8|UTF-8)$'; then
    export LANG=zh_CN.UTF-8
  else
    export LANG=C.UTF-8
  fi
fi
export LC_ALL="$LANG"
export LC_CTYPE="$LANG"
# 切勿 export IBUS_USE_PORTAL（即便=0）：Qt 见变量名就走 portal → invalid portal bus
unset IBUS_USE_PORTAL || true

# Qt ibus 插件需要 session dbus（docker/SSH 终端常缺失）
if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]] && command -v dbus-launch >/dev/null 2>&1; then
  # shellcheck disable=SC2046
  eval "$(dbus-launch --sh-syntax)"
fi

export QT_IM_MODULE=ibus
export XMODIFIERS=@im=ibus
export GTK_IM_MODULE=ibus
unset IBUS_USE_PORTAL || true

if ! command -v ibus-daemon >/dev/null 2>&1; then
  echo "未安装 ibus。请执行: apt install -y ibus ibus-libpinyin"
  exit 1
fi

if ! pgrep -x ibus-daemon >/dev/null 2>&1; then
  ibus-daemon -drx >/dev/null 2>&1 || true
  sleep 0.5
fi

if command -v ibus >/dev/null 2>&1; then
  ibus write-cache >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5; do
    if ibus engine libpinyin >/dev/null 2>&1; then
      break
    fi
    sleep 0.3
  done
  IBUS_ADDR="$(ibus address 2>/dev/null || true)"
  if [[ -n "${IBUS_ADDR}" ]]; then
    export IBUS_ADDRESS="${IBUS_ADDR}"
  fi
  cur="$(ibus engine 2>/dev/null || true)"
  echo "中文输入法: 引擎=${cur:-unknown}  locale=${LANG}  dbus=${DBUS_SESSION_BUS_ADDRESS:+ok}"
fi
