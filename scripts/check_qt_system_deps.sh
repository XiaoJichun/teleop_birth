#!/usr/bin/env bash
# 检查 PyQt6 场景工坊所需的系统图形库（Ubuntu/Debian）
set -euo pipefail

# name|apt包
CHECKS=(
  "libxcb-cursor.so|libxcb-cursor0"
  "libEGL.so|libegl1"
  "libGL.so|libgl1"
  "libxkbcommon-x11.so|libxkbcommon-x11-0"
)

missing_pkgs=()
for item in "${CHECKS[@]}"; do
  so="${item%%|*}"
  pkg="${item##*|}"
  if ! ldconfig -p 2>/dev/null | grep -q "${so}"; then
    # ldconfig 无记录时再扫常见路径
    found=0
    for base in /usr/lib/x86_64-linux-gnu /usr/lib/aarch64-linux-gnu /lib/x86_64-linux-gnu /usr/lib; do
      if ls "${base}/${so}"* >/dev/null 2>&1; then
        found=1
        break
      fi
    done
    if [[ "$found" -eq 0 ]]; then
      missing_pkgs+=("$pkg")
    fi
  fi
done

if [[ ${#missing_pkgs[@]} -eq 0 ]]; then
  exit 0
fi

# 去重
mapfile -t missing_pkgs < <(printf '%s\n' "${missing_pkgs[@]}" | sort -u)

echo "检测到缺少 Qt/X11 系统库: ${missing_pkgs[*]}"
echo "场景工坊 PyQt 需要这些包。请执行："
echo "  sudo apt update && sudo apt install -y ${missing_pkgs[*]}"
if [[ "${TELEOP_AUTO_APT:-}" == "1" ]] && command -v apt-get >/dev/null 2>&1; then
  echo "TELEOP_AUTO_APT=1，尝试自动安装…"
  sudo apt-get install -y "${missing_pkgs[@]}"
  exit 0
fi
exit 1
