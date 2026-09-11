#!/usr/bin/env bash
# 关闭 / 放宽 Pico 自动休眠（需 USB 或无线 adb 已连接）
# 企业版完整开关在头显「企业设置」里；本脚本尽量用 adb 辅助保活
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ADB="$ROOT/tools/bin/adb"
if [[ ! -x "$ADB" ]]; then
  echo "未找到 $ADB，先跑: bash tools/scripts/install_xrobo.sh"
  exit 1
fi

export PATH="$(dirname "$ADB"):$PATH"

echo "当前 adb 设备:"
"$ADB" devices
if ! "$ADB" get-state 2>/dev/null | grep -q device; then
  echo
  echo "没有已授权的 Pico。请："
  echo "  1. 设置 → 通用 → 关于本机 → 连点软件版本 10 次 → 开开发者"
  echo "  2. USB 连电脑并允许调试"
  echo "  3. 再执行本脚本"
  exit 1
fi

echo
echo "========== adb 保活 =========="
"$ADB" shell settings put global stay_on_while_plugged_in 7
"$ADB" shell settings put system screen_off_timeout 2147483647

# 企业属性（部分固件可写；失败可忽略，以头显企业设置为准）
set_prop() {
  local key="$1" val="$2"
  if "$ADB" shell "setprop $key $val" 2>/dev/null; then
    echo "  setprop $key=$val  OK"
  else
    echo "  setprop $key=$val  (无权限/不支持，请用企业设置手动改)"
  fi
}
echo "尝试写入 Pico 休眠相关属性…"
set_prop persist.pvr.sleep_by_static 0          # 自动休眠关
set_prop persist.psensor.screenoff.delay -1     # 灭屏永不
set_prop persist.psensor.sleep.delay -1         # 系统休眠永不
set_prop persist.psensor.sleepmode 3            # 摘下休眠相关（3=关，因固件而异）

echo
echo "当前 Android 设置:"
echo -n "  stay_on_while_plugged_in = "
"$ADB" shell settings get global stay_on_while_plugged_in
echo -n "  screen_off_timeout(ms)   = "
"$ADB" shell settings get system screen_off_timeout

echo
echo "========== 头显里还要手动确认（企业设置）=========="
echo "  设置 → 通用 → 关于本机 → 连点「软件版本」10 次"
echo "  → 开发者选项 → 企业设置 / Business Settings"
echo "  1) 自动休眠模式 = 关"
echo "  2) 灭屏后保持网络连接 = 开"
echo "  3) 系统设置 → 电源策略：系统休眠=永不，再设灭屏=永不"
echo "  4) 充电时亮屏 = 开（若有）"
echo
echo "遥操时建议插着 USB；XRoboToolkit 前台 + Send 开。"
