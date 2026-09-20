# scripts/

安装与运维脚本（原 `tools/` 已废弃并迁入此处）。

| 路径 | 作用 |
|------|------|
| `bootstrap_third_party.sh` | 拉取 mujoco_menagerie + tacto |
| `xrobo/install_xrobo.sh` | 安装 PC Service / adb / APK / pybind |
| `xrobo/ensure_xrobo_service.sh` | 确保 PC Service 在跑 |
| `xrobo/pico_keep_awake.sh` | adb 保活 |
| `bin/adb` | platform-tools 入口（安装脚本生成） |
| `fetch_gripper.py` | 列出 catalog 中的真实夹爪资产 |

场景搭建 / 采集请用 `./start.sh` → **场景工坊**。
