# third_party（勿提交到 GitHub）

本目录下的大文件由安装脚本自动下载，**不要** push 到仓库。

| 目录 | 来源 | 谁下载 |
|------|------|--------|
| `mujoco_menagerie/` | DeepMind menagerie | `tools/scripts/bootstrap_third_party.sh` |
| `xrobo_downloads/` | PC Service deb + Pico APK | `tools/scripts/install_xrobo.sh` |
| `xrobo_runtime/` | deb 解压结果 | 同上 |
| `platform-tools/` | Google adb | 同上 |
| `XRoboToolkit-PC-Service-Pybind/` | GitHub clone + 编译 | 同上 |

新环境：

```bash
uv sync --all-groups
bash tools/scripts/bootstrap_third_party.sh   # menagerie
bash tools/scripts/install_xrobo.sh           # Pico/XRobo（或直接 ./start_pico_sim.sh）
```
