# third_party（勿提交到 GitHub）

本目录下的大文件由安装脚本自动下载，**不要** push 到仓库。

| 目录 | 来源 | 谁下载 |
|------|------|--------|
| `mujoco_menagerie/` | DeepMind menagerie | `scripts/bootstrap_third_party.sh` |
| `tacto/` | [facebookresearch/tacto](https://github.com/facebookresearch/tacto) DIGIT 触觉仿真 | 同上 |
| `xrobo_downloads/` | PC Service deb + Pico APK | `scripts/xrobo/install_xrobo.sh` |
| `xrobo_runtime/` | deb 解压结果 | 同上 |
| `platform-tools/` | Google adb | 同上 |
| `XRoboToolkit-PC-Service-Pybind/` | GitHub clone + 编译 | 同上 |

新环境：

```bash
uv sync --all-groups
bash scripts/bootstrap_third_party.sh   # menagerie + tacto
uv sync --extra tacto --extra collect   # 触觉 / 采集
bash scripts/xrobo/install_xrobo.sh     # Pico/XRobo（或 ./start_pico_sim.sh）
```

插件资产索引见 `src/teleop_birth/scene/assets/catalog.yaml`。
