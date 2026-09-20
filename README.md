# Progressive Extremity Teleop (PET) / teleop_birth

Pico 4 Ultra Enterprise 遥操框架：**双臂先落地，同一协议扩到人形全身**。  
环境用 **uv**，参数全部走 **YAML / Hydra**，入口为仿 `data_process` 的交互式终端。

包路径：`src/teleop_birth/`，顶层四类：`teleop/`（闭环）、`robot/`（机型）、`scene/`（场景与插件）、`ui/`（菜单）。  
**架构与算法总览**：[doc/架构与算法流程总览.md](doc/架构与算法流程总览.md)。

场景物品 / 语义采集：`./start.sh` → **场景工坊** 或快捷「自动化采数据」（读 `configs/scene/cli.yaml`）。  
场景清单：[`configs/scenes/`](configs/scenes/)；插件：[`src/teleop_birth/scene/plugins/`](src/teleop_birth/scene/plugins/)。

## 依赖与 third_party

**不要**把 `third_party/` 里的 deb/APK/运行时/menagerie 推到 GitHub（约 1GB+）。已写入 `.gitignore`。

```bash
uv sync --all-groups
bash scripts/bootstrap_third_party.sh   # mujoco_menagerie + tacto
bash scripts/xrobo/install_xrobo.sh     # XRobo/adb/APK（或 ./start_pico_sim.sh）
# 可选：
uv sync --extra ik
uv sync --extra tacto --extra collect
```

```bash
./start.sh   # 场景工坊 / Pico 遥操 / 自动采集
# 或
./start_pico_sim.sh
```

## 配置目录（算法 vs 设备 vs 场景插件）

```
configs/
  config.yaml              # Hydra 入口
  algorithm/               # runtime / mapper / ik
  device/                  # source / backend / end_effector / paths
  scenes/                  # 声明式场景清单（工坊）
  plugins/                 # 插件默认
  setup/                   # Pico 安装与一键 overrides
  ui/                      # 菜单
scripts/                   # bootstrap / xrobo（原 tools/ 已迁入）
src/teleop_birth/          # 四类：ui / scene / robot / teleop
```

换设备只动 **`configs/device/`**。根目录有指向上述目录的 symlink，因此命令行仍可用短写法 `source=xrobo`、`ik=ssik`。详见 [configs/README.md](configs/README.md)。

## 非交互运行

```bash
uv run python run_teleop.py \
  source=mock backend=mujoco_dual_ur ik=dls mapper=dual_arm end_effector=gripper \
  runtime.duration_s=3 runtime.render=false runtime.interactive_confirm=false
```

切换 IK：`ik=ssik` 或 `ik=curobo`（见 [doc/IK求解器切换.md](doc/IK求解器切换.md)）。

## 文档

- [doc/模块与接口.md](doc/模块与接口.md) — 包结构、仿真/真机双端口
- [doc/IK求解器切换.md](doc/IK求解器切换.md) — DLS / SSIK / cuRobo
- [doc/遥操作算法深入解析.md](doc/遥操作算法深入解析.md)
- [doc/遥操作全链路详解.md](doc/遥操作全链路详解.md)
- [doc/架构与创新点_ProgressiveExtremityTeleop.md](doc/架构与创新点_ProgressiveExtremityTeleop.md)
- [doc/Pico连接指南.md](doc/Pico连接指南.md)
- [doc/pico_teleop_方案调研与选型.md](doc/pico_teleop_方案调研与选型.md)

## License

MIT
