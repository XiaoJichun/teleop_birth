# Progressive Extremity Teleop (PET) / birth_teleop

Pico 4 Ultra Enterprise 遥操框架：**双臂先落地，同一协议扩到人形全身**。  
环境用 **uv**，参数全部走 **YAML / Hydra**，入口为仿 `data_process` 的交互式终端。

## 依赖与 third_party

**不要**把 `third_party/` 里的 deb/APK/运行时/menagerie 推到 GitHub（约 1GB+）。已写入 `.gitignore`。

新机器：

```bash
uv sync --all-groups
bash tools/scripts/bootstrap_third_party.sh   # 自动 clone mujoco_menagerie
bash tools/scripts/install_xrobo.sh           # 自动下 XRobo/adb/APK（或直接 ./start_pico_sim.sh）
```

`./start.sh` / `./start_pico_sim.sh` 也会在缺失时自动拉 menagerie / 装 XRobo。

```bash
cd /root/workspace/teleop_birth
./start_pico_sim.sh
```

首次自动解压 PC Service（**无需 sudo**）并编译 SDK。终端会打印本机 IP；Pico 打开 **XRoboToolkit** → 选 IP → WORKING + Send → 握 Grip。

头显首次需安装 APK（非自带应用）：见 [doc/Pico连接指南.md](doc/Pico连接指南.md)「首次：安装 XRoboToolkit APK」。

交互菜单：

```bash
./start.sh
```

首选「一键：Pico 仿真遥操」。详见 [doc/Pico连接指南.md](doc/Pico连接指南.md)。

## 非交互运行（Hydra）

```bash
uv sync --all-groups
uv run python run_teleop.py \
  source=mock backend=mujoco_dual_ur mapper=dual_arm end_effector=gripper \
  runtime.duration_s=3 runtime.render=false runtime.interactive_confirm=false
```

## 配置目录

```
configs/
  config.yaml              # Hydra 主配置 + runtime/paths
  menu.yaml                # 交互主菜单（无硬编码）
  interactive_labels.yaml  # 菜单显示名
  source/                  # mock | xrobo
  backend/                 # mock_dual_arm | mujoco_dual_ur | g1_stub
  mapper/                  # dual_arm | humanoid
  end_effector/            # gripper | dex_hand
```

**约定：数值与选项只写在 YAML；Python 只读配置组装对象。**

## 当前硬件决策

| 项 | 选择 |
|----|------|
| 双臂 | UR 双臂 MuJoCo（`backend=mujoco_dual_ur`） |
| 末端 | 夹爪优先（`end_effector=gripper`），可切 `dex_hand` |
| 人形 | G1（`backend=g1_stub` 预留） |

## 文档

- [doc/架构与创新点_ProgressiveExtremityTeleop.md](doc/架构与创新点_ProgressiveExtremityTeleop.md)
- [doc/pico_teleop_方案调研与选型.md](doc/pico_teleop_方案调研与选型.md)

## License

MIT
