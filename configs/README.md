# configs 分层说明

完整「改什么改哪个文件」对照表见 **[doc/配置文件指南.md](../doc/配置文件指南.md)**。

```
configs/
  config.yaml                 # Hydra 入口（遥操）
  algorithm/                  # ★ 常驻算法（换设备基本不改）
    runtime/default.yaml      # hz / scale / 日志
    mapper/                   # 相对遥操、clutch、姿态模式
    ik/                       # dls | ssik | curobo
  device/                     # ★ 换设备时常改
    source/                   # mock | xrobo
    backend/                  # mujoco_* / g1_* / mock
    end_effector/             # 夹爪 / 灵巧手
    paths/                    # menagerie、logs
  scene/                      # ★ 场景采集（物品 / 任务 / 录制）
    cli.yaml                  # 生成/摆放/采集默认
    recording.yaml            # MCAP topic + 第三人称/双腕相机
    props/                    # 道具
    tasks/                    # 语义任务
  setup/                      # Pico/XRobo 安装
  ui/                         # 终端菜单

  # 根目录 symlink → 上述目录，只为 Hydra 短 override：
  #   ik=ssik  source=xrobo  backend=mujoco_dual_ur …
```

## 换设备时改哪里？

| 场景 | 改 `device/` 下 |
|------|----------------|
| 换头显/传感 | `source/`（mock ↔ xrobo） |
| 换仿真或接真机 | `backend/` |
| 换末端 | `end_effector/` |
| 换资产路径 | `paths/` |

## 场景采集改哪里？

| 场景 | 文件 |
|------|------|
| 开窗口、集数、流畅度 | `scene/cli.yaml` → `collect` |
| 相机机位 / Foxglove 话题 | `scene/recording.yaml` |
| 任务指令与物体 | `scene/tasks/*.yaml` |
| 方块/盘子外观 | `scene/props/*.yaml` |

## Hydra 示例

```bash
uv run python run_teleop.py source=xrobo backend=mujoco_dual_ur
uv run python run_teleop.py ik=ssik
```
