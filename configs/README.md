# configs 说明

本目录由 **Hydra** 组合，交互菜单由 `menu.yaml` 驱动。

| 文件/目录 | 作用 |
|-----------|------|
| `config.yaml` | 主配置：`defaults` + `runtime` + `paths` |
| `menu.yaml` | `./start.sh` 主菜单项与 handler |
| `interactive_labels.yaml` | 交互选项显示名 |
| `source/` | XR 源（mock / xrobo） |
| `backend/` | 机器人后端（mock / mujoco_dual_ur / g1_stub） |
| `mapper/` | 映射器（dual_arm / humanoid） |
| `end_effector/` | 末端（gripper / dex_hand） |

新增选项：在对应目录加 yaml，并在 `interactive_labels.yaml` 写上显示名即可，无需改 Python 菜单代码。
