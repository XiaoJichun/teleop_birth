# 场景物品 / 布局 / 语义采集

对齐智元 [Genie Sim](https://agibot-world.com/genie-sim) 的 asset → layout → autocollect；本仓库用 MuJoCo 双臂实现。

## 配置（优先改 YAML）

| 文件 | 作用 |
|------|------|
| [`configs/scene/cli.yaml`](../../../configs/scene/cli.yaml) | 生成 / 摆放 / 采集的**交互默认** |
| [`configs/scene/props/*.yaml`](../../../configs/scene/props/) | 物品库 |
| [`configs/scene/tasks/*.yaml`](../../../configs/scene/tasks/) | 语义任务 stages |
| [`configs/ui/menu.yaml`](../../../configs/ui/menu.yaml) | 主菜单项 |
| [`configs/ui/labels.yaml`](../../../configs/ui/labels.yaml) | 物品/任务显示名 |

## 交互式终端（一键，推荐）

```bash
./start.sh
```

选菜单即可，**不必再敲采集命令**：

| 菜单 | 读配置 |
|------|--------|
| **一键：自动化采数据（红方块→蓝盘子 → MCAP）** | `configs/scene/cli.yaml` → `collect` |
| 一键：生成场景物品 | → `generate` |
| 一键：随机摆放物品 | → `place` |

改集数 / 种子 / 是否可视化：只改 `configs/scene/cli.yaml`，再回菜单点一键。

「自定义：…」会逐步提问；日常采数据用一键即可。

## Foxglove 打开 MCAP

新采集使用 **foxglove protobuf**（旧 JSON 版请删掉重采）：

| Topic | Schema |
|-------|--------|
| `/cam/front/compressed` | `foxglove.CompressedImage` |
| `/tf` | `foxglove.FrameTransforms` |
| `/robot/ee/*`、`/props/*` | `foxglove.PoseInFrame` |
| `/task/log` | `foxglove.Log` |

Foxglove Studio → Open local file → 选 `logs/mcap/*.mcap` → 加 **Image** 面板绑 `/cam/front/compressed`，**3D** 面板绑 `/tf`。

相机：`configs/scene/cli.yaml` → `collect.camera`（`enabled` / `width` / `height` / `every_n_steps`）。
