# configs/scene — 场景物品 / 布局 / 语义任务 / 交互默认

```
configs/scene/
  cli.yaml     # ★ 交互式终端与脚本共用默认（改这里）
  props/       # 物品库 YAML
  tasks/       # 语义采集任务（对齐 Genie Sim stages）
```

交互入口：`./start.sh` / `uv run teleop-birth` → 场景三项菜单。  
标签：`configs/ui/labels.yaml`（props / tasks）。  
菜单项：`configs/ui/menu.yaml`。
