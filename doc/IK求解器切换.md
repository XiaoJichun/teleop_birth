# IK 求解器切换

| `ik=` | 含义 | 依赖 |
|-------|------|------|
| `dls`（默认） | MuJoCo site 雅可比 DLS | 无 |
| `ssik` | [SSIK](https://github.com/personalrobotics/ssik) | `uv sync --extra ik` |
| `curobo` | [cuRobo](https://curobo.org/) | 官网 CUDA 安装 |

配置：`configs/algorithm/ik/{dls,ssik,curobo}.yaml`（与 `backend` 场景解耦）。

```bash
# CLI
./start.sh → 自定义遥操 → 选「IK 求解器」

# Hydra
uv run python run_teleop.py ik=ssik ...
```

失败时 `fallback: dls`。TCP↔flange 换算见代码 `src/teleop_birth/ik/`。
