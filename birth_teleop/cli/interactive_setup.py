"""交互式启动：逐项选择 Hydra 配置组，返回 overrides。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from birth_teleop.cli import tui
from birth_teleop.cli.util import hydra_default_choice, list_config_group, load_yaml


def _pick_default(options: list[tuple[str, str]], preferred: str | None) -> str | None:
    if not options:
        return None
    if preferred and any(value == preferred for value, _ in options):
        return preferred
    return options[0][0]


def _labeled_options(
    group_dir: Path,
    labels: dict[str, str],
) -> list[tuple[str, str]]:
    options: list[tuple[str, str]] = []
    for value, _ in list_config_group(group_dir):
        label = labels.get(value) or value
        options.append((value, f"{label}  [{value}]"))
    return options


def build_interactive_overrides(project_root: Path) -> list[str] | None:
    """询问 source/backend/mapper/end_effector/runtime，返回 Hydra overrides。"""
    config_root = project_root / "configs"
    root_cfg = load_yaml(config_root / "config.yaml")
    runtime = dict(root_cfg.get("runtime") or {})
    labels_all = load_yaml(config_root / "interactive_labels.yaml")

    tui.banner("遥操交互式配置")

    source = tui.select(
        "XR 数据源",
        _labeled_options(config_root / "source", labels_all.get("source") or {}),
        default=_pick_default(
            list_config_group(config_root / "source"),
            hydra_default_choice(config_root, "source"),
        ),
    )
    if source is None:
        return None

    backend = tui.select(
        "机器人后端",
        _labeled_options(config_root / "backend", labels_all.get("backend") or {}),
        default=_pick_default(
            list_config_group(config_root / "backend"),
            hydra_default_choice(config_root, "backend"),
        ),
    )
    if backend is None:
        return None

    mapper = tui.select(
        "映射器",
        _labeled_options(config_root / "mapper", labels_all.get("mapper") or {}),
        default=_pick_default(
            list_config_group(config_root / "mapper"),
            hydra_default_choice(config_root, "mapper"),
        ),
    )
    if mapper is None:
        return None

    ee = tui.select(
        "末端执行器",
        _labeled_options(config_root / "end_effector", labels_all.get("end_effector") or {}),
        default=_pick_default(
            list_config_group(config_root / "end_effector"),
            hydra_default_choice(config_root, "end_effector"),
        ),
    )
    if ee is None:
        return None

    hz = tui.text(
        "控制频率 hz",
        default=str(runtime.get("hz", 50.0)),
        validate=lambda s: True if _is_pos_float(s) else "请输入正数",
    )
    if hz is None:
        return None

    duration = tui.text(
        "运行时长 duration_s（<=0 直到 Ctrl+C）",
        default=str(runtime.get("duration_s", 8.0)),
        validate=lambda s: True if _is_float(s) else "请输入数字",
    )
    if duration is None:
        return None

    scale = tui.text(
        "XR 位移缩放 scale",
        default=str(runtime.get("scale", 1.0)),
        validate=lambda s: True if _is_pos_float(s) else "请输入正数",
    )
    if scale is None:
        return None

    render = tui.confirm(
        "开启 MuJoCo 可视化窗口？",
        default=bool(runtime.get("render", False)),
    )
    if render is None:
        return None

    log_enabled = tui.confirm(
        "记录 JSONL 会话日志？",
        default=bool(runtime.get("log_enabled", False)),
    )
    if log_enabled is None:
        return None

    overrides = [
        f"source={source}",
        f"backend={backend}",
        f"mapper={mapper}",
        f"end_effector={ee}",
        f"runtime.hz={hz.strip()}",
        f"runtime.duration_s={duration.strip()}",
        f"runtime.scale={scale.strip()}",
        f"runtime.render={str(render).lower()}",
        f"runtime.log_enabled={str(log_enabled).lower()}",
        "runtime.interactive_confirm=true",
    ]

    tui.section("配置摘要（将写入 Hydra overrides）")
    for item in overrides:
        print(f"  {item}")
    ok = tui.confirm("确认并启动遥操？", default=True)
    if not ok:
        return None
    return overrides


def _is_float(raw: str) -> bool:
    try:
        float((raw or "").strip())
        return True
    except ValueError:
        return False


def _is_pos_float(raw: str) -> bool:
    try:
        return float((raw or "").strip()) > 0
    except ValueError:
        return False
