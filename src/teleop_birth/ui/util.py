"""YAML 工具与动态 handler 加载。"""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Callable

import yaml


def project_root() -> Path:
    # src/teleop_birth/ui/util.py → repo root
    return Path(__file__).resolve().parents[3]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data if isinstance(data, dict) else {}


def resolve_handler(spec: str | None) -> Callable[..., Any] | None:
    if not spec:
        return None
    module_name, _, attr = spec.partition(":")
    if not module_name or not attr:
        raise ValueError(f"非法 handler 规格: {spec!r}（应为 module:attr）")
    mod = importlib.import_module(module_name)
    fn = getattr(mod, attr, None)
    if fn is None or not callable(fn):
        raise AttributeError(f"handler 不可调用: {spec}")
    return fn


def list_config_group(group_dir: Path) -> list[tuple[str, str]]:
    if not group_dir.is_dir():
        return []
    return [(p.stem, p.stem) for p in sorted(group_dir.glob("*.yaml"))]


def hydra_default_choice(config_root: Path, group: str) -> str | None:
    """从 config.yaml defaults 解析 group 的默认 stem（支持 algorithm/x@group 写法）。"""
    root = load_yaml(config_root / "config.yaml")
    for item in root.get("defaults") or []:
        if not isinstance(item, dict):
            continue
        for key, value in item.items():
            if value in (None, "_self_"):
                continue
            # key 可能是 "algorithm/ik@ik" 或 "ik"
            if key == group or key.endswith(f"@{group}"):
                return str(value)
    return None
