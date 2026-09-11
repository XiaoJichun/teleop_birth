"""YAML 工具与动态 handler 加载。"""

from __future__ import annotations

import importlib
from pathlib import Path
from typing import Any, Callable

import yaml


def project_root() -> Path:
    # birth_teleop/cli/util.py → repo root
    return Path(__file__).resolve().parents[2]


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return data if isinstance(data, dict) else {}


def resolve_handler(spec: str | None) -> Callable[..., Any] | None:
    """'pkg.mod:func' → callable；None → None。"""
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
    """扫描 group_dir/*.yaml → [(stem, stem), ...]。"""
    if not group_dir.is_dir():
        return []
    return [(p.stem, p.stem) for p in sorted(group_dir.glob("*.yaml"))]


def hydra_default_choice(config_root: Path, group: str) -> str | None:
    root = load_yaml(config_root / "config.yaml")
    for item in root.get("defaults") or []:
        if isinstance(item, dict) and group in item:
            value = item[group]
            if value not in (None, "_self_"):
                return str(value)
    return None
