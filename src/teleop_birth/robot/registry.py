"""机器人 embodiment 注册表：臂 / 轮式人形 / 双足，即插即用。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RobotSpec:
    """一台可调用机器人的登记信息。"""

    id: str
    title: str
    category: str  # arm | wheeled_humanoid | biped
    vendor: str
    status: str  # ready | stub | experimental
    module: str  # import path of factory
    factory: str  # callable name: build_backend / connect helper
    assets: str = ""  # relative asset root under assets/robots or menagerie
    notes: str = ""
    tags: tuple[str, ...] = ()
    # 可搭配的夹爪/灵巧手 id（catalog grippers.*）；空=当前不可选任何末端
    compatible_grippers: tuple[str, ...] = ()
    # 场景搭建 MuJoCo 预览是否可用（有真实编译路径）
    scene_preview: bool = False


_REGISTRY: dict[str, RobotSpec] = {}


def register_robot(spec: RobotSpec) -> RobotSpec:
    _REGISTRY[spec.id] = spec
    return spec


def get_robot(robot_id: str) -> RobotSpec:
    if robot_id not in _REGISTRY:
        raise KeyError(f"未知机器人: {robot_id}；已注册: {sorted(_REGISTRY)}")
    return _REGISTRY[robot_id]


def list_robots(
    *,
    category: str | None = None,
    ready_only: bool = False,
    preview_only: bool = False,
) -> list[RobotSpec]:
    items = list(_REGISTRY.values())
    if category:
        items = [r for r in items if r.category == category]
    if ready_only:
        items = [r for r in items if r.status == "ready"]
    if preview_only:
        items = [r for r in items if r.scene_preview]
    return sorted(items, key=lambda r: (r.category, r.id))


def grippers_compatible_with(robot_id: str) -> tuple[str, ...]:
    """返回该机器人允许的夹爪 id；未知机器人 → 空。"""
    try:
        return get_robot(robot_id).compatible_grippers
    except KeyError:
        return ()


def load_backend(robot_id: str, **kwargs: Any):
    """按登记信息 import 并构造后端（stub 也可实例化）。"""
    import importlib

    spec = get_robot(robot_id)
    mod = importlib.import_module(spec.module)
    fn = getattr(mod, spec.factory)
    return fn(**kwargs)
