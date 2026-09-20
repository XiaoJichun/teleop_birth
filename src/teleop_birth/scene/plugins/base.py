"""插件统一契约：场景 / 相机 / 夹爪 / 传感器 / 物理 / 采集。"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from teleop_birth.scene.assets.types import AssetRef, MountSpec

__all__ = [
    "AssetRef",
    "MountSpec",
    "Plugin",
    "PluginContext",
    "get_plugin",
    "list_plugins",
    "register",
]


@dataclass
class PluginContext:
    """组装时共享上下文。"""

    project_root: Path
    scene_cfg: dict[str, Any]
    name_map: dict[str, Any] = field(default_factory=dict)
    runtime_hooks: list[Callable[..., None]] = field(default_factory=list)
    extras: dict[str, Any] = field(default_factory=dict)


class Plugin(ABC):
    """即插即用插件基类。"""

    id: str = "plugin"
    title: str = "Plugin"

    def manifest(self) -> dict[str, Any]:
        return {"id": self.id, "title": self.title}

    def validate(self, ctx: PluginContext) -> list[str]:
        """返回缺失/错误提示（空=通过）。"""
        return []

    def fetch(self, ctx: PluginContext) -> list[AssetRef]:
        """解析/拉取资产路径。"""
        return []

    def attach(self, ctx: PluginContext, mount: MountSpec | None = None) -> None:
        """挂到场景（子类覆盖）。"""
        return None


# 全局注册表
_REGISTRY: dict[str, Plugin] = {}


def register(plugin: Plugin) -> Plugin:
    _REGISTRY[plugin.id] = plugin
    return plugin


def get_plugin(plugin_id: str) -> Plugin | None:
    return _REGISTRY.get(plugin_id)


def list_plugins() -> list[Plugin]:
    return list(_REGISTRY.values())
