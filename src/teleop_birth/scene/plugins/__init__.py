"""插件层：组装场景附件（不依赖 cli；类型来自 assets.types）。

含：scene / camera / gripper / sensor / physics / collect / frame / comms
（frame = 臂坐标映射 / R_a / 桌面 base；comms = ROS 观测/动作桥）
"""

from teleop_birth.scene.plugins.base import (
    AssetRef,
    MountSpec,
    Plugin,
    PluginContext,
    get_plugin,
    list_plugins,
    register,
)

# 确保插件类注册
import teleop_birth.scene.plugins.comms  # noqa: F401

__all__ = [
    "AssetRef",
    "MountSpec",
    "Plugin",
    "PluginContext",
    "get_plugin",
    "list_plugins",
    "register",
]
