"""CommsPlugin：把 ROS 数据通讯配置挂到 PluginContext。"""

from __future__ import annotations

from typing import Any

from teleop_birth.scene.plugins.base import Plugin, PluginContext, MountSpec, register
from teleop_birth.scene.plugins.comms.bridge import CommsConfig, load_comms_config, RosDataComms


@register
class CommsPlugin(Plugin):
    """数据通讯插件（ROS pub/sub）。场景 YAML::

        comms:
          enabled: true
          publish:
            state: { mode: ee }       # 或 joint
            cameras: { names: [third_person, wrist_right] }
          subscribe:
            mode: ee                  # 或 joint
    """

    id = "comms"
    title = "ROS 数据通讯"

    def validate(self, ctx: PluginContext) -> list[str]:
        raw = ctx.scene_cfg.get("comms")
        if raw is None:
            return []
        if isinstance(raw, dict) and raw.get("enabled") is False:
            return []
        cfg = self._resolve_cfg(ctx)
        errs: list[str] = []
        if cfg.state_mode not in ("joint", "ee"):
            errs.append(f"comms.publish.state.mode 无效: {cfg.state_mode}")
        if cfg.action_mode not in ("joint", "ee"):
            errs.append(f"comms.subscribe.mode 无效: {cfg.action_mode}")
        return errs

    def attach(self, ctx: PluginContext, mount: MountSpec | None = None) -> None:
        cfg = self._resolve_cfg(ctx)
        ctx.extras["comms_cfg"] = cfg
        ctx.name_map["comms_state_mode"] = cfg.state_mode
        ctx.name_map["comms_action_mode"] = cfg.action_mode
        ctx.name_map["comms_transport"] = cfg.transport

    def build(self, ctx: PluginContext, *, overrides: dict[str, Any] | None = None) -> RosDataComms:
        """构造可 start 的通讯实例。"""
        cfg = self._resolve_cfg(ctx, overrides=overrides)
        return RosDataComms(cfg)

    def _resolve_cfg(
        self, ctx: PluginContext, *, overrides: dict[str, Any] | None = None
    ) -> CommsConfig:
        scene_comms = ctx.scene_cfg.get("comms")
        over: dict[str, Any] = {}
        rel = "plugins/comms/ros_bridge.yaml"
        if isinstance(scene_comms, str):
            rel = scene_comms
        elif isinstance(scene_comms, dict):
            over = dict(scene_comms)
        if overrides:
            over = {**over, **overrides}
        return load_comms_config(ctx.project_root, rel=rel, overrides=over or None)
