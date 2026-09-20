"""遥操会话计划：按轮次重置物品（读 TELEOP_PLAN_JSON）。

按键：左手 X 或 右手 A → 下一轮布局（上升沿）。
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Optional

from teleop_birth.scene.placement.pose_controller import PropPoseController
from teleop_birth.scene.placement.randomizer import LayoutRandomizer
from teleop_birth.ui.session.round_layouts import load_plan, resolve_prop_specs


def load_plan_from_env() -> dict[str, Any] | None:
    raw = os.environ.get("TELEOP_PLAN_JSON") or ""
    if not raw:
        return None
    p = Path(raw)
    if not p.is_file():
        print(f"[teleop-plan] TELEOP_PLAN_JSON 不存在: {p}")
        return None
    return load_plan(p)


def inject_plan_props_into_scene(
    scene_cfg: dict[str, Any],
    plan: dict[str, Any],
    project_root: Path,
) -> dict[str, Any]:
    """在 build 前把计划中的物品写入 scene.props（dict 规格列表）。"""
    names = list(plan.get("props") or [])
    if not names:
        return scene_cfg
    out = dict(scene_cfg)
    specs = resolve_prop_specs(project_root, names)
    out["props"] = [s.to_dict() for s in specs]
    out["project_root"] = str(project_root)
    return out


def unwrap_mujoco_backend(backend) -> Any:
    """EndpointBackendAdapter → BackendAsEndpoint → MujocoDualURBackend。"""
    cur = backend
    # EndpointBackendAdapter
    if hasattr(cur, "endpoint"):
        cur = cur.endpoint
    # BackendAsEndpoint
    if hasattr(cur, "_backend"):
        cur = cur._backend
    return cur


class TeleopRoundManager:
    """连接后应用第 0 轮；X/A 上升沿切下一轮。"""

    def __init__(self, plan: dict[str, Any], project_root: Path) -> None:
        self.plan = plan
        self.project_root = Path(project_root)
        self.layouts: list[dict] = list(plan.get("layouts") or [])
        self.prop_names = list(plan.get("props") or [])
        self.episode = 0
        self.n_episodes = int(plan.get("episodes") or max(1, len(self.layouts)))
        self._ctrl: Optional[PropPoseController] = None
        self._specs = resolve_prop_specs(self.project_root, self.prop_names) if self.prop_names else []
        self._prev_x = False
        self._prev_a = False
        self._ready = False

    def attach(self, backend) -> None:
        raw = unwrap_mujoco_backend(backend)
        # LogisticsTeleopBackend 自带 props
        if getattr(raw, "props", None) is not None:
            self._ctrl = raw.props
            if self.prop_names:
                try:
                    self._ctrl.discover(self.prop_names)
                except Exception as exc:  # noqa: BLE001
                    print(f"[teleop-plan] discover 失败: {exc}")
                    return
        elif getattr(raw, "model", None) is not None and getattr(raw, "data", None) is not None:
            self._ctrl = PropPoseController(raw.model, raw.data)
            try:
                self._ctrl.discover(self.prop_names)
            except Exception as exc:  # noqa: BLE001
                print(f"[teleop-plan] 场景中无对应物品 freejoint，跳过轮次重置: {exc}")
                return
        else:
            print("[teleop-plan] 后端无 MuJoCo model/data，跳过轮次重置")
            return
        self._ready = True
        self.apply_episode(0)
        print(
            f"[teleop-plan] 已加载 {self.n_episodes} 轮布局；"
            f"按 左手X 或 右手A 切换下一轮物品位置"
        )

    def apply_episode(self, ep: int) -> None:
        if not self._ready or self._ctrl is None:
            return
        ep = int(ep) % max(1, self.n_episodes)
        self.episode = ep
        if ep < len(self.layouts) and self.layouts[ep]:
            layout = self.layouts[ep]
        elif self.plan.get("region"):
            rnd = LayoutRandomizer.from_region_dict(self.plan["region"], seed=ep * 997)
            layout = rnd.sample_layout(self._specs)
        else:
            return
        self._ctrl.set_many(layout)
        print(f"[teleop-plan] → 第 {ep + 1}/{self.n_episodes} 轮物品已重置")

    def on_obs(self, obs) -> None:
        if not self._ready:
            return
        btns = obs.buttons
        x_now = bool(getattr(btns, "button_x", False))
        a_now = bool(getattr(btns, "button_a", False))
        edge = (x_now and not self._prev_x) or (a_now and not self._prev_a)
        self._prev_x, self._prev_a = x_now, a_now
        if edge:
            self.apply_episode(self.episode + 1)
