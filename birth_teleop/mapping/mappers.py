"""XRObservation → SparseTarget mappers（参数由调用方/YAML 注入）。"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Sequence

import numpy as np

from birth_teleop.control import PoseLowPass, RelativeClutch
from birth_teleop.schema import (
    EXTREMITY_HEAD,
    EXTREMITY_LEFT_EE,
    EXTREMITY_LEFT_FOOT,
    EXTREMITY_PELVIS,
    EXTREMITY_RIGHT_EE,
    EXTREMITY_RIGHT_FOOT,
    ButtonState,
    SparseTarget,
    XRObservation,
    identity_pose,
)


class Mapper(ABC):
    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def map(self, obs: XRObservation, robot_feedback: Dict[str, np.ndarray]) -> SparseTarget: ...


def _button_down(buttons: ButtonState, name: str) -> bool:
    key = str(name).lower().strip()
    mapping = {
        "a": buttons.button_a,
        "button_a": buttons.button_a,
        "b": buttons.button_b,
        "button_b": buttons.button_b,
        "x": buttons.button_x,
        "button_x": buttons.button_x,
        "y": buttons.button_y,
        "button_y": buttons.button_y,
        "menu_left": buttons.left_menu,
        "left_menu": buttons.left_menu,
        "menu_right": buttons.right_menu,
        "right_menu": buttons.right_menu,
        "menu": buttons.left_menu or buttons.right_menu,
        "axis_left": buttons.left_axis_click,
        "left_axis_click": buttons.left_axis_click,
        "axis_right": buttons.right_axis_click,
        "right_axis_click": buttons.right_axis_click,
    }
    return bool(mapping.get(key, False))


def _any_button(buttons: ButtonState, names: Sequence[str]) -> bool:
    return any(_button_down(buttons, n) for n in names)


class DualArmMapper(Mapper):
    def __init__(
        self,
        scale: float,
        filter_alpha: float,
        map_head: bool,
        grip_threshold: float,
        clutch_mode: str = "long_press_grip",
        hold_s: float = 0.35,
        position_only: bool = True,
        max_step_m: float = 0.03,
        workspace_center: Optional[Dict[str, List[float]]] = None,
        workspace_radius_m: float = 0.55,
        # 左右手柄各自：复位键（上升沿触发该臂回零）
        reset_buttons_left: Optional[List[str]] = None,
        reset_buttons_right: Optional[List[str]] = None,
        # 兼容旧字段（忽略）
        reanchor_button: str = "b",
    ) -> None:
        self.scale = scale
        self.map_head = map_head
        self.clutch_mode = str(clutch_mode).lower()
        self.position_only = bool(position_only)
        self.max_step_m = float(max_step_m)
        self.workspace_radius_m = float(workspace_radius_m)
        self.reset_buttons_left = [str(x).lower() for x in (reset_buttons_left or ["y", "left_menu"])]
        self.reset_buttons_right = [str(x).lower() for x in (reset_buttons_right or ["b", "right_menu"])]
        self.workspace_center = {
            EXTREMITY_LEFT_EE: np.asarray(
                (workspace_center or {}).get(EXTREMITY_LEFT_EE, [-0.45, 0.35, 0.55]),
                dtype=np.float64,
            ),
            EXTREMITY_RIGHT_EE: np.asarray(
                (workspace_center or {}).get(EXTREMITY_RIGHT_EE, [0.45, 0.35, 0.55]),
                dtype=np.float64,
            ),
        }
        self.clutch = RelativeClutch(
            grip_threshold=grip_threshold,
            mode=self.clutch_mode,
            position_only=self.position_only,
            hold_s=hold_s,
        )
        self.lpf = PoseLowPass(alpha=filter_alpha)
        self._last: Dict[str, np.ndarray] = {
            EXTREMITY_LEFT_EE: identity_pose(),
            EXTREMITY_RIGHT_EE: identity_pose(),
            EXTREMITY_HEAD: identity_pose(),
        }
        self._prev_reset_l = False
        self._prev_reset_r = False

    def reset(self) -> None:
        for k in list(self._last):
            self.clutch.reset(k)
            self.lpf.reset(k)
        self._last = {
            EXTREMITY_LEFT_EE: identity_pose(),
            EXTREMITY_RIGHT_EE: identity_pose(),
            EXTREMITY_HEAD: identity_pose(),
        }
        self._prev_reset_l = False
        self._prev_reset_r = False

    def _limit_step(self, name: str, target: np.ndarray) -> np.ndarray:
        prev = self._last[name]
        out = target.copy()
        dp = out[:3] - prev[:3]
        n = float(np.linalg.norm(dp))
        if self.max_step_m > 0 and n > self.max_step_m:
            out[:3] = prev[:3] + dp * (self.max_step_m / n)
        return out

    def _clamp_workspace(self, name: str, target: np.ndarray) -> np.ndarray:
        if self.workspace_radius_m <= 0:
            return target
        center = self.workspace_center.get(name)
        if center is None:
            return target
        out = target.copy()
        d = out[:3] - center
        n = float(np.linalg.norm(d))
        if n > self.workspace_radius_m:
            out[:3] = center + d * (self.workspace_radius_m / max(n, 1e-12))
        return out

    def map(self, obs: XRObservation, robot_feedback: Dict[str, np.ndarray]) -> SparseTarget:
        reset_l = _any_button(obs.buttons, self.reset_buttons_left)
        reset_r = _any_button(obs.buttons, self.reset_buttons_right)
        home_left = bool(reset_l and not self._prev_reset_l)
        home_right = bool(reset_r and not self._prev_reset_r)
        self._prev_reset_l = reset_l
        self._prev_reset_r = reset_r

        if home_left:
            self.clutch.reset(EXTREMITY_LEFT_EE)
            self.lpf.reset(EXTREMITY_LEFT_EE)
        if home_right:
            self.clutch.reset(EXTREMITY_RIGHT_EE)
            self.lpf.reset(EXTREMITY_RIGHT_EE)

        extremities: Dict[str, np.ndarray] = {}
        active: Dict[str, bool] = {}

        sides = (
            (
                EXTREMITY_LEFT_EE,
                obs.left_controller,
                obs.buttons.left_grip,
                obs.buttons.left_trigger,
                home_left,
            ),
            (
                EXTREMITY_RIGHT_EE,
                obs.right_controller,
                obs.buttons.right_grip,
                obs.buttons.right_trigger,
                home_right,
            ),
        )

        for name, xr_pose, grip, trigger, do_home in sides:
            robot_pose = robot_feedback.get(name, self._last[name])
            if np.allclose(self._last[name][:3], 0.0, atol=1e-9):
                self._last[name] = robot_pose.copy()

            if do_home:
                # 本帧不遥操，交给 backend 回零；反馈下一帧再锁
                active[name] = False
                extremities[name] = self._last[name].copy()
                continue

            xr_scaled = xr_pose.copy()
            xr_scaled[:3] *= self.scale
            eng, target = self.clutch.update(
                name, grip, xr_scaled, robot_pose, trigger=trigger
            )
            active[name] = eng
            if eng and target is not None:
                target = self._limit_step(name, target)
                target = self._clamp_workspace(name, target)
                target = self.lpf.filter(name, target)
                self._last[name] = target
                extremities[name] = target
            else:
                extremities[name] = self._last[name].copy()

        if self.map_head:
            head = self.lpf.filter(EXTREMITY_HEAD, obs.head)
            self._last[EXTREMITY_HEAD] = head
            extremities[EXTREMITY_HEAD] = head
            active[EXTREMITY_HEAD] = True

        return SparseTarget(
            timestamp_ns=obs.timestamp_ns,
            extremities=extremities,
            active=active,
            left_gripper=float(np.clip(obs.buttons.left_trigger, 0.0, 1.0)),
            right_gripper=float(np.clip(obs.buttons.right_trigger, 0.0, 1.0)),
            meta={
                "scale": float(self.scale),
                "clutch_mode": self.clutch_mode,
                "position_only": self.position_only,
                "home_left": home_left,
                "home_right": home_right,
                "xr_grip_l": float(obs.buttons.left_grip),
                "xr_grip_r": float(obs.buttons.right_grip),
            },
        )


class HumanoidMapper(Mapper):
    def __init__(
        self,
        dual_arm: DualArmMapper,
        body_joint_indices: Dict[str, int],
        tracker_left_keywords: List[str],
        tracker_right_keywords: List[str],
    ) -> None:
        self.dual_arm = dual_arm
        required = ("pelvis", "left_ankle", "right_ankle", "head", "left_wrist", "right_wrist")
        for key in required:
            if key not in body_joint_indices:
                raise KeyError(f"body_joint_indices 缺少: {key}")
        self.idx = {k: int(body_joint_indices[k]) for k in required}
        self.tracker_left_keywords = [k.lower() for k in tracker_left_keywords]
        self.tracker_right_keywords = [k.lower() for k in tracker_right_keywords]

    def reset(self) -> None:
        self.dual_arm.reset()

    def _match_side(self, serial: str, keywords: List[str]) -> bool:
        key = serial.lower()
        return any(token in key for token in keywords)

    def map(self, obs: XRObservation, robot_feedback: Dict[str, np.ndarray]) -> SparseTarget:
        target = self.dual_arm.map(obs, robot_feedback)
        max_idx = max(self.idx.values())

        if obs.body_joints is not None and obs.body_joints.shape[0] > max_idx:
            bj = obs.body_joints
            target.extremities[EXTREMITY_PELVIS] = bj[self.idx["pelvis"]].copy()
            target.extremities[EXTREMITY_LEFT_FOOT] = bj[self.idx["left_ankle"]].copy()
            target.extremities[EXTREMITY_RIGHT_FOOT] = bj[self.idx["right_ankle"]].copy()
            target.extremities[EXTREMITY_HEAD] = bj[self.idx["head"]].copy()
            target.extremities[EXTREMITY_LEFT_EE] = bj[self.idx["left_wrist"]].copy()
            target.extremities[EXTREMITY_RIGHT_EE] = bj[self.idx["right_wrist"]].copy()
            for k in (
                EXTREMITY_PELVIS,
                EXTREMITY_LEFT_FOOT,
                EXTREMITY_RIGHT_FOOT,
                EXTREMITY_HEAD,
                EXTREMITY_LEFT_EE,
                EXTREMITY_RIGHT_EE,
            ):
                target.active[k] = True

            target.keypoints = {
                "pelvis": bj[self.idx["pelvis"], :3].copy(),
                "left_ankle": bj[self.idx["left_ankle"], :3].copy(),
                "right_ankle": bj[self.idx["right_ankle"], :3].copy(),
                "head": bj[self.idx["head"], :3].copy(),
                "left_wrist": bj[self.idx["left_wrist"], :3].copy(),
                "right_wrist": bj[self.idx["right_wrist"], :3].copy(),
            }
            target.root_height = float(bj[self.idx["pelvis"], 2])

        for serial, pose in obs.trackers.items():
            if self._match_side(serial, self.tracker_left_keywords):
                target.extremities[EXTREMITY_LEFT_FOOT] = pose.copy()
                target.active[EXTREMITY_LEFT_FOOT] = True
            elif self._match_side(serial, self.tracker_right_keywords):
                target.extremities[EXTREMITY_RIGHT_FOOT] = pose.copy()
                target.active[EXTREMITY_RIGHT_FOOT] = True

        return target
