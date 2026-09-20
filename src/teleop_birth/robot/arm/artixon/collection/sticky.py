"""粘性抓取：物体钉在左右指垫中点（实测指缝），避免吸进/穿模。"""

from __future__ import annotations

from typing import Optional

import mujoco
import numpy as np

from teleop_birth.scene.plugins.physics import AntiPenetrationManager
from teleop_birth.robot.arm.artixon.collection.grasp_geometry import DEFAULT_PAD_OFFSET_XYZ
from teleop_birth.robot.arm.artixon.scene.props import PropPoseController


class StickyGrasp:
    def __init__(
        self,
        props: PropPoseController,
        anti: AntiPenetrationManager,
        *,
        object_name: str,
        model: mujoco.MjModel | None = None,
        data: mujoco.MjData | None = None,
        grasp_offset_xyz: tuple[float, float, float] = DEFAULT_PAD_OFFSET_XYZ,
        attach_radius_m: float = 0.08,
        attach_gripper_min: float = 0.30,
    ) -> None:
        self.props = props
        self.anti = anti
        self.object_name = object_name
        self.model = model
        self.data = data
        self.grasp_offset = np.asarray(grasp_offset_xyz, dtype=np.float64).reshape(3)
        self.attach_radius_m = float(attach_radius_m)
        self.attach_gripper_min = float(attach_gripper_min)
        self._held_quat: Optional[np.ndarray] = None
        self.holding_side: Optional[str] = None
        self._finger_bids: dict[str, tuple[int, int]] = {}
        if model is not None:
            for side, pref in (("left", "L_"), ("right", "R_")):
                bl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{pref}arm_gripper_left_link")
                br = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{pref}arm_gripper_right_link")
                if bl >= 0 and br >= 0:
                    self._finger_bids[side] = (bl, br)

    @property
    def attached(self) -> bool:
        return self._held_quat is not None

    def pad_center(self, side: str, ee_pose: np.ndarray) -> np.ndarray:
        """左右指连杆中点 = 视觉上的夹持中心。"""
        if self.data is not None and side in self._finger_bids:
            bl, br = self._finger_bids[side]
            return 0.5 * (self.data.xpos[bl] + self.data.xpos[br])
        # fallback：TCP + 固定偏移
        from teleop_birth.robot.arm.artixon.ik_dls import pose_to_matrix

        T = pose_to_matrix(ee_pose)
        return T[:3, 3] + T[:3, :3] @ self.grasp_offset

    def update(self, side: str, ee_pose: np.ndarray, gripper: float) -> None:
        ee = np.asarray(ee_pose, dtype=np.float64)
        obj = self.props.get_pose(self.object_name)
        ideal = self.pad_center(side, ee)
        if gripper >= self.attach_gripper_min:
            if self._held_quat is None:
                near = float(np.linalg.norm(ideal - obj[:3])) <= self.attach_radius_m
                xy_ok = float(np.linalg.norm(ideal[:2] - obj[:2])) <= self.attach_radius_m
                if near and xy_ok:
                    self._held_quat = obj[3:].copy()
                    self.holding_side = side
                    self.anti.on_grasp(self.object_name)
            if self._held_quat is not None and self.holding_side == side:
                self.props.set_pose(self.object_name, pos=ideal, quat_xyzw=self._held_quat)
            return

        if self._held_quat is not None and self.holding_side == side:
            self.anti.on_release(self.object_name)
            self._held_quat = None
            self.holding_side = None

    def force_release(self) -> None:
        if self._held_quat is not None:
            self.anti.on_release(self.object_name)
        self._held_quat = None
        self.holding_side = None
