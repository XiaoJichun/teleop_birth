"""采集抓取辅助：加强摩擦；可选接触后刚体锁定。

默认（enable_latch=False）：
  全程靠 MuJoCo 接触与摩擦，不改写物体位姿、不加外力。
  注意：当前 2F-85 仿真里纯摩擦抬起成功率很低。

可选（enable_latch=True）：
  须左右指垫都碰到物体且夹爪闭合后，才锁定「当前」相对位姿以便抬起；
  禁止远距离吸附；松爪后解除，自由落体。

采集若要稳定成功，请打开 sticky_grasp；若要观察纯物理，请关闭。
"""

from __future__ import annotations

from typing import Optional

import mujoco
import numpy as np

from teleop_birth.scene.collection.grasp_geometry import DEFAULT_PAD_OFFSET_XYZ
from teleop_birth.scene.placement.pose_controller import PropPoseController
from teleop_birth.teleop.schema import pose_to_matrix


class StickyGrasp:
    def __init__(
        self,
        props: PropPoseController,
        *,
        object_name: str,
        model: mujoco.MjModel | None = None,
        data: mujoco.MjData | None = None,
        attach_radius_m: float = 0.05,
        grasp_offset_xyz: list[float] | tuple[float, float, float] = DEFAULT_PAD_OFFSET_XYZ,
        attach_gripper_min: float = 0.50,
        place_object: str | None = None,
        place_radius_m: float = 0.14,
        place_z_offset: float = 0.028,
        table_top_z: float = 0.42,
        object_half_z: float = 0.025,
        arm_prefix: str = "R_",
        contact_frames_needed: int = 3,
        enable_latch: bool = False,
    ) -> None:
        self.props = props
        self.object_name = object_name
        self.model = model
        self.data = data
        self.attach_radius_m = float(attach_radius_m)
        self.grasp_offset = np.asarray(grasp_offset_xyz, dtype=np.float64).reshape(3)
        self.attach_gripper_min = float(attach_gripper_min)
        self.place_object = place_object
        self.place_radius_m = float(place_radius_m)
        self.place_z_offset = float(place_z_offset)
        self.table_top_z = float(table_top_z)
        self.object_half_z = float(object_half_z)
        self.arm_prefix = str(arm_prefix)
        self.contact_frames_needed = max(1, int(contact_frames_needed))
        self.enable_latch = bool(enable_latch)
        self._held_quat_xyzw: Optional[np.ndarray] = None
        self._attach_delta: Optional[np.ndarray] = None
        self.did_place = False
        self._contact_streak = 0
        self._pad_bids: list[int] = []
        self._pad_gids: list[int] = []
        self._obj_gids: set[int] = set()
        self._obj_bid = -1
        self._qadr = -1
        if model is not None:
            self._cache(model)
            self._boost_friction(model)

    def _cache(self, model: mujoco.MjModel) -> None:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.object_name)
        self._obj_bid = bid
        if bid >= 0:
            for gid in range(model.ngeom):
                if int(model.geom_bodyid[gid]) == bid:
                    self._obj_gids.add(gid)
            for j in range(model.njnt):
                if int(model.jnt_bodyid[j]) == bid and model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                    self._qadr = int(model.jnt_qposadr[j])
                    break
        pref = self.arm_prefix
        for name in (
            f"{pref}g_left_silicone_pad",
            f"{pref}g_right_silicone_pad",
            f"{pref}g_left_pad",
            f"{pref}g_right_pad",
            f"{pref}left_pad",
            f"{pref}right_pad",
        ):
            pbid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if pbid < 0 or pbid in self._pad_bids:
                continue
            self._pad_bids.append(pbid)
            for gid in range(model.ngeom):
                if int(model.geom_bodyid[gid]) == pbid:
                    self._pad_gids.append(gid)
        for bid in range(model.nbody):
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
            if not bname.startswith(pref) or "pad" not in bname.lower():
                continue
            if bid in self._pad_bids:
                continue
            self._pad_bids.append(bid)
            for gid in range(model.ngeom):
                if int(model.geom_bodyid[gid]) == bid:
                    self._pad_gids.append(gid)

    def _boost_friction(self, model: mujoco.MjModel) -> None:
        for gid in self._obj_gids:
            model.geom_friction[gid] = np.array([4.5, 0.25, 0.02], dtype=np.float64)
            if int(model.geom_condim[gid]) < 4:
                model.geom_condim[gid] = 4
        for gid in self._pad_gids:
            model.geom_friction[gid] = np.array([5.0, 0.3, 0.02], dtype=np.float64)
            if int(model.geom_condim[gid]) < 4:
                model.geom_condim[gid] = 4
        for gid in range(model.ngeom):
            if int(model.geom_bodyid[gid]) != 0:
                continue
            model.geom_friction[gid] = np.array([0.35, 0.005, 0.0001], dtype=np.float64)
        if hasattr(model.opt, "noslip_iterations") and int(model.opt.noslip_iterations) < 3:
            model.opt.noslip_iterations = 3

    def _zero_velocity(self) -> None:
        if self.model is None or self.data is None or self._qadr < 0:
            return
        for j in range(self.model.njnt):
            if int(self.model.jnt_qposadr[j]) == self._qadr:
                da = int(self.model.jnt_dofadr[j])
                self.data.qvel[da : da + 6] = 0.0
                break

    def _pad_contact_count(self) -> int:
        if self.model is None or self.data is None:
            return 0
        if not self._obj_gids or not self._pad_gids:
            return 0
        touched: set[int] = set()
        for i in range(int(self.data.ncon)):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            for g_pad, g_other in ((g1, g2), (g2, g1)):
                if g_other not in self._obj_gids or g_pad not in self._pad_gids:
                    continue
                touched.add(int(self.model.geom_bodyid[g_pad]))
        return sum(1 for b in self._pad_bids if b in touched)

    def _ready_to_latch(self, ideal: np.ndarray, obj: np.ndarray) -> bool:
        d = float(np.linalg.norm(ideal - obj[:3]))
        if d > self.attach_radius_m:
            return False
        return self._pad_contact_count() >= 2

    @property
    def attached(self) -> bool:
        return self._held_quat_xyzw is not None

    def pad_center(self, ee_pose: np.ndarray) -> np.ndarray:
        T_ee = pose_to_matrix(ee_pose)
        return T_ee[:3, 3] + T_ee[:3, :3] @ self.grasp_offset

    def update(self, ee_pose: np.ndarray, gripper_closed: float) -> None:
        ee = np.asarray(ee_pose, dtype=np.float64)
        obj = self.props.get_pose(self.object_name)
        ideal = self.pad_center(ee)

        if gripper_closed < self.attach_gripper_min:
            self._contact_streak = 0
            if self._held_quat_xyzw is not None:
                self._held_quat_xyzw = None
                self._attach_delta = None
                self._zero_velocity()
                if self.place_object:
                    plate = self.props.get_pose(self.place_object)
                    cube = self.props.get_pose(self.object_name)
                    if float(np.linalg.norm(cube[:2] - plate[:2])) <= self.place_radius_m:
                        self.did_place = True
            return

        if self._ready_to_latch(ideal, obj):
            self._contact_streak += 1
        else:
            self._contact_streak = 0

        if not self.enable_latch:
            return

        if self._held_quat_xyzw is None:
            if self._contact_streak >= self.contact_frames_needed:
                self._held_quat_xyzw = obj[3:].copy()
                self._attach_delta = (obj[:3] - ideal).copy()
                self._zero_velocity()

        if self._held_quat_xyzw is not None and self._attach_delta is not None:
            pos = ideal + self._attach_delta
            z_floor = self.table_top_z + self.object_half_z + 0.001
            if float(pos[2]) < z_floor:
                pos = np.asarray(pos, dtype=np.float64).copy()
                pos[2] = z_floor
            self.props.set_pose(
                self.object_name,
                pos=pos,
                quat_xyzw=self._held_quat_xyzw,
            )
            self._zero_velocity()
