"""采集用粘性抓取：闭合后物体对齐到夹爪内偏移，并关闭碰撞避免穿模。"""

from __future__ import annotations

from typing import Optional

import mujoco
import numpy as np

from teleop_birth.scene_ops.placement.pose_controller import PropPoseController
from teleop_birth.schema import matrix_to_pose, pose_to_matrix


class StickyGrasp:
    def __init__(
        self,
        props: PropPoseController,
        *,
        object_name: str,
        model: mujoco.MjModel | None = None,
        attach_radius_m: float = 0.08,
        # 物体中心相对 TCP（夹爪 site）的工具系偏移；+Z 通常朝向桌面/夹取方向
        grasp_offset_xyz: list[float] | tuple[float, float, float] = (0.0, 0.0, 0.028),
        place_object: str | None = None,
        place_radius_m: float = 0.10,
        place_z_offset: float = 0.03,
    ) -> None:
        self.props = props
        self.object_name = object_name
        self.model = model
        self.attach_radius_m = float(attach_radius_m)
        self.grasp_offset = np.asarray(grasp_offset_xyz, dtype=np.float64).reshape(3)
        self.place_object = place_object
        self.place_radius_m = float(place_radius_m)
        self.place_z_offset = float(place_z_offset)
        self._T_ee_obj: Optional[np.ndarray] = None
        self.did_place = False
        self._geom_ids: list[int] = []
        self._saved_contype: list[int] = []
        self._saved_conaffinity: list[int] = []
        if model is not None:
            self._cache_object_geoms(model)

    def _cache_object_geoms(self, model: mujoco.MjModel) -> None:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, self.object_name)
        if bid < 0:
            return
        for gid in range(model.ngeom):
            if int(model.geom_bodyid[gid]) == bid:
                self._geom_ids.append(gid)
                self._saved_contype.append(int(model.geom_contype[gid]))
                self._saved_conaffinity.append(int(model.geom_conaffinity[gid]))

    def _set_collisions(self, enabled: bool) -> None:
        if self.model is None or not self._geom_ids:
            return
        for i, gid in enumerate(self._geom_ids):
            if enabled:
                self.model.geom_contype[gid] = self._saved_contype[i]
                self.model.geom_conaffinity[gid] = self._saved_conaffinity[i]
            else:
                self.model.geom_contype[gid] = 0
                self.model.geom_conaffinity[gid] = 0

    @property
    def attached(self) -> bool:
        return self._T_ee_obj is not None

    def _snap_relative(self, ee_pose: np.ndarray) -> np.ndarray:
        """物体中心对齐到 TCP + 工具系偏移，姿态与 EE 一致（避免斜插穿模）。"""
        T_ee = pose_to_matrix(ee_pose)
        T_obj = np.eye(4, dtype=np.float64)
        T_obj[:3, :3] = T_ee[:3, :3]
        T_obj[:3, 3] = T_ee[:3, 3] + T_ee[:3, :3] @ self.grasp_offset
        return np.linalg.inv(T_ee) @ T_obj

    def update(self, ee_pose: np.ndarray, gripper_closed: float) -> None:
        ee = np.asarray(ee_pose, dtype=np.float64)
        obj = self.props.get_pose(self.object_name)
        if gripper_closed >= 0.55:
            if self._T_ee_obj is None:
                # 需足够近才吸附，并立刻对齐到夹爪内，避免「没夹到却跟着走」
                if float(np.linalg.norm(ee[:3] - obj[:3])) <= self.attach_radius_m:
                    self._T_ee_obj = self._snap_relative(ee)
                    self._set_collisions(False)
            if self._T_ee_obj is not None:
                T_ee = pose_to_matrix(ee)
                pose = matrix_to_pose(T_ee @ self._T_ee_obj)
                self.props.set_pose(self.object_name, pos=pose[:3], quat_xyzw=pose[3:])
            return

        if self._T_ee_obj is not None and self.place_object:
            plate = self.props.get_pose(self.place_object)
            if float(np.linalg.norm(ee[:2] - plate[:2])) <= self.place_radius_m:
                self._pin_to_place()
                self.did_place = True
            self._set_collisions(True)
        elif self._T_ee_obj is not None:
            self._set_collisions(True)
        self._T_ee_obj = None

        if self.did_place and self.place_object:
            self._pin_to_place()

    def _pin_to_place(self) -> None:
        assert self.place_object is not None
        plate = self.props.get_pose(self.place_object)
        drop = plate.copy()
        drop[2] = float(plate[2] + self.place_z_offset)
        self.props.set_pose(self.object_name, pos=drop[:3], quat_xyzw=drop[3:])
