"""右臂夹取红色方块 → 放入蓝色盘子（语义自动化）。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleop_birth.scene.collection.grasp_geometry import (
    DEFAULT_PAD_OFFSET_XYZ,
    gripper_close_for_width,
    pinch_pose_for_object,
    top_down_quat_xyzw,
)
from teleop_birth.scene.placement.pose_controller import PropPoseController
from teleop_birth.scene.props.catalog import PropCatalog, PropSpec
from teleop_birth.scene.tasks.base import TaskSpec, TaskStage


@dataclass
class PickPlaceWaypoints:
    """一组笛卡尔路点（世界系 xyz + quat xyzw）与夹爪开合。"""

    poses: list[np.ndarray]
    grippers: list[float]  # 0 open … 1 closed
    labels: list[str]


class SemanticPickPlaceTask:
    """语义: 「右臂夹取红色的方块，并放在蓝色的盘子里面」。"""

    def __init__(
        self,
        catalog: PropCatalog | None = None,
        *,
        grasp_object: str = "red_cube",
        place_object: str = "blue_plate",
        arm: str = "right",
    ) -> None:
        self.catalog = catalog or PropCatalog()
        self.grasp_object = grasp_object
        self.place_object = place_object
        self.arm = arm

    def prop_specs(self) -> list[PropSpec]:
        return self.catalog.resolve_many([self.grasp_object, self.place_object])

    def task_spec(self, episodes: int = 5, seed: int = 0) -> TaskSpec:
        return TaskSpec(
            name="right_pick_red_cube_to_blue_plate",
            instruction_zh="右臂夹取红色的方块，并放在蓝色的盘子里面",
            instruction_en="Use the right arm to pick the red cube and place it into the blue plate",
            objects=[self.grasp_object, self.place_object],
            stages=[
                TaskStage(
                    action="pick",
                    description="右臂夹取红色方块",
                    active_object=self.grasp_object,
                    arm=self.arm,
                ),
                TaskStage(
                    action="place",
                    description="将方块放入蓝色盘子",
                    active_object=self.grasp_object,
                    passive_object=self.place_object,
                    arm=self.arm,
                ),
                TaskStage(action="reset", description="右臂复位", arm=self.arm),
            ],
            episodes=episodes,
            seed=seed,
        )

    def _object_width_m(self) -> float:
        spec = self.catalog.get(self.grasp_object)
        size = list(spec.size or [0.025, 0.025, 0.025])
        # box: 半尺寸；plate: [radius, half_h] — 抓取物按 XY 最小边
        if spec.kind == "box" and len(size) >= 2:
            return float(2.0 * min(size[0], size[1]))
        return float(2.0 * size[0]) if size else 0.05

    def _object_half_z(self) -> float:
        spec = self.catalog.get(self.grasp_object)
        size = list(spec.size or [0.025, 0.025, 0.025])
        if spec.kind == "box" and len(size) >= 3:
            return float(size[2])
        return float(size[0]) if size else 0.025

    def build_waypoints(
        self,
        props: PropPoseController,
        *,
        approach_z: float = 0.14,
        lift_z: float = 0.16,
        # 指垫对准中心略偏上：指尖清桌，同时仍能夹到方块侧面
        grasp_aim_up: float = 0.015,
        # 放置悬停：足够高，搬运时方块不穿盘子
        place_hover_z: float = 0.14,
        table_top_z: float = 0.42,
        # TCP 相对桌面最小高度；过大→指垫够不着物体，只能靠磁吸
        tcp_table_clearance: float = 0.010,
        down_quat_xyzw: Optional[np.ndarray] = None,
        grasp_offset_xyz: tuple[float, float, float] | list[float] = DEFAULT_PAD_OFFSET_XYZ,
    ) -> PickPlaceWaypoints:
        """先到位再半合；放置只在悬停高度松爪，不再往下探。"""
        cube = props.get_pose(self.grasp_object)
        plate = props.get_pose(self.place_object)
        q = (
            np.asarray(down_quat_xyzw, dtype=np.float64)
            if down_quat_xyzw is not None
            else top_down_quat_xyzw()
        )
        g_close = gripper_close_for_width(self._object_width_m())

        aim_up = float(max(0.0, min(float(grasp_aim_up), 0.02)))
        grasp_aim = np.array(
            [cube[0], cube[1], float(cube[2] + aim_up)], dtype=np.float64
        )
        grasp = pinch_pose_for_object(
            grasp_aim, quat_xyzw=q, grasp_offset_xyz=grasp_offset_xyz
        )
        # 仅挡住「TCP 钻进桌面」；不得把抓取抬到指垫够不着物体
        z_min = float(table_top_z) + float(tcp_table_clearance)
        if float(grasp[2]) < z_min:
            grasp[2] = z_min

        above_cube = grasp.copy()
        above_cube[2] = float(max(grasp[2] + 0.10, cube[2] + approach_z))

        hover_z = float(max(place_hover_z, self._object_half_z() + 0.08))
        hover = pinch_pose_for_object(
            np.array(
                [plate[0], plate[1], float(plate[2] + hover_z)],
                dtype=np.float64,
            ),
            quat_xyzw=q,
            grasp_offset_xyz=grasp_offset_xyz,
        )
        if float(hover[2]) < z_min + 0.06:
            hover[2] = z_min + 0.06

        lift = grasp.copy()
        lift[2] = float(max(grasp[2] + 0.10, cube[2] + lift_z, hover[2]))

        # 闭合后短距上抬，撕开桌面接触；步数由 collector 压短，避免久停
        squeeze_pose = grasp.copy()
        hold_pose = grasp.copy()
        hold_pose[2] = float(grasp[2] + 0.012)

        poses = [
            above_cube,
            grasp,
            squeeze_pose,
            hold_pose,
            lift,
            hover,
            hover,
        ]
        grippers = [0.0, 0.0, g_close, g_close, g_close, g_close, 0.0]
        labels = [
            "approach_cube",
            "descend_open",
            "squeeze",
            "squeeze_hold",
            "lift_cube",
            "transport",
            "place_release",
        ]
        return PickPlaceWaypoints(poses=poses, grippers=grippers, labels=labels)

    def success(
        self,
        props: PropPoseController,
        *,
        xy_tol: float = 0.06,
        z_tol: float = 0.08,
    ) -> bool:
        cube = props.get_pose(self.grasp_object)
        plate = props.get_pose(self.place_object)
        plate_spec = self.catalog.get(self.place_object)
        r = float(plate_spec.place_radius_m or xy_tol)
        xy_ok = float(np.linalg.norm(cube[:2] - plate[:2])) <= max(r, xy_tol)
        z_ok = abs(float(cube[2] - plate[2])) <= z_tol
        return xy_ok and z_ok
