"""右臂夹取红色方块 → 放入蓝色盘子（语义自动化）。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from teleop_birth.scene_ops.placement.pose_controller import PropPoseController
from teleop_birth.scene_ops.props.catalog import PropCatalog, PropSpec
from teleop_birth.scene_ops.tasks.base import TaskSpec, TaskStage


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

    def build_waypoints(
        self,
        props: PropPoseController,
        *,
        approach_z: float = 0.12,
        grasp_z: float = 0.022,
        lift_z: float = 0.12,
        place_z: float = 0.05,
        down_quat_xyzw: Optional[np.ndarray] = None,
    ) -> PickPlaceWaypoints:
        """连续路点（无重复停顿点）；夹爪开合在段内插值完成。"""
        cube = props.get_pose(self.grasp_object)
        plate = props.get_pose(self.place_object)
        q = (
            np.asarray(down_quat_xyzw, dtype=np.float64)
            if down_quat_xyzw is not None
            else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        )

        def pose_at(xy: np.ndarray, z: float) -> np.ndarray:
            return np.array([xy[0], xy[1], z, q[0], q[1], q[2], q[3]], dtype=np.float64)

        cxy = cube[:2]
        pxy = plate[:2]
        poses = [
            pose_at(cxy, float(cube[2] + approach_z)),
            pose_at(cxy, float(cube[2] + grasp_z)),
            pose_at(cxy, float(cube[2] + lift_z)),
            pose_at(pxy, float(plate[2] + lift_z)),
            pose_at(pxy, float(plate[2] + place_z)),
            pose_at(pxy, float(plate[2] + lift_z)),
        ]
        # 在 descend 段末闭合；在 place 段末打开
        grippers = [0.0, 1.0, 1.0, 1.0, 0.0, 0.0]
        labels = [
            "approach_cube",
            "descend_grasp",
            "lift_cube",
            "transport",
            "place_release",
            "retreat",
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
