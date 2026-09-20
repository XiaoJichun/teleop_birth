"""物流分拣任务：源箱取包裹 → 目标箱放置。"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from teleop_birth.robot.arm.artixon.ik_dls import top_down_quat_xyzw
from teleop_birth.robot.arm.artixon.scene.props import PropPoseController


@dataclass
class Waypoints:
    poses: list[np.ndarray]
    grippers: list[float]
    labels: list[str]


def build_sort_waypoints(
    props: PropPoseController,
    *,
    package_name: str,
    dest_xy: tuple[float, float],
    dest_z: float,
    approach_z: float = 0.12,
    grasp_aim_up: float = 0.03,
    place_hover_z: float = 0.10,
    grasp_close: float = 0.55,
    tcp_offset_x: float = 0.02,
) -> Waypoints:
    """TCP 在夹爪基座前方；朝下抓取时用世界系抬高对准。"""
    pkg = props.get_pose(package_name)
    q = top_down_quat_xyzw()

    def pose_at(xy: np.ndarray, z: float) -> np.ndarray:
        # 朝下时工具 +X ≈ 世界水平某向；简化：TCP 目标直接用世界位姿，
        # sticky 用 grasp_offset 把物体钉在 TCP 前方。
        return np.array([xy[0], xy[1], z, q[0], q[1], q[2], q[3]], dtype=np.float64)

    cxy = pkg[:2]
    grasp_z = float(pkg[2] + grasp_aim_up)
    above = pose_at(cxy, float(pkg[2] + approach_z))
    grasp = pose_at(cxy, grasp_z)
    lift = pose_at(cxy, float(pkg[2] + approach_z))
    hover = pose_at(np.asarray(dest_xy, dtype=np.float64), float(dest_z + place_hover_z))

    poses = [above, grasp, grasp, lift, hover, hover]
    grippers = [0.0, 0.0, grasp_close, grasp_close, grasp_close, 0.0]
    labels = [
        "approach",
        "descend_open",
        "squeeze",
        "lift",
        "transport",
        "place_release",
    ]
    return Waypoints(poses=poses, grippers=grippers, labels=labels)
