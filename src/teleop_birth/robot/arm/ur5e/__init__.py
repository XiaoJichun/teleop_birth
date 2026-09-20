"""Universal Robots UR5e（menagerie，双臂就绪）。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from teleop_birth.robot.arm.ur5e.builder import (
    actuator_ids,
    build_dual_ur5e_2f85,
    resolve_menagerie,
    site_id,
)
from teleop_birth.robot.registry import RobotSpec, register_robot

__all__ = [
    "actuator_ids",
    "build_backend",
    "build_dual_ur5e_2f85",
    "resolve_menagerie",
    "site_id",
]


def build_backend(
    *,
    menagerie: Path | str,
    scene_cfg: dict[str, Any],
    ik_cfg: dict[str, Any] | None = None,
    ee_cfg: dict[str, Any] | None = None,
    render: bool = False,
    **kwargs,
):
    from teleop_birth.robot.sim.mujoco_dual_ur import MujocoDualURBackend

    return MujocoDualURBackend(
        menagerie=Path(menagerie),
        scene_cfg=scene_cfg,
        ik_cfg=ik_cfg or {},
        ee_cfg=ee_cfg or {},
        render=render,
        **kwargs,
    )


register_robot(
    RobotSpec(
        id="ur5e",
        title="Universal Robots UR5e",
        category="arm",
        vendor="Universal Robots",
        status="ready",
        module="teleop_birth.robot.arm.ur5e",
        factory="build_backend",
        assets="third_party/mujoco_menagerie/universal_robots_ur5e",
        notes="双臂 + Robotiq 2F-85；Hydra backend=mujoco_dual_ur",
        tags=("dual_arm", "menagerie", "mujoco"),
        compatible_grippers=("robotiq_2f85",),
        scene_preview=True,
    )
)
