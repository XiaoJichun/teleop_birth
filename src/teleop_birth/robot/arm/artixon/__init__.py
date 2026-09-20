"""自变量 ArtiXon Arm-6A（双臂物流就绪）。"""

from __future__ import annotations

from typing import Any

from teleop_birth.robot.arm.artixon.backend import DualArtixOnBackend
from teleop_birth.robot.arm.artixon.builder import build_logistics_scene
from teleop_birth.robot.registry import RobotSpec, register_robot


def build_backend(*, scene_cfg: dict[str, Any] | None = None, render: bool = False, **kwargs) -> DualArtixOnBackend:
    return DualArtixOnBackend(scene_cfg=scene_cfg, render=render, **kwargs)


def build_teleop_backend(**kwargs):
    from teleop_birth.robot.arm.artixon.teleop_backend import LogisticsTeleopBackend

    return LogisticsTeleopBackend(**kwargs)


register_robot(
    RobotSpec(
        id="artixon_arm_6a",
        title="ArtiXon Arm-6A（自变量）",
        category="arm",
        vendor="X-Square / ArtiXon",
        status="ready",
        module="teleop_birth.robot.arm.artixon",
        factory="build_backend",
        assets="assets/robots/arm/artixon_6a",
        notes="双臂物流分拣场景；内置平行爪",
        tags=("dual_arm", "logistics", "mujoco"),
        compatible_grippers=("artixon_parallel",),
        scene_preview=True,
    )
)


def __getattr__(name: str):
    if name == "LogisticsTeleopBackend":
        from teleop_birth.robot.arm.artixon.teleop_backend import LogisticsTeleopBackend

        return LogisticsTeleopBackend
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


__all__ = [
    "DualArtixOnBackend",
    "LogisticsTeleopBackend",
    "build_backend",
    "build_logistics_scene",
    "build_teleop_backend",
]
