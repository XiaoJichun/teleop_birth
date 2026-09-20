"""银河通用 Galaxea — 轮式人形登记（后端待接官方 SDK / 仿真）。"""

from __future__ import annotations

from teleop_birth.robot.base import BackendState, RobotBackend
from teleop_birth.robot.registry import RobotSpec, register_robot
from teleop_birth.teleop.schema import (
    EXTREMITY_LEFT_EE,
    EXTREMITY_PELVIS,
    EXTREMITY_RIGHT_EE,
    SparseTarget,
    identity_pose,
)


class GalaxeaBackendStub(RobotBackend):
    def __init__(self, model: str = "r1") -> None:
        self.model = model
        self._pose = {
            EXTREMITY_LEFT_EE: identity_pose(),
            EXTREMITY_RIGHT_EE: identity_pose(),
            EXTREMITY_PELVIS: identity_pose(),
        }
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def get_feedback(self):
        return {k: v.copy() for k, v in self._pose.items()}

    def apply(self, target: SparseTarget) -> BackendState:
        for name in (EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, EXTREMITY_PELVIS):
            if name in target.extremities and target.is_active(name):
                self._pose[name] = target.extremities[name].copy()
        return BackendState(extremities=self.get_feedback())


def build_backend(**kwargs) -> GalaxeaBackendStub:
    return GalaxeaBackendStub(**{k: v for k, v in kwargs.items() if k in ("model",)})


register_robot(
    RobotSpec(
        id="galaxea_r1",
        title="银河通用 Galaxea R1（轮式人形）",
        category="wheeled_humanoid",
        vendor="Galaxea",
        status="stub",
        module="teleop_birth.robot.wheeled.galaxea",
        factory="build_backend",
        notes="已登记占位；接入官方仿真/SDK 后改 status=ready",
        tags=("wheeled", "humanoid", "dual_arm"),
        compatible_grippers=(),
        scene_preview=False,
    )
)
