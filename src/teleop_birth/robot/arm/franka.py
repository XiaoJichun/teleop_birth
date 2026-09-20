"""Franka Emika Panda — catalog 登记；后端待接 menagerie/franka_emika_panda。"""

from __future__ import annotations

from teleop_birth.robot.base import BackendState, RobotBackend
from teleop_birth.robot.registry import RobotSpec, register_robot
from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, SparseTarget, identity_pose


class FrankaBackendStub(RobotBackend):
    """占位：后续接 mujoco_menagerie/franka_emika_panda 或真机 libfranka。"""

    def __init__(self) -> None:
        self._pose = {EXTREMITY_LEFT_EE: identity_pose()}
        self.connected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def get_feedback(self):
        return {k: v.copy() for k, v in self._pose.items()}

    def apply(self, target: SparseTarget) -> BackendState:
        if EXTREMITY_LEFT_EE in target.extremities and target.is_active(EXTREMITY_LEFT_EE):
            self._pose[EXTREMITY_LEFT_EE] = target.extremities[EXTREMITY_LEFT_EE].copy()
        return BackendState(extremities=self.get_feedback())


def build_backend(**kwargs) -> FrankaBackendStub:
    return FrankaBackendStub()


register_robot(
    RobotSpec(
        id="franka_panda",
        title="Franka Emika Panda",
        category="arm",
        vendor="Franka Emika",
        status="stub",
        module="teleop_birth.robot.arm.franka",
        factory="build_backend",
        assets="third_party/mujoco_menagerie/franka_emika_panda",
        notes="占位：尚未接入场景搭建/仿真；就绪后开放夹爪与预览",
        tags=("single_arm", "menagerie"),
        compatible_grippers=(),  # 未实现前不开放任何夹爪
        scene_preview=False,
    )
)
