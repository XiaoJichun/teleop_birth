"""宇树 Unitree G1 — 双足人形（SparseTarget 总线已通，真机/策略待接）。"""

from __future__ import annotations

from teleop_birth.robot.real.g1_stub import G1BackendStub
from teleop_birth.robot.registry import RobotSpec, register_robot


def build_backend(*, mode: str = "upper_body", **kwargs) -> G1BackendStub:
    return G1BackendStub(mode=mode)


register_robot(
    RobotSpec(
        id="unitree_g1",
        title="Unitree G1（双足人形）",
        category="biped",
        vendor="Unitree",
        status="stub",
        module="teleop_birth.robot.biped.unitree_g1",
        factory="build_backend",
        notes="G1BackendStub 已可走 TeleopLoop；接 DDS / HuMiT / MOSAIC 后升 ready",
        tags=("biped", "humanoid", "unitree"),
        compatible_grippers=(),
        scene_preview=False,
    )
)
