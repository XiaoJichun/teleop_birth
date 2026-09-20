"""场景换臂等运维操作。"""

from teleop_birth.scene.ops.swap_arm import (
    ContactPose,
    SwapResult,
    desktop_arm_id,
    extract_contact,
    infer_robot_id,
    swap_scene_robot,
    supported_robot_ids,
    write_scene_yaml,
)

__all__ = [
    "ContactPose",
    "SwapResult",
    "desktop_arm_id",
    "extract_contact",
    "infer_robot_id",
    "swap_scene_robot",
    "supported_robot_ids",
    "write_scene_yaml",
]
