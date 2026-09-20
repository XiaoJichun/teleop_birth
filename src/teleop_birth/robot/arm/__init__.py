"""机械臂类别：UR / Franka / 自变量 ArtiXon。

子模块在属性访问时再加载，避免 embodiment ↔ robots 循环。
"""

from __future__ import annotations

__all__ = ["artixon", "franka", "ur5e"]


def __getattr__(name: str):
    if name == "artixon":
        from teleop_birth.robot.arm import artixon as mod

        return mod
    if name == "franka":
        from teleop_birth.robot.arm import franka as mod

        return mod
    if name == "ur5e":
        from teleop_birth.robot.arm import ur5e as mod

        return mod
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
