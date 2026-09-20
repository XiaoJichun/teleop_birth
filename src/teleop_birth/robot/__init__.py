"""机型与驱动：RobotBackend、末端、双端口、按 id 注册加载。"""

from __future__ import annotations

from typing import Any

from teleop_birth.robot.base import BackendState, LoggingBackend, RobotBackend
from teleop_birth.robot.registry import (
    RobotSpec,
    get_robot as _get_robot,
    grippers_compatible_with as _grippers_compatible_with,
    list_robots as _list_robots,
    load_backend as _load_backend,
    register_robot,
)

__all__ = [
    "BackendState",
    "G1BackendStub",
    "LoggingBackend",
    "MockDualArmBackend",
    "MujocoDualURBackend",
    "RobotBackend",
    "RobotSpec",
    "get_robot",
    "grippers_compatible_with",
    "list_robots",
    "load_backend",
    "register_robot",
]

_LOADED = False


def _ensure_registered() -> None:
    global _LOADED
    if _LOADED:
        return
    import teleop_birth.robot.arm.artixon  # noqa: F401
    import teleop_birth.robot.arm.franka  # noqa: F401
    import teleop_birth.robot.arm.ur5e  # noqa: F401
    import teleop_birth.robot.biped.unitree_g1  # noqa: F401
    import teleop_birth.robot.wheeled.galaxea  # noqa: F401

    _LOADED = True


def get_robot(robot_id: str) -> RobotSpec:
    _ensure_registered()
    return _get_robot(robot_id)


def list_robots(
    *,
    category: str | None = None,
    ready_only: bool = False,
    preview_only: bool = False,
) -> list[RobotSpec]:
    _ensure_registered()
    return _list_robots(category=category, ready_only=ready_only, preview_only=preview_only)


def grippers_compatible_with(robot_id: str) -> tuple[str, ...]:
    _ensure_registered()
    return _grippers_compatible_with(robot_id)


def load_backend(robot_id: str, **kwargs: Any):
    _ensure_registered()
    return _load_backend(robot_id, **kwargs)


def __getattr__(name: str):
    if name == "MujocoDualURBackend":
        from teleop_birth.robot.sim import MujocoDualURBackend

        return MujocoDualURBackend
    if name == "MockDualArmBackend":
        from teleop_birth.robot.sim import MockDualArmBackend

        return MockDualArmBackend
    if name == "G1BackendStub":
        from teleop_birth.robot.real import G1BackendStub

        return G1BackendStub
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
