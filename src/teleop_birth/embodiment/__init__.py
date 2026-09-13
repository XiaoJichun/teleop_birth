"""Embodiments: sim / real robot drivers behind a common bus.

Prefer robot_io ports for new work. RobotBackend remains for TeleopLoop.
"""

from teleop_birth.embodiment.base import BackendState, LoggingBackend, RobotBackend
from teleop_birth.embodiment.real import G1BackendStub
from teleop_birth.embodiment.sim import MockDualArmBackend, MujocoDualURBackend

__all__ = [
    "BackendState",
    "G1BackendStub",
    "LoggingBackend",
    "MockDualArmBackend",
    "MujocoDualURBackend",
    "RobotBackend",
]
