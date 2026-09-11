"""End-effector plugins: gripper now, dexterous hand later — same SparseTarget bus."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np


@dataclass
class EECommand:
    """Normalized command from teleop layer.

    gripper_openness: 0 = fully open, 1 = fully closed (matches trigger convention).
    finger_targets: optional named joints for dex hands (unused by parallel gripper).
    """

    gripper_openness: float = 0.0
    finger_targets: Dict[str, float] = field(default_factory=dict)


@dataclass
class EEState:
    gripper_openness: float = 0.0
    finger_positions: Dict[str, float] = field(default_factory=dict)


class EndEffector(ABC):
    kind: str  # "gripper" | "dex_hand"

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def apply(self, cmd: EECommand) -> None:
        """Write into the backend's control buffer (MuJoCo ctrl / DDS / …)."""

    @abstractmethod
    def read(self) -> EEState: ...


class ParallelGripper(EndEffector):
    """Scalar parallel jaw (Robotiq 2F-85 style)."""

    kind = "gripper"

    def __init__(
        self,
        set_ctrl,
        get_openness,
        ctrl_open: float = 0.0,
        ctrl_closed: float = 255.0,
    ) -> None:
        self._set_ctrl = set_ctrl
        self._get_openness = get_openness
        self.ctrl_open = ctrl_open
        self.ctrl_closed = ctrl_closed
        self._last = 0.0

    def reset(self) -> None:
        self.apply(EECommand(gripper_openness=0.0))

    def apply(self, cmd: EECommand) -> None:
        g = float(np.clip(cmd.gripper_openness, 0.0, 1.0))
        self._last = g
        ctrl = self.ctrl_open + g * (self.ctrl_closed - self.ctrl_open)
        self._set_ctrl(ctrl)

    def read(self) -> EEState:
        try:
            openness = float(self._get_openness())
        except Exception:
            openness = self._last
        return EEState(gripper_openness=openness)


class DexHandStub(EndEffector):
    """Placeholder for Shadow / Inspire / Unitree Dex3.

    Keeps the same EECommand interface. Until retargeting is wired, maps the
    scalar gripper channel to a crude 'all fingers curl' so demos keep working.
    """

    kind = "dex_hand"

    def __init__(self, n_fingers: int = 5, set_joint_ctrls=None) -> None:
        self.n_fingers = n_fingers
        self._set_joint_ctrls = set_joint_ctrls
        self._last = EEState()

    def reset(self) -> None:
        self.apply(EECommand(gripper_openness=0.0))

    def apply(self, cmd: EECommand) -> None:
        if cmd.finger_targets:
            fingers = dict(cmd.finger_targets)
        else:
            # Crude fallback: one DOF openness → all fingers
            fingers = {f"finger_{i}": float(cmd.gripper_openness) for i in range(self.n_fingers)}
        self._last = EEState(
            gripper_openness=float(cmd.gripper_openness),
            finger_positions=fingers,
        )
        if self._set_joint_ctrls is not None:
            self._set_joint_ctrls(fingers)

    def read(self) -> EEState:
        return self._last


def make_end_effector(kind: str, **kwargs) -> EndEffector:
    kind = kind.lower()
    if kind in ("gripper", "parallel", "2f85", "robotiq"):
        return ParallelGripper(**kwargs)
    if kind in ("dex", "dex_hand", "dexterous", "hand"):
        return DexHandStub(**kwargs)
    raise ValueError(f"Unknown end-effector kind: {kind}")
