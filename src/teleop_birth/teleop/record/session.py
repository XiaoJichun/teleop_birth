"""Lightweight session recording (JSONL) — LeRobot export can come later."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from teleop_birth.robot import BackendState
from teleop_birth.teleop.control.filters import LatencySample
from teleop_birth.teleop.schema import SparseTarget, XRObservation


def _pose_list(pose: np.ndarray) -> list:
    return [float(x) for x in np.asarray(pose).reshape(-1)]


class SessionRecorder:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", encoding="utf-8")
        self.count = 0

    def log(
        self,
        obs: XRObservation,
        target: SparseTarget,
        state: BackendState,
        latency: Optional[LatencySample] = None,
    ) -> None:
        row: Dict[str, Any] = {
            "i": self.count,
            "obs_ts": obs.timestamp_ns,
            "left_ctrl": _pose_list(obs.left_controller),
            "right_ctrl": _pose_list(obs.right_controller),
            "head": _pose_list(obs.head),
            "grip_l": obs.buttons.left_grip,
            "grip_r": obs.buttons.right_grip,
            "trigger_l": obs.buttons.left_trigger,
            "trigger_r": obs.buttons.right_trigger,
            "target": {k: _pose_list(v) for k, v in target.extremities.items()},
            "active": target.active,
            "gripper": [target.left_gripper, target.right_gripper],
            "state": {k: _pose_list(v) for k, v in state.extremities.items()},
        }
        if latency is not None:
            row["loop_ms"] = latency.loop_ms
            row["xr_to_cmd_ms"] = latency.xr_to_cmd_ms
        self._fp.write(json.dumps(row) + "\n")
        self.count += 1

    def close(self) -> None:
        self._fp.close()
