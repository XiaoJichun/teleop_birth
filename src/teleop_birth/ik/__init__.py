"""Cartesian IK solvers (extractable package).

solver: dls | ssik | curobo — selected via configs/algorithm/ik/*.yaml
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, List

import numpy as np


@dataclass
class ArmIKHandles:
    name: str
    arm_aids: List[int]
    tcp_sid: int
    flange_sid: int
    base_bid: int
    T_flange_tcp: np.ndarray


class ArmIKSolver(ABC):
    @abstractmethod
    def solve(self, handles: ArmIKHandles, pose_des: np.ndarray, *, position_only: bool) -> bool: ...


def build_arm_ik_solver(ik_cfg: dict[str, Any], *, model, data) -> ArmIKSolver:
    from teleop_birth.ik.dls import DlsIKSolver

    kind = str(ik_cfg.get("solver", "dls")).lower().strip()
    fallback = str(ik_cfg.get("fallback", "dls")).lower().strip()
    dls = DlsIKSolver(ik_cfg, model=model, data=data)

    if kind in ("dls", "numeric", "jacobian"):
        return dls

    if kind in ("ssik", "analytical"):
        try:
            from teleop_birth.ik.ssik_solver import SsikIKSolver

            return SsikIKSolver(
                ik_cfg, model=model, data=data, fallback=dls if fallback == "dls" else None
            )
        except Exception as exc:  # noqa: BLE001
            if fallback == "dls":
                print(f"[ik] ssik 不可用 ({exc}); 回退 dls")
                return dls
            raise

    if kind in ("curobo", "cu_robo", "nvidia"):
        try:
            from teleop_birth.ik.curobo_solver import CuroboIKSolver

            return CuroboIKSolver(
                ik_cfg, model=model, data=data, fallback=dls if fallback == "dls" else None
            )
        except Exception as exc:  # noqa: BLE001
            if fallback == "dls":
                print(f"[ik] curobo 不可用 ({exc}); 回退 dls")
                return dls
            raise

    raise ValueError(f"未知 ik.solver={kind!r}（支持 dls | ssik | curobo）")


__all__ = ["ArmIKHandles", "ArmIKSolver", "build_arm_ik_solver"]
