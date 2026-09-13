"""SSIK analytical IK adapter — https://github.com/personalrobotics/ssik

Solves UR tool0 (attachment_site) in arm base frame; teleop targets TCP (pinch).
"""

from __future__ import annotations

from typing import Any, Optional

import numpy as np

from teleop_birth.ik.frames import (
    read_arm_q,
    tcp_des_to_flange_in_base,
    write_arm_q,
)


class SsikIKSolver:
    def __init__(
        self,
        ik_cfg: dict[str, Any],
        *,
        model,
        data,
        fallback: Optional[Any] = None,
    ) -> None:
        try:
            from ssik.prebuilt.universal_robots import ur3e_ik, ur5e_ik, ur10e_ik
        except ImportError as exc:
            raise ImportError(
                "未安装 ssik。请执行: uv sync --extra ik\n"
                "或: uv pip install 'ssik>=5'\n"
                f"原始错误: {exc}"
            ) from exc

        arm = str(ik_cfg.get("ssik", {}).get("arm", "ur5e")).lower()
        table = {"ur5e": ur5e_ik, "ur3e": ur3e_ik, "ur10e": ur10e_ik, "ur5": ur5e_ik}
        if arm not in table:
            raise ValueError(f"ssik.arm 不支持 {arm!r}（支持 {list(table)}）")
        self._arm = table[arm]
        self.model = model
        self.data = data
        self.fallback = fallback
        ss = dict(ik_cfg.get("ssik") or {})
        self.seed_tolerance = ss.get("seed_tolerance_rad")
        if self.seed_tolerance is not None:
            self.seed_tolerance = float(self.seed_tolerance)
        self.seed_metric = str(ss.get("seed_metric", "wrap_linf"))
        self.max_solutions = int(ss.get("max_solutions", 1))
        print(f"[ik] SSIK solver ready ({arm}, EE={getattr(self._arm, 'EE_LINK', '?')})")

    def solve(self, handles, pose_des: np.ndarray, *, position_only: bool) -> bool:
        T_base_flange = tcp_des_to_flange_in_base(
            model=self.model,
            data=self.data,
            base_bid=handles.base_bid,
            flange_sid=handles.flange_sid,
            tcp_sid=handles.tcp_sid,
            T_flange_tcp=handles.T_flange_tcp,
            pose_des=pose_des,
            position_only=position_only,
        )
        q_seed = read_arm_q(self.model, self.data, handles.arm_aids)
        kwargs: dict[str, Any] = {
            "max_solutions": self.max_solutions,
            "q_seed": q_seed,
            "seed_metric": self.seed_metric,
        }
        if self.seed_tolerance is not None:
            kwargs["seed_tolerance"] = self.seed_tolerance
        try:
            sols = self._arm.solve(T_base_flange, **kwargs)
        except Exception as exc:  # noqa: BLE001
            if self.fallback is not None:
                return self.fallback.solve(handles, pose_des, position_only=position_only)
            raise RuntimeError(f"ssik.solve 失败: {exc}") from exc

        if not sols:
            if self.fallback is not None:
                return self.fallback.solve(handles, pose_des, position_only=position_only)
            return False

        write_arm_q(self.model, self.data, handles.arm_aids, np.asarray(sols[0].q, dtype=np.float64))
        return True
