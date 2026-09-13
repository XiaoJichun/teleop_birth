"""Damped least-squares IK (current default) on MuJoCo site Jacobian."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from teleop_birth.backend.ik.frames import mat_to_6err
from teleop_birth.schema import pose_to_matrix


class DlsIKSolver:
    def __init__(self, ik_cfg: dict[str, Any], *, model, data) -> None:
        self.model = model
        self.data = data
        self.ik_iters = int(ik_cfg.get("iters", 16))
        self.damping = float(ik_cfg.get("damping", 0.05))
        self.pos_weight = float(ik_cfg.get("pos_weight", 1.0))
        self.rot_weight = float(ik_cfg.get("rot_weight", 0.55))
        self.max_dq = float(ik_cfg.get("max_dq", 0.22))
        self.err_tol = float(ik_cfg.get("err_tol", 0.0008))

    def solve(self, handles, pose_des: np.ndarray, *, position_only: bool) -> bool:
        from teleop_birth.backend.ik import ArmIKHandles

        assert isinstance(handles, ArmIKHandles)
        model, data = self.model, self.data
        T = pose_to_matrix(pose_des)
        pos_des, R_des = T[:3, 3], T[:3, :3]
        arm_aids = handles.arm_aids
        sid = handles.tcp_sid
        joint_ids = [model.actuator_trnid[a, 0] for a in arm_aids]
        qadr = [model.jnt_qposadr[j] for j in joint_ids]
        dof_ids = [model.jnt_dofadr[j] for j in joint_ids]
        rot_w = 0.0 if position_only else self.rot_weight
        n_task = 3 if position_only else 6

        for _ in range(self.ik_iters):
            mujoco.mj_forward(model, data)
            pos = data.site_xpos[sid]
            R = data.site_xmat[sid].reshape(3, 3)
            err6 = mat_to_6err(pos_des, R_des, pos, R)
            err6[:3] *= self.pos_weight
            err6[3:] *= rot_w
            err = err6[:n_task]
            if np.linalg.norm(err) < self.err_tol:
                break

            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, jacr, sid)
            J = jacp[:, dof_ids] if position_only else np.vstack([jacp[:, dof_ids], jacr[:, dof_ids]])
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + self.damping * np.eye(n_task), err)
            dq = np.clip(dq, -self.max_dq, self.max_dq)
            for a, qa, dqi in zip(arm_aids, qadr, dq):
                q_new = float(data.qpos[qa] + dqi)
                lo, hi = model.actuator_ctrlrange[a]
                if hi > lo:
                    q_new = float(np.clip(q_new, lo, hi))
                data.qpos[qa] = q_new
                data.ctrl[a] = q_new
        return True
