"""Damped least-squares IK (current default) on MuJoCo site Jacobian."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np

from teleop_birth.teleop.ik.frames import mat_to_6err
from teleop_birth.teleop.schema import pose_to_matrix


class DlsIKSolver:
    def __init__(self, ik_cfg: dict[str, Any], *, model, data) -> None:
        self.model = model
        self.data = data
        self.ik_iters = int(ik_cfg.get("iters", 16))
        self.damping = float(ik_cfg.get("damping", 0.05))
        self.pos_weight = float(ik_cfg.get("pos_weight", 1.0))
        self.rot_weight = float(ik_cfg.get("rot_weight", 0.55))
        # 姿态误差按弧度、平移按米。不截断时 60° 误差会吃掉整步 dq，TCP 几乎不平移。
        self.ori_err_cap = float(ik_cfg.get("ori_err_cap", 0.25))
        self.max_dq = float(ik_cfg.get("max_dq", 0.22))
        self.err_tol = float(ik_cfg.get("err_tol", 0.0008))

    def solve(self, handles, pose_des: np.ndarray, *, position_only: bool) -> bool:
        from teleop_birth.teleop.ik import ArmIKHandles

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
        # TCP 离目标超过这个距离时，这一拍只追位置。否则姿态会把整臂拽偏。
        pos_gate_m = 0.02

        for _ in range(self.ik_iters):
            mujoco.mj_forward(model, data)
            pos = data.site_xpos[sid]
            R = data.site_xmat[sid].reshape(3, 3)
            err6 = mat_to_6err(pos_des, R_des, pos, R)
            pos_err = float(np.linalg.norm(err6[:3]))
            chase_ori = rot_w > 0.0 and pos_err <= pos_gate_m
            if chase_ori and self.ori_err_cap > 0.0:
                ori = err6[3:]
                n_ori = float(np.linalg.norm(ori))
                if n_ori > self.ori_err_cap:
                    err6[3:] = ori * (self.ori_err_cap / n_ori)
            if pos_err < self.err_tol and not chase_ori:
                break
            if chase_ori and float(np.linalg.norm(err6[3:])) < self.err_tol and pos_err < self.err_tol:
                break

            jacp = np.zeros((3, model.nv))
            jacr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, jacp, jacr, sid)
            Jp = jacp[:, dof_ids]
            if not chase_ori:
                J = Jp
                err = err6[:3] * self.pos_weight
            else:
                # 姿态走位置雅可比的零空间，不把 TCP 拉开。
                Jr = jacr[:, dof_ids]
                damp = self.damping
                n = Jp.shape[1]
                A = Jp @ Jp.T + damp * np.eye(3)
                Jp_pinv = Jp.T @ np.linalg.solve(A, np.eye(3))
                N = np.eye(n) - Jp_pinv @ Jp
                Jr_n = Jr @ N
                B = Jr_n @ Jr_n.T + damp * np.eye(3)
                dq_ori = N @ (Jr_n.T @ np.linalg.solve(B, err6[3:] * rot_w))
                dq_pos = Jp_pinv @ (err6[:3] * self.pos_weight)
                dq = dq_pos + dq_ori
                dq = np.clip(dq, -self.max_dq, self.max_dq)
                self._apply_dq(model, data, arm_aids, qadr, dq)
                continue
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + self.damping * np.eye(3), err)
            dq = np.clip(dq, -self.max_dq, self.max_dq)
            self._apply_dq(model, data, arm_aids, qadr, dq)
        return True

    @staticmethod
    def _apply_dq(model, data, arm_aids, qadr, dq) -> None:
        for a, qa, dqi in zip(arm_aids, qadr, dq):
            q_new = float(data.qpos[qa] + dqi)
            lo, hi = model.actuator_ctrlrange[a]
            if hi > lo:
                q_new = float(np.clip(q_new, lo, hi))
            data.qpos[qa] = q_new
            data.ctrl[a] = q_new
