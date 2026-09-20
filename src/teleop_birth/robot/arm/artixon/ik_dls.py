"""轻量 DLS IK（仅本模块依赖 mujoco/numpy）。"""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np


def pose_to_matrix(pose_xyzw: np.ndarray) -> np.ndarray:
    p = np.asarray(pose_xyzw, dtype=np.float64).reshape(7)
    x, y, z, qx, qy, qz, qw = p
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [x, y, z]
    n = max(np.linalg.norm([qx, qy, qz, qw]), 1e-12)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    T[:3, :3] = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )
    return T


def matrix_to_pose(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    tr = float(np.trace(R))
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + R[0, 0] - R[1, 1] - R[2, 2]))
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + R[1, 1] - R[0, 0] - R[2, 2]))
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(1e-12, 1.0 + R[2, 2] - R[0, 0] - R[1, 1]))
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
    return np.array([t[0], t[1], t[2], qx, qy, qz, qw], dtype=np.float64)


def _mat_to_6err(pos_des, R_des, pos, R) -> np.ndarray:
    err = np.zeros(6, dtype=np.float64)
    err[:3] = pos_des - pos
    R_err = R_des @ R.T
    err[3] = 0.5 * (R_err[2, 1] - R_err[1, 2])
    err[4] = 0.5 * (R_err[0, 2] - R_err[2, 0])
    err[5] = 0.5 * (R_err[1, 0] - R_err[0, 1])
    return err


def top_down_quat_xyzw() -> np.ndarray:
    """Arm-6A 俯抓：工具 +X 朝下，+Y 沿世界 +Y（夹爪开合）。"""
    x = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    y = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    z = np.cross(x, y)
    z /= max(float(np.linalg.norm(z)), 1e-8)
    y = np.cross(z, x)
    y /= max(float(np.linalg.norm(y)), 1e-8)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.column_stack([x, y, z])
    return matrix_to_pose(T)[3:]


class DlsIK:
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, cfg: dict[str, Any] | None = None) -> None:
        cfg = dict(cfg or {})
        self.model = model
        self.data = data
        self.iters = int(cfg.get("iters", 24))
        self.damping = float(cfg.get("damping", 0.06))
        self.pos_weight = float(cfg.get("pos_weight", 1.0))
        self.rot_weight = float(cfg.get("rot_weight", 0.7))
        self.max_dq = float(cfg.get("max_dq", 0.25))
        self.err_tol = float(cfg.get("err_tol", 0.001))

    def solve(
        self,
        *,
        tcp_sid: int,
        arm_aids: list[int],
        pose_des: np.ndarray,
        position_only: bool = False,
    ) -> bool:
        T = pose_to_matrix(pose_des)
        pos_des, R_des = T[:3, 3], T[:3, :3]
        joint_ids = [self.model.actuator_trnid[a, 0] for a in arm_aids]
        qadr = [self.model.jnt_qposadr[j] for j in joint_ids]
        dof_ids = [self.model.jnt_dofadr[j] for j in joint_ids]
        rot_w = 0.0 if position_only else self.rot_weight
        n_task = 3 if position_only else 6

        for _ in range(self.iters):
            mujoco.mj_forward(self.model, self.data)
            pos = self.data.site_xpos[tcp_sid]
            R = self.data.site_xmat[tcp_sid].reshape(3, 3)
            err6 = _mat_to_6err(pos_des, R_des, pos, R)
            err6[:3] *= self.pos_weight
            err6[3:] *= rot_w
            err = err6[:n_task]
            if float(np.linalg.norm(err)) < self.err_tol:
                return True
            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, tcp_sid)
            J = jacp[:, dof_ids] if position_only else np.vstack([jacp[:, dof_ids], jacr[:, dof_ids]])
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + self.damping * np.eye(n_task), err)
            dq = np.clip(dq, -self.max_dq, self.max_dq)
            for a, qa, dqi in zip(arm_aids, qadr, dq):
                q_new = float(self.data.qpos[qa] + dqi)
                lo, hi = self.model.actuator_ctrlrange[a]
                if hi > lo:
                    q_new = float(np.clip(q_new, lo, hi))
                self.data.qpos[qa] = q_new
                self.data.ctrl[a] = q_new
        return False
