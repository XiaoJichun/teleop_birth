"""Shared SE(3) helpers for IK adapters."""

from __future__ import annotations

import mujoco
import numpy as np

from teleop_birth.teleop.schema import pose_to_matrix


def body_pose_T(_model: mujoco.MjModel, data: mujoco.MjData, bid: int) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = data.xpos[bid].copy()
    T[:3, :3] = data.xmat[bid].reshape(3, 3).copy()
    return T


def site_pose_T(data: mujoco.MjData, sid: int) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = data.site_xpos[sid].copy()
    T[:3, :3] = data.site_xmat[sid].reshape(3, 3).copy()
    return T


def read_arm_q(model: mujoco.MjModel, data: mujoco.MjData, arm_aids: list[int]) -> np.ndarray:
    qs = []
    for aid in arm_aids:
        j = model.actuator_trnid[aid, 0]
        qa = model.jnt_qposadr[j]
        qs.append(float(data.qpos[qa]))
    return np.asarray(qs, dtype=np.float64)


def write_arm_q(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    arm_aids: list[int],
    q: np.ndarray,
) -> None:
    for aid, qi in zip(arm_aids, q):
        j = model.actuator_trnid[aid, 0]
        qa = model.jnt_qposadr[j]
        q_new = float(qi)
        lo, hi = model.actuator_ctrlrange[aid]
        if hi > lo:
            q_new = float(np.clip(q_new, lo, hi))
        data.qpos[qa] = q_new
        data.ctrl[aid] = q_new


def tcp_des_to_flange_in_base(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base_bid: int,
    flange_sid: int,
    tcp_sid: int,
    T_flange_tcp: np.ndarray,
    pose_des: np.ndarray,
    position_only: bool,
) -> np.ndarray:
    """Desired TCP (world, 7D) → flange pose in arm base frame (4x4) for analytical IK."""
    mujoco.mj_forward(model, data)
    T_w_tcp = pose_to_matrix(pose_des)
    if position_only:
        # Keep current TCP orientation; only drive position
        T_cur = site_pose_T(data, tcp_sid)
        T_w_tcp = T_cur.copy()
        T_w_tcp[:3, 3] = pose_des[:3]

    T_tcp_flange = np.linalg.inv(T_flange_tcp)
    T_w_flange = T_w_tcp @ T_tcp_flange
    T_w_base = body_pose_T(model, data, base_bid)
    return np.linalg.inv(T_w_base) @ T_w_flange


def mat_to_6err(pos_des: np.ndarray, R_des: np.ndarray, pos: np.ndarray, R: np.ndarray) -> np.ndarray:
    dp = pos_des - pos
    R_err = R_des @ R.T
    dw = 0.5 * np.array(
        [
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ]
    )
    return np.concatenate([dp, dw])
