"""MuJoCo dual UR5e backend — IK / EE / scene 全部来自配置字典。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import mujoco
import numpy as np

from birth_teleop.backend.robots import BackendState, RobotBackend
from birth_teleop.ee import DexHandStub, EECommand, EndEffector, ParallelGripper
from birth_teleop.schema import (
    EXTREMITY_LEFT_EE,
    EXTREMITY_RIGHT_EE,
    SparseTarget,
    matrix_to_pose,
    pose_to_matrix,
)
from birth_teleop.sim.dual_ur5e_builder import actuator_ids, build_dual_ur5e_2f85, site_id


def _require(cfg: dict[str, Any], key: str) -> Any:
    if key not in cfg:
        raise KeyError(f"配置缺少字段: {key}")
    return cfg[key]


def _mat_to_6err(pos_des: np.ndarray, R_des: np.ndarray, pos: np.ndarray, R: np.ndarray) -> np.ndarray:
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


def _pose_to_pos_R(pose: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    T = pose_to_matrix(pose)
    return T[:3, 3].copy(), T[:3, :3].copy()


class MujocoDualURBackend(RobotBackend):
    def __init__(
        self,
        *,
        menagerie: str | Path,
        scene_cfg: dict[str, Any],
        ik_cfg: dict[str, Any],
        ee_cfg: dict[str, Any],
        render: bool,
        sim_substeps: int = 1,
        left_ee: Optional[EndEffector] = None,
        right_ee: Optional[EndEffector] = None,
    ) -> None:
        self.menagerie = Path(menagerie)
        self.scene_cfg = dict(scene_cfg)
        self.ik_cfg = dict(ik_cfg)
        self.ee_cfg = dict(ee_cfg)
        self.render = bool(render)
        self._left_ee_override = left_ee
        self._right_ee_override = right_ee

        self.ik_iters = int(_require(self.ik_cfg, "iters"))
        self.damping = float(_require(self.ik_cfg, "damping"))
        self.pos_weight = float(_require(self.ik_cfg, "pos_weight"))
        self.rot_weight = float(_require(self.ik_cfg, "rot_weight"))
        self.max_dq = float(_require(self.ik_cfg, "max_dq"))
        self.err_tol = float(_require(self.ik_cfg, "err_tol"))
        self.sim_substeps = max(1, int(sim_substeps))
        self.viewer_sync_every = max(1, int(self.ik_cfg.get("viewer_sync_every", 1)))
        self._viewer_tick = 0
        self.position_ik_only = bool(self.ik_cfg.get("position_only", True))

        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.name_map: dict = {}
        self._viewer = None
        self._left_arm_aids: List[int] = []
        self._right_arm_aids: List[int] = []
        self._left_grip_aid = -1
        self._right_grip_aid = -1
        self._left_sid = -1
        self._right_sid = -1
        self.left_ee: Optional[EndEffector] = None
        self.right_ee: Optional[EndEffector] = None

    def connect(self) -> None:
        model, name_map = build_dual_ur5e_2f85(self.menagerie, self.scene_cfg)
        self.model = model
        self.data = mujoco.MjData(model)
        self.name_map = name_map

        self._left_arm_aids = actuator_ids(model, name_map["left_arm_actuators"])
        self._right_arm_aids = actuator_ids(model, name_map["right_arm_actuators"])
        self._left_grip_aid = actuator_ids(model, [name_map["left_gripper_actuator"]])[0]
        self._right_grip_aid = actuator_ids(model, [name_map["right_gripper_actuator"]])[0]
        self._left_sid = site_id(model, name_map["left_tcp_site"])
        self._right_sid = site_id(model, name_map["right_tcp_site"])

        for aid, q in zip(self._left_arm_aids, name_map["left_home"]):
            self.data.ctrl[aid] = q
        for aid, q in zip(self._right_arm_aids, name_map["right_home"]):
            self.data.ctrl[aid] = q
        self.data.ctrl[self._left_grip_aid] = float(_require(self.ee_cfg, "ctrl_open"))
        self.data.ctrl[self._right_grip_aid] = float(_require(self.ee_cfg, "ctrl_open"))
        mujoco.mj_forward(model, self.data)
        for _ in range(int(name_map["settle_steps"])):
            mujoco.mj_step(model, self.data)

        self.left_ee = self._left_ee_override or self._make_ee(self._left_grip_aid)
        self.right_ee = self._right_ee_override or self._make_ee(self._right_grip_aid)
        self.left_ee.reset()
        self.right_ee.reset()

        if self.render:
            try:
                from mujoco import viewer as mj_viewer

                self._viewer = mj_viewer.launch_passive(model, self.data)
            except Exception as exc:
                print(f"[warn] viewer unavailable ({exc}); running headless")
                self._viewer = None

    def _make_ee(self, aid: int) -> EndEffector:
        kind = str(_require(self.ee_cfg, "type")).lower()

        def set_ctrl(v: float, _aid: int = aid) -> None:
            assert self.data is not None
            self.data.ctrl[_aid] = v

        def get_openness(_aid: int = aid) -> float:
            assert self.data is not None
            closed = float(_require(self.ee_cfg, "ctrl_closed"))
            open_v = float(_require(self.ee_cfg, "ctrl_open"))
            span = closed - open_v
            if abs(span) < 1e-9:
                return 0.0
            return float((self.data.ctrl[_aid] - open_v) / span)

        if kind in ("gripper", "parallel", "2f85", "robotiq"):
            return ParallelGripper(
                set_ctrl=set_ctrl,
                get_openness=get_openness,
                ctrl_open=float(_require(self.ee_cfg, "ctrl_open")),
                ctrl_closed=float(_require(self.ee_cfg, "ctrl_closed")),
            )
        if kind in ("dex", "dex_hand", "dexterous", "hand"):
            n_fingers = int(_require(self.ee_cfg, "n_fingers"))
            proxy = bool(_require(self.ee_cfg, "proxy_use_gripper_actuator"))
            if proxy:
                closed = float(_require(self.ee_cfg, "ctrl_closed"))
                open_v = float(_require(self.ee_cfg, "ctrl_open"))

                def set_joints(fingers: dict[str, float]) -> None:
                    val = float(next(iter(fingers.values()), 0.0))
                    set_ctrl(open_v + val * (closed - open_v))

                return DexHandStub(n_fingers=n_fingers, set_joint_ctrls=set_joints)
            return DexHandStub(n_fingers=n_fingers)
        raise ValueError(f"未知 end_effector.type: {kind}")

    def disconnect(self) -> None:
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None

    def get_feedback(self) -> Dict[str, np.ndarray]:
        assert self.model is not None and self.data is not None
        mujoco.mj_forward(self.model, self.data)
        return {
            EXTREMITY_LEFT_EE: self._site_pose(self._left_sid),
            EXTREMITY_RIGHT_EE: self._site_pose(self._right_sid),
        }

    def _site_pose(self, sid: int) -> np.ndarray:
        assert self.data is not None
        pos = self.data.site_xpos[sid].copy()
        R = self.data.site_xmat[sid].reshape(3, 3).copy()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        return matrix_to_pose(T)

    def apply(self, target: SparseTarget) -> BackendState:
        assert self.model is not None and self.data is not None
        meta = target.meta or {}
        if bool(meta.get("home_left")):
            self._goto_home("left")
        if bool(meta.get("home_right")):
            self._goto_home("right")

        if target.is_active(EXTREMITY_LEFT_EE) and EXTREMITY_LEFT_EE in target.extremities:
            self._ik_arm(self._left_arm_aids, self._left_sid, target.extremities[EXTREMITY_LEFT_EE])
        if target.is_active(EXTREMITY_RIGHT_EE) and EXTREMITY_RIGHT_EE in target.extremities:
            self._ik_arm(self._right_arm_aids, self._right_sid, target.extremities[EXTREMITY_RIGHT_EE])

        assert self.left_ee is not None and self.right_ee is not None
        self.left_ee.apply(EECommand(gripper_openness=target.left_gripper))
        self.right_ee.apply(EECommand(gripper_openness=target.right_gripper))
        for _ in range(self.sim_substeps):
            mujoco.mj_step(self.model, self.data)
        if self._viewer is not None:
            self._viewer_tick += 1
            if self._viewer_tick % self.viewer_sync_every == 0:
                self._viewer.sync()

        fb = self.get_feedback()
        return BackendState(
            extremities=fb,
            left_gripper=self.left_ee.read().gripper_openness,
            right_gripper=self.right_ee.read().gripper_openness,
            extra={
                "ee_left_is_gripper": 1.0 if self.left_ee.kind == "gripper" else 0.0,
                "ee_right_is_gripper": 1.0 if self.right_ee.kind == "gripper" else 0.0,
            },
        )

    def _goto_home(self, side: str) -> None:
        """Snap one arm to configured home joint angles and settle briefly."""
        assert self.model is not None and self.data is not None
        if side == "left":
            aids = self._left_arm_aids
            home = list(self.name_map["left_home"])
            grip_aid = self._left_grip_aid
        else:
            aids = self._right_arm_aids
            home = list(self.name_map["right_home"])
            grip_aid = self._right_grip_aid
        for aid, q in zip(aids, home):
            j = self.model.actuator_trnid[aid, 0]
            qa = self.model.jnt_qposadr[j]
            self.data.qpos[qa] = float(q)
            self.data.ctrl[aid] = float(q)
        self.data.ctrl[grip_aid] = float(_require(self.ee_cfg, "ctrl_open"))
        mujoco.mj_forward(self.model, self.data)
        for _ in range(30):
            mujoco.mj_step(self.model, self.data)

    def _ik_arm(self, arm_aids: List[int], sid: int, pose_des: np.ndarray) -> None:
        assert self.model is not None and self.data is not None
        pos_des, R_des = _pose_to_pos_R(pose_des)
        joint_ids = [self.model.actuator_trnid[a, 0] for a in arm_aids]
        qadr = [self.model.jnt_qposadr[j] for j in joint_ids]
        dof_ids = [self.model.jnt_dofadr[j] for j in joint_ids]
        rot_w = 0.0 if self.position_ik_only else self.rot_weight
        n_task = 3 if self.position_ik_only else 6

        for _ in range(self.ik_iters):
            mujoco.mj_forward(self.model, self.data)
            pos = self.data.site_xpos[sid]
            R = self.data.site_xmat[sid].reshape(3, 3)
            err6 = _mat_to_6err(pos_des, R_des, pos, R)
            err6[:3] *= self.pos_weight
            err6[3:] *= rot_w
            err = err6[:n_task]
            if np.linalg.norm(err) < self.err_tol:
                break

            jacp = np.zeros((3, self.model.nv))
            jacr = np.zeros((3, self.model.nv))
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, sid)
            if self.position_ik_only:
                J = jacp[:, dof_ids]
            else:
                J = np.vstack([jacp[:, dof_ids], jacr[:, dof_ids]])
            # Damped least squares
            JT = J.T
            dq = JT @ np.linalg.solve(J @ JT + self.damping * np.eye(n_task), err)
            dq = np.clip(dq, -self.max_dq, self.max_dq)
            for a, qa, dqi in zip(arm_aids, qadr, dq):
                q_new = float(self.data.qpos[qa] + dqi)
                lo, hi = self.model.actuator_ctrlrange[a]
                if hi > lo:
                    q_new = float(np.clip(q_new, lo, hi))
                # Critical: update qpos so next iter FK/Jac is consistent
                self.data.qpos[qa] = q_new
                self.data.ctrl[a] = q_new
