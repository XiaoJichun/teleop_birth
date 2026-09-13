"""MuJoCo dual UR5e backend — IK / EE / scene 全部来自配置字典。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Optional

import mujoco
import numpy as np

from teleop_birth.ik import ArmIKHandles, build_arm_ik_solver
from teleop_birth.ik.frames import site_pose_T
from teleop_birth.embodiment.base import BackendState, RobotBackend
from teleop_birth.ee import DexHandStub, EECommand, EndEffector, ParallelGripper
from teleop_birth.schema import (
    EXTREMITY_LEFT_EE,
    EXTREMITY_RIGHT_EE,
    SparseTarget,
    matrix_to_pose,
)
from teleop_birth.sim.dual_ur5e_builder import actuator_ids, build_dual_ur5e_2f85, site_id


def _require(cfg: dict[str, Any], key: str) -> Any:
    if key not in cfg:
        raise KeyError(f"配置缺少字段: {key}")
    return cfg[key]


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

        self.sim_substeps = max(1, int(sim_substeps))
        self.viewer_sync_every = max(1, int(self.ik_cfg.get("viewer_sync_every", 1)))
        self._viewer_tick = 0
        self.position_ik_only = bool(self.ik_cfg.get("position_only", True))
        self.gripper_force_scale = float(self.ee_cfg.get("force_scale", 1.0))
        self.gripper_kp_scale = float(self.ee_cfg.get("kp_scale", 1.0))
        self.gripper_kv_scale = float(self.ee_cfg.get("kv_scale", 1.0))
        self.gripper_extra_steps = max(0, int(self.ee_cfg.get("extra_sim_steps", 0)))

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
        self._left_handles: Optional[ArmIKHandles] = None
        self._right_handles: Optional[ArmIKHandles] = None
        self._ik = None
        self.left_ee: Optional[EndEffector] = None
        self.right_ee: Optional[EndEffector] = None

    def connect(self) -> None:
        model, name_map = build_dual_ur5e_2f85(self.menagerie, self.scene_cfg)
        self.model = model
        self.data = mujoco.MjData(model)
        self.name_map = name_map

        left_prefix = str(_require(self.scene_cfg, "left_prefix"))
        right_prefix = str(_require(self.scene_cfg, "right_prefix"))

        self._left_arm_aids = actuator_ids(model, name_map["left_arm_actuators"])
        self._right_arm_aids = actuator_ids(model, name_map["right_arm_actuators"])
        self._left_grip_aid = actuator_ids(model, [name_map["left_gripper_actuator"]])[0]
        self._right_grip_aid = actuator_ids(model, [name_map["right_gripper_actuator"]])[0]
        self._boost_gripper_actuators()
        self._left_sid = site_id(model, name_map["left_tcp_site"])
        self._right_sid = site_id(model, name_map["right_tcp_site"])
        left_flange = site_id(model, f"{left_prefix}attachment_site")
        right_flange = site_id(model, f"{right_prefix}attachment_site")
        left_base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{left_prefix}base")
        right_base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{right_prefix}base")
        if left_base < 0 or right_base < 0:
            raise KeyError("未找到 L_base / R_base body，无法做基坐标系 IK")

        for aid, q in zip(self._left_arm_aids, name_map["left_home"]):
            self.data.ctrl[aid] = q
            j = model.actuator_trnid[aid, 0]
            self.data.qpos[model.jnt_qposadr[j]] = q
        for aid, q in zip(self._right_arm_aids, name_map["right_home"]):
            self.data.ctrl[aid] = q
            j = model.actuator_trnid[aid, 0]
            self.data.qpos[model.jnt_qposadr[j]] = q
        self.data.ctrl[self._left_grip_aid] = float(_require(self.ee_cfg, "ctrl_open"))
        self.data.ctrl[self._right_grip_aid] = float(_require(self.ee_cfg, "ctrl_open"))
        mujoco.mj_forward(model, self.data)
        for _ in range(int(name_map["settle_steps"])):
            mujoco.mj_step(model, self.data)

        mujoco.mj_forward(model, self.data)
        self._left_handles = ArmIKHandles(
            name="left",
            arm_aids=self._left_arm_aids,
            tcp_sid=self._left_sid,
            flange_sid=left_flange,
            base_bid=left_base,
            T_flange_tcp=np.linalg.inv(site_pose_T(self.data, left_flange))
            @ site_pose_T(self.data, self._left_sid),
        )
        self._right_handles = ArmIKHandles(
            name="right",
            arm_aids=self._right_arm_aids,
            tcp_sid=self._right_sid,
            flange_sid=right_flange,
            base_bid=right_base,
            T_flange_tcp=np.linalg.inv(site_pose_T(self.data, right_flange))
            @ site_pose_T(self.data, self._right_sid),
        )
        self._ik = build_arm_ik_solver(self.ik_cfg, model=model, data=self.data)
        print(
            f"[ik] solver={self.ik_cfg.get('solver', 'dls')} "
            f"fallback={self.ik_cfg.get('fallback', 'dls')}"
        )

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

    def _boost_gripper_actuators(self) -> None:
        assert self.model is not None
        for aid in (self._left_grip_aid, self._right_grip_aid):
            lo, hi = self.model.actuator_forcerange[aid]
            fs = max(1.0, self.gripper_force_scale)
            self.model.actuator_forcerange[aid, 0] = float(lo) * fs
            self.model.actuator_forcerange[aid, 1] = float(hi) * fs
            bp = self.model.actuator_biasprm[aid]
            if abs(bp[1]) > 1e-9:
                bp[1] = float(bp[1]) * self.gripper_kp_scale
            if abs(bp[2]) > 1e-9:
                bp[2] = float(bp[2]) * self.gripper_kv_scale
            gp = self.model.actuator_gainprm[aid]
            if abs(gp[0]) > 1e-12:
                gp[0] = float(gp[0]) * self.gripper_kp_scale

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
        assert self._ik is not None and self._left_handles and self._right_handles
        meta = target.meta or {}
        if bool(meta.get("home_left")):
            self._goto_home("left")
        if bool(meta.get("home_right")):
            self._goto_home("right")

        if target.is_active(EXTREMITY_LEFT_EE) and EXTREMITY_LEFT_EE in target.extremities:
            self._ik.solve(
                self._left_handles,
                target.extremities[EXTREMITY_LEFT_EE],
                position_only=self.position_ik_only,
            )
        if target.is_active(EXTREMITY_RIGHT_EE) and EXTREMITY_RIGHT_EE in target.extremities:
            self._ik.solve(
                self._right_handles,
                target.extremities[EXTREMITY_RIGHT_EE],
                position_only=self.position_ik_only,
            )

        assert self.left_ee is not None and self.right_ee is not None
        self.left_ee.apply(EECommand(gripper_openness=target.left_gripper))
        self.right_ee.apply(EECommand(gripper_openness=target.right_gripper))
        steps = self.sim_substeps + self.gripper_extra_steps
        for _ in range(steps):
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
