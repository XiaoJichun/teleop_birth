"""双 Arm-6A 后端：左右 TCP 独立 IK + 夹爪。"""

from __future__ import annotations

from typing import Any, Optional

import mujoco
import numpy as np

from teleop_birth.robot.arm.artixon.builder import build_logistics_scene
from teleop_birth.robot.arm.artixon.ik_dls import DlsIK, matrix_to_pose


class DualArtixOnBackend:
    def __init__(
        self,
        scene_cfg: dict[str, Any] | None = None,
        ik_cfg: dict[str, Any] | None = None,
        *,
        render: bool = False,
        sim_substeps: int = 5,
    ) -> None:
        self.scene_cfg = dict(scene_cfg or {})
        self.ik_cfg = dict(ik_cfg or {})
        self.render = bool(render)
        self.sim_substeps = max(1, int(sim_substeps))
        self.model: Optional[mujoco.MjModel] = None
        self.data: Optional[mujoco.MjData] = None
        self.name_map: dict[str, Any] = {}
        self.ik: Optional[DlsIK] = None
        self._viewer = None
        self._tcp: dict[str, int] = {}
        self._arm_aids: dict[str, list[int]] = {}
        self._grip_aids: dict[str, list[int]] = {}
        self._grip_qadr: dict[str, tuple[int, int]] = {}

    def connect(self) -> None:
        self.model, self.name_map = build_logistics_scene(self.scene_cfg)
        self.data = mujoco.MjData(self.model)
        self.ik = DlsIK(self.model, self.data, self.ik_cfg)
        for side in ("left", "right"):
            self._tcp[side] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SITE, self.name_map["tcp_sites"][side]
            )
            self._arm_aids[side] = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                for n in self.name_map["arm_actuators"][side]
            ]
            self._grip_aids[side] = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
                for n in self.name_map["grip_actuators"][side]
            ]
            pref = "L_" if side == "left" else "R_"
            jl = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{pref}arm_gripper_left_joint")
            jr = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"{pref}arm_gripper_right_joint")
            self._grip_qadr[side] = (int(self.model.jnt_qposadr[jl]), int(self.model.jnt_qposadr[jr]))
        self.reset_home()
        if self.render:
            try:
                from mujoco import viewer as mj_viewer

                self._viewer = mj_viewer.launch_passive(self.model, self.data)
                self._setup_region_overlay()
            except Exception as exc:  # pragma: no cover
                print(f"[artixon] viewer 启动失败: {exc}")
                self._viewer = None

    def disconnect(self) -> None:
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None
        self.model = None
        self.data = None

    def reset_home(self) -> None:
        assert self.model is not None and self.data is not None
        for side in ("left", "right"):
            home = list(self.name_map["home_q"][side])
            for i, jn in enumerate(self.name_map["arm_joints"][side]):
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                self.data.qpos[self.model.jnt_qposadr[jid]] = float(home[i])
                self.data.ctrl[self._arm_aids[side][i]] = float(home[i])
            self.set_gripper(side, 0.0)
        mujoco.mj_forward(self.model, self.data)

    def set_gripper(self, side: str, closed: float) -> None:
        """闭合量 0=全开 … 1=近全合。同时写 ctrl+qpos。"""
        assert self.model is not None and self.data is not None
        g = float(np.clip(closed, 0.0, 1.0))
        open_q = 0.04875
        close_q = 0.006
        left = open_q + g * (close_q - open_q)
        right = -left
        self._apply_gripper_q(side, left, right)

    def set_gripper_half_gap(self, side: str, half_gap_m: float) -> None:
        """按半开口设置夹爪（指关节到中线距离），用于贴盒外侧不插进。"""
        assert self.model is not None and self.data is not None
        left = float(np.clip(half_gap_m, 0.008, 0.04875))
        self._apply_gripper_q(side, left, -left)

    def _apply_gripper_q(self, side: str, left: float, right: float) -> None:
        la, ra = self._grip_aids[side]
        qa, qb = self._grip_qadr[side]
        self.data.ctrl[la] = left
        self.data.ctrl[ra] = right
        self.data.qpos[qa] = left
        self.data.qpos[qb] = right

    def tcp_pose(self, side: str) -> np.ndarray:
        assert self.model is not None and self.data is not None
        mujoco.mj_forward(self.model, self.data)
        sid = self._tcp[side]
        pos = self.data.site_xpos[sid].copy()
        R = self.data.site_xmat[sid].reshape(3, 3).copy()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = pos
        return matrix_to_pose(T)

    def step_to_poses(
        self,
        *,
        left: np.ndarray | None = None,
        right: np.ndarray | None = None,
        left_gripper: float | None = None,
        right_gripper: float | None = None,
        position_only: bool = False,
    ) -> dict[str, np.ndarray]:
        assert self.model is not None and self.data is not None and self.ik is not None
        if left is not None:
            self.ik.solve(
                tcp_sid=self._tcp["left"],
                arm_aids=self._arm_aids["left"],
                pose_des=np.asarray(left, dtype=np.float64),
                position_only=position_only,
            )
        if right is not None:
            self.ik.solve(
                tcp_sid=self._tcp["right"],
                arm_aids=self._arm_aids["right"],
                pose_des=np.asarray(right, dtype=np.float64),
                position_only=position_only,
            )
        if left_gripper is not None:
            self.set_gripper("left", left_gripper)
        if right_gripper is not None:
            self.set_gripper("right", right_gripper)
        for _ in range(self.sim_substeps):
            mujoco.mj_step(self.model, self.data)
            # 每步锁住夹爪开度，避免桌面/自碰把指撑开
            if left_gripper is not None:
                self.set_gripper("left", left_gripper)
            if right_gripper is not None:
                self.set_gripper("right", right_gripper)
        if self._viewer is not None:
            try:
                self._apply_region_overlay()
                self._viewer.sync()
            except Exception:
                pass
        return {"left": self.tcp_pose("left"), "right": self.tcp_pose("right")}

    def _setup_region_overlay(self) -> None:
        try:
            from teleop_birth.ui.session.base_frame_viz import get_base_frame_overlay
            from teleop_birth.ui.session.region_viz import attach_region_from_env_or_dict

            ov = attach_region_from_env_or_dict(
                {"randomize_region": self.scene_cfg.get("randomize_region")}
                if self.scene_cfg.get("randomize_region")
                else None
            )
            if ov is None:
                ov = attach_region_from_env_or_dict()
            get_base_frame_overlay().set_from_scene(self.scene_cfg)
            get_base_frame_overlay().set_visible(True)
            print(
                "  [base] 已显示桌面接触 base（青）与转换后 URDF base（品红）；"
                "RGB=XYZ 轴"
            )
            if ov is not None:
                print("  [region] 已挂载随机化区域绿盒（MuJoCo）")
        except Exception as exc:  # noqa: BLE001
            print(f"  [region/base] 可视化跳过: {exc}")

    def _apply_region_overlay(self) -> None:
        try:
            from teleop_birth.ui.session.base_frame_viz import (
                apply_viewer_overlays,
                get_base_frame_overlay,
            )

            bov = get_base_frame_overlay()
            if not bov.has_frames():
                bov.set_from_scene(self.scene_cfg)
            apply_viewer_overlays(self._viewer)
        except Exception:
            try:
                from teleop_birth.ui.session.region_viz import get_region_overlay

                get_region_overlay().apply(self._viewer)
            except Exception:
                pass


# 兼容旧单臂名
ArtixOnArmBackend = DualArtixOnBackend
