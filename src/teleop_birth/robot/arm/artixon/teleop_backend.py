"""物流双 Arm-6A 遥操后端：对接 teleop_birth SparseTarget + 粘性抓取。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional

import mujoco
import numpy as np

from teleop_birth.scene.plugins.physics import AntiPenetrationManager
from teleop_birth.robot.arm.artixon.backend import DualArtixOnBackend
from teleop_birth.robot.arm.artixon.collection.sticky import StickyGrasp
from teleop_birth.robot.arm.artixon.scene.props import PropPoseController

try:
    from teleop_birth.robot.base import BackendState, RobotBackend
    from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, SparseTarget
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "物流遥操需要 teleop_birth（在仓库根目录用 uv run 启动）"
    ) from exc


class LogisticsTeleopBackend(RobotBackend):
    """Pico → DualArmMapper → 本后端（物流场景 + sticky + TACTO）。"""

    def __init__(
        self,
        scene_cfg: dict[str, Any] | None = None,
        ik_cfg: dict[str, Any] | None = None,
        *,
        render: bool = True,
        sim_substeps: int = 4,
        sticky_attach_radius_m: float = 0.09,
        sticky_gripper_min: float = 0.25,
        position_ik_only: bool = False,
        tacto_cfg: dict[str, Any] | None = None,
        repo_root: Path | None = None,
    ) -> None:
        self._inner = DualArtixOnBackend(
            scene_cfg=scene_cfg,
            ik_cfg=ik_cfg,
            render=render,
            sim_substeps=sim_substeps,
        )
        self.position_ik_only = bool(position_ik_only)
        self.sticky_attach_radius_m = float(sticky_attach_radius_m)
        self.sticky_gripper_min = float(sticky_gripper_min)
        self._tacto_cfg = dict(tacto_cfg or {})
        self._repo_root = repo_root
        self.props: Optional[PropPoseController] = None
        self.anti: Optional[AntiPenetrationManager] = None
        self.tacto = None
        self._stickies: dict[str, StickyGrasp] = {}
        self._scene: dict[str, Any] = dict(scene_cfg or {})
        self._hold_pose: dict[str, np.ndarray] = {}

    @property
    def model(self):
        return self._inner.model

    @property
    def data(self):
        return self._inner.data

    @property
    def name_map(self) -> dict[str, Any]:
        return self._inner.name_map

    def connect(self) -> None:
        self._inner.connect()
        assert self._inner.model is not None and self._inner.data is not None
        nm = self._inner.name_map
        self.props = PropPoseController(self._inner.model, self._inner.data)
        self.props.discover(list(nm["prop_names"]))
        self._hold_pose = {
            "left": self._inner.tcp_pose("left"),
            "right": self._inner.tcp_pose("right"),
        }

        table_z = float(nm["table_z"])
        table_top = table_z + float((self._scene.get("table_size") or [0.48, 0.62, 0.02])[2])
        self.anti = AntiPenetrationManager(
            self._inner.model,
            self._inner.data,
            table_top_z=table_top,
            min_separation_m=0.08,
            soft_push=0.8,
            tcp_clearance_m=0.07,
        )
        pkg_half_z: dict[str, float] = {}
        for p in self._scene.get("packages") or []:
            if isinstance(p, dict) and p.get("name") and p.get("size"):
                pkg_half_z[str(p["name"])] = float(p["size"][2])
        for name in nm["prop_names"]:
            self.anti.register_prop(name, half_extent_z=pkg_half_z.get(name))
        for side in ("left", "right"):
            self.anti.register_tcp_site(nm["tcp_sites"][side])
        for pref in ("L_", "R_"):
            self.anti.register_finger_bodies(
                f"{pref}arm_gripper_left_link",
                f"{pref}arm_gripper_right_link",
                f"{pref}arm_gripper_base_link",
            )
        for key in ("in_conveyor", "out_conveyor"):
            c = dict(self._scene.get(key) or {})
            if not c.get("pos") or not c.get("size"):
                continue
            pos = [float(x) for x in c["pos"]]
            size = [float(x) for x in c["size"]]
            self.anti.register_support_plane(
                center_xy=(pos[0], pos[1]),
                half_xy=(size[0], size[1]),
                top_z=pos[2] + size[2],
            )

        for name in nm["prop_names"]:
            self._stickies[name] = StickyGrasp(
                self.props,
                self.anti,
                object_name=name,
                model=self._inner.model,
                data=self._inner.data,
                grasp_offset_xyz=(0.0, 0.0, 0.0),
                attach_radius_m=self.sticky_attach_radius_m,
                attach_gripper_min=self.sticky_gripper_min,
            )
        self.anti.step()

        # TACTO DIGIT 触觉（可选）
        self.tacto = None
        tc = self._tacto_cfg
        if bool(tc.get("enabled", False)):
            try:
                from teleop_birth.scene.plugins.sensor import MujocoTactoSensors

                self.tacto = MujocoTactoSensors(
                    self._inner.model,
                    self._inner.data,
                    MujocoTactoSensors.default_arm6a_specs(),
                    repo_root=self._repo_root,
                    width=int(tc.get("width", 120)),
                    height=int(tc.get("height", 160)),
                    enabled=True,
                    show_window=bool(tc.get("show_window", True)),
                    every_n=int(tc.get("every_n", 3)),
                )
                print(
                    f"[tacto] DIGIT 触觉已挂到四指垫 "
                    f"(show_window={bool(tc.get('show_window', True))})"
                )
            except Exception as exc:
                print(f"[tacto] 初始化失败，继续无触觉: {exc}")
                self.tacto = None

    def disconnect(self) -> None:
        if self.tacto is not None:
            try:
                self.tacto.close()
            except Exception:
                pass
            self.tacto = None
        self._stickies.clear()
        self.props = None
        self.anti = None
        self._inner.disconnect()

    def get_feedback(self) -> Dict[str, object]:
        return {
            EXTREMITY_LEFT_EE: self._inner.tcp_pose("left"),
            EXTREMITY_RIGHT_EE: self._inner.tcp_pose("right"),
        }

    def _goto_home(self, side: str) -> None:
        assert self._inner.model is not None and self._inner.data is not None
        home = list(self._inner.name_map["home_q"][side])
        for i, jn in enumerate(self._inner.name_map["arm_joints"][side]):
            jid = mujoco.mj_name2id(self._inner.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self._inner.data.qpos[self._inner.model.jnt_qposadr[jid]] = float(home[i])
            self._inner.data.ctrl[self._inner._arm_aids[side][i]] = float(home[i])
        self._inner.set_gripper(side, 0.0)
        mujoco.mj_forward(self._inner.model, self._inner.data)
        self._hold_pose[side] = self._inner.tcp_pose(side)

    def _update_sticky(self, side: str, ee: np.ndarray, gripper: float) -> None:
        """最近且满足条件的包裹粘住；已粘住的优先更新。"""
        held = [
            s for s in self._stickies.values()
            if s.attached and s.holding_side == side
        ]
        if held:
            held[0].update(side, ee, gripper)
            return
        if gripper < self.sticky_gripper_min:
            return
        # 选最近未占用包裹
        best: StickyGrasp | None = None
        best_d = 1e9
        for s in self._stickies.values():
            if s.attached:
                continue
            obj = s.props.get_pose(s.object_name)
            mid = s.pad_center(side, ee)
            d = float(np.linalg.norm(mid - obj[:3]))
            if d < best_d:
                best_d = d
                best = s
        if best is not None and best_d <= self.sticky_attach_radius_m:
            best.update(side, ee, gripper)

    def apply(self, target: SparseTarget) -> BackendState:
        meta = target.meta or {}
        if bool(meta.get("home_left")):
            self._goto_home("left")
            for s in self._stickies.values():
                if s.holding_side == "left":
                    s.force_release()
        if bool(meta.get("home_right")):
            self._goto_home("right")
            for s in self._stickies.values():
                if s.holding_side == "right":
                    s.force_release()

        # 未 clutch 时用 hold 位姿锁住，避免仿真漂移背离操作面
        if target.is_active(EXTREMITY_LEFT_EE) and EXTREMITY_LEFT_EE in target.extremities:
            left = np.asarray(target.extremities[EXTREMITY_LEFT_EE], dtype=np.float64)
            self._hold_pose["left"] = left.copy()
        else:
            left = self._hold_pose.get("left")
        if target.is_active(EXTREMITY_RIGHT_EE) and EXTREMITY_RIGHT_EE in target.extremities:
            right = np.asarray(target.extremities[EXTREMITY_RIGHT_EE], dtype=np.float64)
            self._hold_pose["right"] = right.copy()
        else:
            right = self._hold_pose.get("right")

        ees = self._inner.step_to_poses(
            left=left,
            right=right,
            left_gripper=float(target.left_gripper),
            right_gripper=float(target.right_gripper),
            position_only=self.position_ik_only,
        )

        self._update_sticky("left", ees["left"], float(target.left_gripper))
        self._update_sticky("right", ees["right"], float(target.right_gripper))
        if self.anti is not None:
            self.anti.step()

        tacto_hits = 0.0
        if self.tacto is not None:
            try:
                bundle = self.tacto.render()
                tacto_hits = float(sum(1 for v in bundle.contact.values() if v))
            except Exception:
                pass

        fb = self.get_feedback()
        return BackendState(
            extremities=fb,
            left_gripper=float(target.left_gripper),
            right_gripper=float(target.right_gripper),
            extra={
                "sticky": 1.0 if any(s.attached for s in self._stickies.values()) else 0.0,
                "tacto_hits": tacto_hits,
            },
        )
