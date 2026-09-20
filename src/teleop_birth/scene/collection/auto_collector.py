"""自动化采集：布局随机 → 语义 pick/place → 写 MCAP。

流程对齐智元 Genie Sim autocollect（任务 stages + 布局扰动 + episode 落盘），
实现适配本仓库 MuJoCo 双臂后端。
参考: https://github.com/AgibotTech/genie_sim · https://agibot-world.com/genie-sim
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Optional
import time

import mujoco
import numpy as np
from omegaconf import OmegaConf

from teleop_birth.robot.sim.mujoco_dual_ur import MujocoDualURBackend
from teleop_birth.scene.collection.motion import ScriptedEEMotion
from teleop_birth.scene.collection.grasp_geometry import (
    DEFAULT_PAD_OFFSET_XYZ,
    top_down_quat_xyzw,
)
from teleop_birth.scene.collection.sticky_grasp import StickyGrasp
from teleop_birth.scene.placement.pose_controller import PropPoseController
from teleop_birth.scene.placement.randomizer import LayoutRandomizer
from teleop_birth.scene.props.catalog import PropCatalog
from teleop_birth.scene.recording.camera import MultiCameraCapture, MultiCameraConfig
from teleop_birth.scene.recording.calibration import (
    CameraCalibration,
    apply_calibration_to_config,
)
from teleop_birth.scene.recording.mcap_writer import McapEpisodeWriter, TopicMap
from teleop_birth.scene.tasks.base import TaskSpec
from teleop_birth.scene.tasks.pick_place import SemanticPickPlaceTask
from teleop_birth.scene.config import CollectConfig
from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, SparseTarget


@dataclass
class EpisodeResult:
    episode: int
    path: Path
    success: bool
    layout: dict[str, Any]
    steps: int


class AutoCollector:
    """语义自动化数据采集器（类封装，便于移植；参数来自 configs/scene）。"""

    def __init__(
        self,
        project_root: Path,
        *,
        task_spec: TaskSpec,
        output_dir: Path | None = None,
        render: bool = False,
        episodes: int | None = None,
        seed: int | None = None,
        arm: str = "right",
        steps_per_segment: int = 35,
        sticky_grasp: bool = True,
        camera_cfg: MultiCameraConfig | None = None,
        topics: TopicMap | None = None,
        pace_s: float = 0.0,
        hold_steps: int = 0,
        backend_rel: str = "device/backend/mujoco_dual_ur.yaml",
        ik_rel: str = "algorithm/ik/dls.yaml",
        ee_rel: str = "device/end_effector/gripper.yaml",
        paths_rel: str = "device/paths/default.yaml",
        calibrate_before_collect: bool = True,
        calibration_override: str = "scene/calibration/override.yaml",
        calibration_file: str = "",
        randomize_region: dict | None = None,
        randomize_props: list[str] | None = None,
        dataset_format: str = "mcap",
    ) -> None:
        self.project_root = Path(project_root)
        self.task_spec = task_spec
        self.output_dir = Path(output_dir or (self.project_root / "logs" / "mcap"))
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.render = bool(render)
        self.episodes = int(episodes if episodes is not None else task_spec.episodes)
        self.seed = int(seed if seed is not None else task_spec.seed)
        self.arm = str(arm)
        self.steps_per_segment = int(steps_per_segment)
        self.sticky_grasp_enabled = bool(sticky_grasp)
        self.camera_cfg = camera_cfg or MultiCameraConfig.from_recording_dict({})
        self.topics = topics or TopicMap()
        self.pace_s = float(max(0.0, pace_s))
        self.hold_steps = max(0, int(hold_steps))
        self.backend_rel = backend_rel
        self.ik_rel = ik_rel
        self.ee_rel = ee_rel
        self.paths_rel = paths_rel
        self.calibrate_before_collect = bool(calibrate_before_collect)
        self.calibration_override = str(calibration_override or "")
        self.calibration_file = str(calibration_file or "")
        self.randomize_region = dict(randomize_region) if randomize_region else None
        self.randomize_props = list(randomize_props or [])
        self.dataset_format = str(dataset_format or "mcap")
        self.calibration: dict[str, Any] | None = None
        self.session_dir: Path | None = None

        self.catalog = PropCatalog(self.project_root / "configs" / "scene" / "props")
        objects = list(task_spec.objects) or ["red_cube", "blue_plate"]
        if self.randomize_props:
            # 会话向导选中的物体优先；仍保留任务物体以便抓取逻辑
            merged = list(dict.fromkeys([*self.randomize_props, *objects]))
            objects = merged
        grasp = objects[0]
        place = objects[1] if len(objects) > 1 else objects[0]
        for stage in task_spec.stages:
            if stage.action == "pick" and stage.active_object:
                grasp = stage.active_object
            if stage.action == "place":
                if stage.active_object:
                    grasp = stage.active_object
                if stage.passive_object:
                    place = stage.passive_object
        self.task = SemanticPickPlaceTask(
            self.catalog, grasp_object=grasp, place_object=place, arm=self.arm
        )
        self.prop_specs = self.catalog.resolve_many(objects)

        self.backend: Optional[MujocoDualURBackend] = None
        self.props: Optional[PropPoseController] = None
        self.randomizer: Optional[LayoutRandomizer] = None
        self.camera: Optional[MultiCameraCapture] = None

    @classmethod
    def from_config(cls, project_root: Path, cfg: "CollectConfig") -> "AutoCollector":
        task_path = Path(project_root) / "configs" / "scene" / "tasks" / f"{cfg.task}.yaml"
        if not task_path.is_file():
            raise FileNotFoundError(f"任务配置不存在: {task_path}")
        task_data = OmegaConf.to_container(OmegaConf.load(task_path), resolve=True)
        assert isinstance(task_data, dict)
        task_spec = TaskSpec.from_dict(task_data)

        recording_rel = str(getattr(cfg, "recording", None) or "scene/recording.yaml")
        recording_path = Path(project_root) / "configs" / recording_rel
        recording: dict = {}
        if recording_path.is_file():
            loaded = OmegaConf.to_container(OmegaConf.load(recording_path), resolve=True)
            assert isinstance(loaded, dict)
            recording = loaded
        cam = MultiCameraConfig.from_recording_dict(recording)
        scene_rel = str(getattr(cfg, "scene", "") or "")
        if scene_rel:
            scene_path = Path(project_root) / "configs" / scene_rel
            if not scene_path.is_file():
                scene_path = Path(project_root) / scene_rel
            if scene_path.is_file():
                scene_cfg = OmegaConf.to_container(OmegaConf.load(scene_path), resolve=True)
                if isinstance(scene_cfg, dict):
                    cam = MultiCameraConfig.from_scene(
                        project_root, scene_cfg, recording=recording
                    )
        # cli 可覆盖 enabled
        if hasattr(cfg, "camera_enabled"):
            cam.enabled = bool(cfg.camera_enabled) and cam.enabled
        topics = TopicMap.from_dict(dict(recording.get("topics") or {}))
        return cls(
            project_root,
            task_spec=task_spec,
            output_dir=Path(project_root) / cfg.output_dir,
            render=cfg.render,
            episodes=cfg.episodes,
            seed=cfg.seed,
            arm=cfg.arm,
            steps_per_segment=cfg.steps_per_segment,
            sticky_grasp=cfg.sticky_grasp,
            camera_cfg=cam,
            topics=topics,
            pace_s=cfg.pace_s,
            hold_steps=int(getattr(cfg, "hold_steps", 0)),
            backend_rel=cfg.backend,
            ik_rel=cfg.ik,
            ee_rel=cfg.end_effector,
            paths_rel=cfg.paths,
            calibrate_before_collect=bool(getattr(cfg, "calibrate_before_collect", True)),
            calibration_override=str(
                getattr(cfg, "calibration_override", None) or "scene/calibration/override.yaml"
            ),
            calibration_file=str(getattr(cfg, "calibration_file", None) or ""),
            randomize_region=getattr(cfg, "randomize_region", None),
            randomize_props=list(getattr(cfg, "randomize_props", None) or []),
            dataset_format=str(getattr(cfg, "dataset_format", None) or "mcap"),
        )

    def _build_backend(self) -> MujocoDualURBackend:
        backend_cfg = OmegaConf.to_container(
            OmegaConf.load(self.project_root / "configs" / self.backend_rel),
            resolve=True,
        )
        ik_cfg = OmegaConf.to_container(
            OmegaConf.load(self.project_root / "configs" / self.ik_rel),
            resolve=True,
        )
        ee_cfg = OmegaConf.to_container(
            OmegaConf.load(self.project_root / "configs" / self.ee_rel),
            resolve=True,
        )
        paths = OmegaConf.to_container(
            OmegaConf.load(self.project_root / "configs" / self.paths_rel),
            resolve=True,
        )
        assert isinstance(backend_cfg, dict) and isinstance(ik_cfg, dict)
        assert isinstance(ee_cfg, dict) and isinstance(paths, dict)

        scene = dict(backend_cfg.get("scene") or {})
        scene["props"] = [p.to_dict() for p in self.prop_specs]
        scene["project_root"] = str(self.project_root)
        # 场景 YAML 中的夹爪 id → gripper_xml
        gid = ""
        for g in scene.get("grippers") or []:
            if isinstance(g, dict) and g.get("id"):
                gid = str(g["id"])
                break
        if not gid:
            gid = str(scene.get("gripper_id") or "")
        if gid:
            from teleop_birth.ui.scene_builder.gripper_util import apply_gripper_to_scene_cfg

            menagerie_pre = self.project_root / str(paths.get("menagerie", "third_party/mujoco_menagerie"))
            apply_gripper_to_scene_cfg(
                self.project_root, scene, gid, menagerie=menagerie_pre
            )
            scene.pop("_gripper_warn", None)
            scene.pop("_gripper_applied", None)
        ik = dict(ik_cfg)
        # 采集需要朝下姿态对准指垫；仅位置 IK 会导致夹爪斜插/穿模
        ik["position_only"] = False
        if self.render:
            ik["viewer_sync_every"] = 1
        menagerie = self.project_root / str(paths.get("menagerie", "third_party/mujoco_menagerie"))
        return MujocoDualURBackend(
            menagerie=menagerie,
            scene_cfg=scene,
            ik_cfg=ik,
            ee_cfg=ee_cfg,
            render=self.render,
            sim_substeps=int(backend_cfg.get("sim_substeps", 4)),
        )

    def connect(self) -> None:
        self.backend = self._build_backend()
        self.backend.connect()
        assert self.backend.model is not None and self.backend.data is not None
        self.props = PropPoseController(self.backend.model, self.backend.data)
        self.props.discover([p.name for p in self.prop_specs])
        scene = self.backend.scene_cfg
        if self.randomize_region:
            self.randomizer = LayoutRandomizer.from_region_dict(
                self.randomize_region, seed=self.seed
            )
            print(
                f"[collect] 使用自定义随机化区域 "
                f"sep={self.randomizer.workspace.min_separation_m:.3f} m"
            )
            try:
                from teleop_birth.ui.session.region_panel import ensure_region_ui
                from teleop_birth.ui.session.region_viz import get_region_overlay

                get_region_overlay().set_region(self.randomize_region)
                get_region_overlay().set_visible(True)
                if self.render:
                    ensure_region_ui(self.randomize_region)
                    print("[collect] 随机化区域已显示在 MuJoCo（绿盒）")
            except Exception as exc:  # noqa: BLE001
                print(f"[collect] 区域可视化跳过: {exc}")
        else:
            self.randomizer = LayoutRandomizer.from_table_cfg(
                dict(scene["table"]),
                float(scene["base_height"]),
                seed=self.seed,
                arm=self.arm,
            )
        self.camera = MultiCameraCapture(
            self.backend.model, self.backend.data, self.camera_cfg
        )

    def close(self) -> None:
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        if self.backend is not None:
            self.backend.disconnect()
            self.backend = None

    def _table_top_z(self) -> float:
        """从后端场景配置解析桌面高度（与 desktop_base / table 一致）。"""
        scene = dict(self.backend.scene_cfg or {}) if self.backend else {}
        try:
            from teleop_birth.teleop.frame.desktop_base import table_top_z_from_scene

            return float(table_top_z_from_scene(scene))
        except Exception:  # noqa: BLE001
            db = dict(scene.get("desktop_base") or {})
            if "table_top_z" in db:
                return float(db["table_top_z"])
            if "table_top_z" in scene:
                return float(scene["table_top_z"])
            return 0.42

    def _settle(self, steps: int = 40) -> None:
        assert self.backend is not None and self.backend.model is not None
        assert self.backend.data is not None
        for _ in range(steps):
            mujoco.mj_step(self.backend.model, self.backend.data)

    def calibrate(self, session_dir: Path | None = None) -> dict[str, Any]:
        """采集前标定：解析 D405/D455 内参外参 → 校验 → 写 calibration.json → 应用到相机配置。"""
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = Path(session_dir or (self.output_dir / f"session_{stamp}"))
        self.session_dir.mkdir(parents=True, exist_ok=True)
        gate = CameraCalibration(self.project_root)
        payload = gate.run(
            self.camera_cfg,
            session_dir=self.session_dir,
            override_rel=self.calibration_override or None,
            existing_json=self.calibration_file or None,
        )
        self.calibration = payload
        self.camera_cfg = apply_calibration_to_config(self.camera_cfg, payload)
        # 若相机已 connect，用新 cfg 重建 capture
        if self.camera is not None and self.backend is not None:
            self.camera.close()
            assert self.backend.model is not None and self.backend.data is not None
            self.camera = MultiCameraCapture(
                self.backend.model, self.backend.data, self.camera_cfg
            )
        print("[collect] 相机标定完成:")
        for line in gate.summary_lines():
            print(f"  {line}")
        return payload

    def run(self) -> list[EpisodeResult]:
        if self.backend is None:
            self.connect()
        assert self.backend is not None and self.props is not None and self.randomizer is not None

        task_spec = self.task_spec
        results: list[EpisodeResult] = []
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"session_{stamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        # —— 先标定，再采集 ——
        if self.calibrate_before_collect or self.calibration is None:
            self.calibrate(self.session_dir)
        assert self.calibration is not None

        for ep in range(self.episodes):
            # 每集开始先回 home，避免上集姿态残留
            home_boot = SparseTarget(
                timestamp_ns=0,
                extremities=dict(self.backend.get_feedback()),
                active={EXTREMITY_LEFT_EE: False, EXTREMITY_RIGHT_EE: False},
                left_gripper=0.0,
                right_gripper=0.0,
                meta={
                    "home_right": float(self.arm.startswith("r")),
                    "home_left": float(not self.arm.startswith("r")),
                },
            )
            for _ in range(20):
                self.backend.apply(home_boot)

            self.randomizer.rng = np.random.default_rng(self.seed + ep * 997)
            layout = self.randomizer.apply(self.props, self.prop_specs)
            self._settle(50)

            # 容器类每步钉住
            anchors = {
                p.name: layout[p.name]
                for p in self.prop_specs
                if p.anchor and p.name in layout
            }

            out = self.session_dir / f"{task_spec.name}_ep{ep:03d}.mcap"
            writer = McapEpisodeWriter(
                out,
                topics=self.topics,
                task_info={
                    "task": task_spec.name,
                    "instruction_zh": task_spec.instruction_zh,
                    "instruction_en": task_spec.instruction_en,
                    "episode": ep,
                    "layout": layout,
                    "stages": [
                        {"action": s.action, "description": s.description} for s in task_spec.stages
                    ],
                    "calibration": self.calibration,
                    "calibration_path": str(self.session_dir / "calibration.json"),
                    "ref": {
                        "agibot_world": "https://agibot-world.com/",
                        "genie_sim": "https://github.com/AgibotTech/genie_sim",
                        "challenge2026": "https://agibot-world.com/challenge2026",
                    },
                },
            )

            table_top = self._table_top_z()
            sticky = StickyGrasp(
                self.props,
                object_name=self.task.grasp_object,
                model=self.backend.model,
                data=self.backend.data,
                attach_radius_m=0.045,
                grasp_offset_xyz=DEFAULT_PAD_OFFSET_XYZ,
                attach_gripper_min=0.55,
                place_object=self.task.place_object,
                place_radius_m=0.14,
                place_z_offset=0.03,
                table_top_z=table_top,
                object_half_z=self.task._object_half_z(),
                arm_prefix="R_" if self.arm.startswith("r") else "L_",
                contact_frames_needed=3,
                enable_latch=bool(self.sticky_grasp_enabled),
            )
            # 始终更新：加强摩擦；仅 sticky_grasp=True 时才运动学锁定
            use_sticky_update = True
            stage_name = "init"
            step_i = 0

            def on_pre_step(pose, g) -> None:
                if use_sticky_update:
                    sticky.update(pose, float(g))

            def on_step(state, target) -> None:
                nonlocal step_i
                if anchors:
                    self.props.set_many(anchors)
                ee = state.extremities
                key_local = EXTREMITY_RIGHT_EE if self.arm.startswith("r") else EXTREMITY_LEFT_EE
                frames = self.camera.maybe_frames() if self.camera is not None else {}
                writer.log_step(
                    step=step_i,
                    stage=stage_name,
                    ee_poses={
                        "left": ee[EXTREMITY_LEFT_EE],
                        "right": ee[EXTREMITY_RIGHT_EE],
                    },
                    prop_poses=self.props.snapshot(),
                    target_pose=target.extremities.get(key_local),
                    gripper={"left": target.left_gripper, "right": target.right_gripper},
                    extra={
                        "sticky_grasp": bool(sticky.attached),
                        "did_place": bool(sticky.did_place),
                        "latch_enabled": bool(self.sticky_grasp_enabled),
                    },
                    camera_jpegs=frames,
                )
                step_i += 1
                if self.pace_s > 0.0:
                    time.sleep(self.pace_s)

            def on_segment(_i: int, label: str) -> None:
                nonlocal stage_name
                stage_name = label

            # 自上而下，指垫开合沿世界 +X（与 top_down_quat 一致）
            down_q = top_down_quat_xyzw()
            wps = self.task.build_waypoints(
                self.props,
                down_quat_xyzw=down_q,
                table_top_z=table_top,
                place_hover_z=0.14,
                tcp_table_clearance=0.002,
                grasp_aim_up=0.0,
            )
            # 闭合建接触；hold=短距撕桌；抬起稍慢
            seg_steps = [self.steps_per_segment] * len(wps.poses)
            for i, lab in enumerate(wps.labels):
                if lab == "squeeze":
                    seg_steps[i] = max(self.steps_per_segment, int(self.steps_per_segment * 1.3))
                elif lab == "squeeze_hold":
                    seg_steps[i] = max(8, int(self.steps_per_segment * 0.35))
                elif lab == "lift_cube":
                    seg_steps[i] = max(self.steps_per_segment, int(self.steps_per_segment * 1.7))
                elif lab == "transport":
                    seg_steps[i] = max(self.steps_per_segment, int(self.steps_per_segment * 1.8))
                elif lab == "place_release":
                    seg_steps[i] = max(self.steps_per_segment, int(self.steps_per_segment * 1.2))
            motion = ScriptedEEMotion(
                self.backend,
                arm=self.arm,
                steps_per_segment=self.steps_per_segment,
                hold_steps=self.hold_steps,
            )
            motion.execute_waypoints(
                wps.poses,
                wps.grippers,
                on_step=on_step,
                on_pre_step=on_pre_step,
                on_segment=on_segment,
                labels=wps.labels,
                steps_per_segment=seg_steps,
            )
            # 松爪后多仿真几步，让方块落到盘子上（接触锁定与纯摩擦都需要）
            stage_name = "settle_drop"
            motion.hold(wps.poses[-1], 0.0, on_step=on_step, on_pre_step=on_pre_step, n_steps=25)

            stage_name = "reset"
            home_target = SparseTarget(
                timestamp_ns=0,
                extremities=dict(self.backend.get_feedback()),
                active={EXTREMITY_LEFT_EE: False, EXTREMITY_RIGHT_EE: False},
                left_gripper=0.0,
                right_gripper=0.0,
                meta={
                    "home_right": float(self.arm.startswith("r")),
                    "home_left": float(not self.arm.startswith("r")),
                },
            )
            for _ in range(25):
                st = self.backend.apply(home_target)
                on_step(st, home_target)

            ok = self.task.success(
                self.props,
                xy_tol=task_spec.success_xy_tol_m,
                z_tol=task_spec.success_z_tol_m,
            )
            writer.write_result(
                {"success": bool(ok), "episode": ep, "steps": writer.count}
            )
            writer.close()
            results.append(
                EpisodeResult(
                    episode=ep,
                    path=out,
                    success=bool(ok),
                    layout=layout,
                    steps=step_i,
                )
            )
            print(f"[collect] episode={ep} success={ok} steps={step_i} → {out}")

        # 多格式导出（HDF5 / LeRobot）；MCAP 始终保留
        if self.dataset_format and str(self.dataset_format).lower() not in ("mcap", ""):
            from teleop_birth.scene.recording.export_formats import export_session_results

            export_session_results(
                results,
                dataset_format=self.dataset_format,
                extra_meta={"task": task_spec.name, "format": self.dataset_format},
            )

        return results
