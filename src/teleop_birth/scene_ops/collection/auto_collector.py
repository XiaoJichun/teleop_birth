"""自动化采集：布局随机 → 语义 pick/place → 写 MCAP。

流程对齐智元 Genie Sim autocollect（任务 stages + 布局扰动 + episode 落盘），
实现适配本仓库 MuJoCo 双臂后端。
参考: https://github.com/AgibotTech/genie_sim · https://agibot-world.com/genie-sim
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Optional
import time

import mujoco
import numpy as np
from omegaconf import OmegaConf

from teleop_birth.embodiment.sim.mujoco_dual_ur import MujocoDualURBackend
from teleop_birth.scene_ops.collection.motion import ScriptedEEMotion
from teleop_birth.scene_ops.collection.sticky_grasp import StickyGrasp
from teleop_birth.scene_ops.placement.pose_controller import PropPoseController
from teleop_birth.scene_ops.placement.randomizer import LayoutRandomizer
from teleop_birth.scene_ops.props.catalog import PropCatalog
from teleop_birth.scene_ops.recording.camera import MultiCameraCapture, MultiCameraConfig
from teleop_birth.scene_ops.recording.mcap_writer import McapEpisodeWriter, TopicMap
from teleop_birth.scene_ops.tasks.base import TaskSpec
from teleop_birth.scene_ops.tasks.pick_place import SemanticPickPlaceTask
from teleop_birth.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, SparseTarget

if TYPE_CHECKING:
    from teleop_birth.cli.interactive_scene import CollectConfig


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

        self.catalog = PropCatalog(self.project_root / "configs" / "scene" / "props")
        objects = list(task_spec.objects) or ["red_cube", "blue_plate"]
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
        ik = dict(ik_cfg)
        ik["position_only"] = True
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

    def _settle(self, steps: int = 40) -> None:
        assert self.backend is not None and self.backend.model is not None
        assert self.backend.data is not None
        for _ in range(steps):
            mujoco.mj_step(self.backend.model, self.backend.data)

    def run(self) -> list[EpisodeResult]:
        if self.backend is None:
            self.connect()
        assert self.backend is not None and self.props is not None and self.randomizer is not None

        task_spec = self.task_spec
        results: list[EpisodeResult] = []
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

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

            out = self.output_dir / f"{task_spec.name}_ep{ep:03d}_{stamp}.mcap"
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
                    "ref": {
                        "agibot_world": "https://agibot-world.com/",
                        "genie_sim": "https://github.com/AgibotTech/genie_sim",
                        "challenge2026": "https://agibot-world.com/challenge2026",
                    },
                },
            )

            sticky = StickyGrasp(
                self.props,
                object_name=self.task.grasp_object,
                model=self.backend.model,
                attach_radius_m=0.08,
                grasp_offset_xyz=(0.0, 0.0, 0.028),
                place_object=self.task.place_object,
                place_radius_m=0.14,
                place_z_offset=0.03,
            )
            stage_name = "init"
            step_i = 0

            def on_step(state, target) -> None:
                nonlocal step_i
                if anchors:
                    self.props.set_many(anchors)
                ee = state.extremities
                key_local = EXTREMITY_RIGHT_EE if self.arm.startswith("r") else EXTREMITY_LEFT_EE
                g = target.right_gripper if self.arm.startswith("r") else target.left_gripper
                if self.sticky_grasp_enabled:
                    sticky.update(ee[key_local], g)
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
                        "sticky_grasp": bool(sticky.attached) if self.sticky_grasp_enabled else False,
                        "did_place": bool(sticky.did_place),
                    },
                    camera_jpegs=frames,
                )
                step_i += 1
                if self.pace_s > 0.0:
                    time.sleep(self.pace_s)

            def on_segment(_i: int, label: str) -> None:
                nonlocal stage_name
                stage_name = label

            fb = self.backend.get_feedback()
            key = EXTREMITY_RIGHT_EE if self.arm.startswith("r") else EXTREMITY_LEFT_EE
            down_q = np.asarray(fb[key][3:], dtype=np.float64)
            wps = self.task.build_waypoints(self.props, down_quat_xyzw=down_q)
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
                on_segment=on_segment,
                labels=wps.labels,
            )

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

        return results
