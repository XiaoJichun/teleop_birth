"""双臂物流自动采集。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from omegaconf import OmegaConf

from teleop_birth.scene.plugins.physics import AntiPenetrationManager
from teleop_birth.robot.arm.artixon.backend import DualArtixOnBackend
from teleop_birth.robot.arm.artixon.collection.motion import DualScriptedMotion
from teleop_birth.robot.arm.artixon.collection.sticky import StickyGrasp
from teleop_birth.robot.arm.artixon.paths import configs_dir, default_output_dir, module_root
from teleop_birth.robot.arm.artixon.policies.dual_sort import PolicyContext, run_pick_flip_place
from teleop_birth.scene.recording.camera import MultiCameraCapture, MultiCameraConfig
from teleop_birth.scene.recording.mcap_writer import McapEpisodeWriter, TopicMap
from teleop_birth.robot.arm.artixon.scene.props import PropPoseController


@dataclass
class EpisodeResult:
    episode: int
    path: Path
    success: bool
    package: str
    policy: str
    steps: int


class LogisticsCollector:
    def __init__(self, cfg: dict[str, Any] | None = None) -> None:
        self.root = module_root()
        self.cfg = dict(cfg or {})
        self.backend: DualArtixOnBackend | None = None
        self.props: PropPoseController | None = None
        self.camera: MultiCameraCapture | None = None
        self.anti: AntiPenetrationManager | None = None

    @classmethod
    def from_yaml(cls, path: Path | None = None) -> "LogisticsCollector":
        p = Path(path or (configs_dir() / "artixon_collect.yaml"))
        raw = OmegaConf.to_container(OmegaConf.load(p), resolve=True)
        assert isinstance(raw, dict)
        return cls(raw)

    def connect(self) -> None:
        scene_file = str(self.cfg.get("scene_file") or "artixon_logistics_scene.yaml")
        sp = configs_dir() / scene_file
        scene: dict[str, Any] = {}
        if sp.is_file():
            loaded = OmegaConf.to_container(OmegaConf.load(sp), resolve=True)
            assert isinstance(loaded, dict)
            scene = loaded
        scene.update(dict(self.cfg.get("scene") or {}))

        self.backend = DualArtixOnBackend(
            scene_cfg=scene,
            ik_cfg=dict(self.cfg.get("ik") or {}),
            render=bool(self.cfg.get("render", False)),
            sim_substeps=int(self.cfg.get("sim_substeps", 5)),
        )
        self.backend.connect()
        assert self.backend.model is not None and self.backend.data is not None
        self.props = PropPoseController(self.backend.model, self.backend.data)
        self.props.discover(list(self.backend.name_map["prop_names"]))

        table_z = float(self.backend.name_map["table_z"])
        table_top = table_z + float((scene.get("table_size") or [0.7, 0.55, 0.02])[2])
        self.anti = AntiPenetrationManager(
            self.backend.model,
            self.backend.data,
            table_top_z=table_top,
            min_separation_m=float(self.cfg.get("min_prop_separation_m", 0.08)),
            soft_push=0.8,
            tcp_clearance_m=0.07,
        )
        # 按包裹半高注册
        pkg_half_z: dict[str, float] = {}
        for p in scene.get("packages") or []:
            if isinstance(p, dict) and p.get("name") and p.get("size"):
                pkg_half_z[str(p["name"])] = float(p["size"][2])
        for name in self.backend.name_map["prop_names"]:
            self.anti.register_prop(name, half_extent_z=pkg_half_z.get(name))
        for side in ("left", "right"):
            self.anti.register_tcp_site(self.backend.name_map["tcp_sites"][side])
        for pref in ("L_", "R_"):
            self.anti.register_finger_bodies(
                f"{pref}arm_gripper_left_link",
                f"{pref}arm_gripper_right_link",
                f"{pref}arm_gripper_base_link",
            )
        for key in ("in_conveyor", "out_conveyor"):
            c = dict(scene.get(key) or {})
            if not c.get("pos") or not c.get("size"):
                continue
            pos = [float(x) for x in c["pos"]]
            size = [float(x) for x in c["size"]]
            self.anti.register_support_plane(
                center_xy=(pos[0], pos[1]),
                half_xy=(size[0], size[1]),
                top_z=pos[2] + size[2],
            )
        self.anti.step()

        rec_path = configs_dir() / str(self.cfg.get("recording") or "artixon_recording.yaml")
        recording: dict[str, Any] = {}
        if rec_path.is_file():
            loaded = OmegaConf.to_container(OmegaConf.load(rec_path), resolve=True)
            assert isinstance(loaded, dict)
            recording = loaded
        cam_cfg = MultiCameraConfig.from_recording_dict(recording)
        self.camera = MultiCameraCapture(self.backend.model, self.backend.data, cam_cfg)
        self._topics = TopicMap.from_dict(dict(recording.get("topics") or {}))
        self._scene = scene

    def close(self) -> None:
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        if self.backend is not None:
            self.backend.disconnect()
            self.backend = None

    def run(self) -> list[EpisodeResult]:
        if self.backend is None:
            self.connect()
        assert self.backend and self.props and self.anti and self.camera

        episodes = int(self.cfg.get("episodes", 2))
        seed = int(self.cfg.get("seed", 0))
        # policy: pick_flip_place（主流程）| auto 等同
        policy_mode = str(self.cfg.get("policy") or "pick_flip_place")
        package = str(self.cfg.get("package") or self.backend.name_map["prop_names"][0])
        out_dir = Path(self.cfg.get("output_dir") or default_output_dir())
        if not out_dir.is_absolute():
            out_dir = self.root / out_dir
        out_dir.mkdir(parents=True, exist_ok=True)
        pace_s = float(self.cfg.get("pace_s", 0.0))
        steps = int(self.cfg.get("steps_per_segment", 35))
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results: list[EpisodeResult] = []
        motion = DualScriptedMotion(self.backend, steps_per_segment=steps)
        rng = np.random.default_rng(seed)

        nm = self.backend.name_map
        table_z = float(nm["table_z"])
        table_top = table_z + float((self._scene.get("table_size") or [0.7, 0.55, 0.02])[2])

        for ep in range(episodes):
            self.backend.reset_home()
            # 重置桌上全部快递盒到配置位姿；作业盒可按策略改面单朝向
            base_label = dict(nm.get("label_up") or {})
            # 视频流程总是翻面：初始面单朝下
            label_up = False
            if str(policy_mode).lower() in ("auto", "random"):
                label_up = bool(rng.integers(0, 2))

            pkg_cfgs = {
                str(p["name"]): p
                for p in (self._scene.get("packages") or [])
                if isinstance(p, dict) and p.get("name")
            }
            for pname in nm["prop_names"]:
                pc = dict(pkg_cfgs.get(pname) or {})
                size = [float(x) for x in (pc.get("size") or [0.045, 0.035, 0.025])]
                pos0 = [float(x) for x in (pc.get("pos") or [0.48, 0.0, table_top + size[2]])]
                # 轻微抖动，仍落在桌面
                jitter = rng.uniform(-0.02, 0.02, size=2)
                xy = np.asarray(pos0[:2], dtype=np.float64) + jitter
                z = table_top + size[2] + 0.001
                if pname == package:
                    lu = label_up
                else:
                    lu = bool(pc.get("label_up", base_label.get(pname, True)))
                quat = (
                    np.array([0.0, 0.0, 0.0, 1.0])
                    if lu
                    else np.array([1.0, 0.0, 0.0, 0.0])
                )
                self.props.set_pose(
                    pname,
                    pos=[float(xy[0]), float(xy[1]), float(z)],
                    quat_xyzw=quat,
                )
            for _ in range(40):
                self.anti.step()
                mujoco.mj_step(self.backend.model, self.backend.data)

            out = out_dir / f"logistics_dual_{package}_ep{ep:03d}_{stamp}.mcap"
            writer = McapEpisodeWriter(
                out,
                topics=self._topics,
                task_info={
                    "task": "dual_arm_logistics_sort",
                    "instruction_zh": "双臂物流分拣：入站取件/翻面单/拨出站",
                    "robot": "ArtiXon Arm-6A x2",
                    "episode": ep,
                    "package": package,
                    "label_up_initial": label_up,
                    "policy_mode": policy_mode,
                    "ref": {
                        "product": "https://x2robot.cn/product/6a746ee800090d7c1dcb05e0",
                        "sdk": "https://github.com/X-Square-Robot/sdk_arm_6a",
                        "note": "视频 /home/xiao/logistics data collection.mp4 本机不可用，按用户动作描述实现",
                    },
                },
            )
            sticky = StickyGrasp(
                self.props,
                self.anti,
                object_name=package,
                model=self.backend.model,
                data=self.backend.data,
                grasp_offset_xyz=(0.0, 0.0, 0.0),
                attach_radius_m=0.10,
                attach_gripper_min=0.02,
            )
            step_i = 0

            def on_step(ees, lg, rg, stage: str) -> None:
                nonlocal step_i
                self.anti.step()
                frames = self.camera.maybe_frames() if self.camera else {}
                writer.log_step(
                    step=step_i,
                    stage=stage,
                    ee_poses={"left": ees["left"], "right": ees["right"]},
                    prop_poses=self.props.snapshot(),
                    gripper={"left": float(lg), "right": float(rg)},
                    extra={
                        "left_ee": ees["left"].tolist(),
                        "right_ee": ees["right"].tolist(),
                        "left_gripper": lg,
                        "right_gripper": rg,
                        "sticky": sticky.attached,
                        "label_up_initial": label_up,
                    },
                    camera_jpegs=frames,
                    target_pose=ees["left"],
                )
                step_i += 1
                if pace_s > 0:
                    time.sleep(pace_s)

            half = (0.045, 0.035, 0.025)
            for p in self._scene.get("packages") or []:
                if isinstance(p, dict) and str(p.get("name")) == package and p.get("size"):
                    sz = [float(x) for x in p["size"]]
                    half = (sz[0], sz[1], sz[2])
                    break

            ctx = PolicyContext(
                props=self.props,
                motion=motion,
                sticky=sticky,
                work_xy=tuple(nm["work_xy"]),
                in_xy=tuple(nm["in_conveyor_xy"]),
                out_xy=tuple(nm["out_conveyor_xy"]),
                table_top_z=table_top,
                label_up=label_up,
                package=package,
                package_half_xyz=half,
            )

            # 固定主流程：左取 → 双手翻 → 右放
            used = run_pick_flip_place(ctx, on_step=on_step)

            self.backend.reset_home()
            for _ in range(10):
                ees = self.backend.step_to_poses(left_gripper=0.0, right_gripper=0.0)
                self.anti.step()
                on_step(ees, 0.0, 0.0, "reset")

            out_xy = np.asarray(nm["out_conveyor_xy"], dtype=np.float64)
            pkg = self.props.get_pose(package)
            ok = float(np.linalg.norm(pkg[:2] - out_xy)) < 0.18
            writer.write_result(
                {"success": ok, "episode": ep, "steps": writer.count, "policy": used}
            )
            writer.close()
            results.append(
                EpisodeResult(
                    episode=ep,
                    path=out,
                    success=ok,
                    package=package,
                    policy=used,
                    steps=step_i,
                )
            )
            print(
                f"[logistics] episode={ep} policy={used} success={ok} steps={step_i} → {out}"
            )

        return results
