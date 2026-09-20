"""推理场景运行器：MuJoCo 仿真 + ROS 通讯 + 可选可视化/录像。"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from omegaconf import OmegaConf

from teleop_birth.scene.plugins.comms.bridge import (
    ActionCommand,
    CommsConfig,
    ObservationBundle,
    RosDataComms,
    load_comms_config,
)
from teleop_birth.scene.recording.camera import MultiCameraCapture, MultiCameraConfig
from teleop_birth.teleop.schema import (
    EXTREMITY_LEFT_EE,
    EXTREMITY_RIGHT_EE,
    SparseTarget,
    identity_pose,
)


@dataclass
class InferenceOptions:
    visualize: bool = True
    save_video: bool = False
    video_path: Path | None = None
    video_camera: str = "third_person"
    video_fps: float = 20.0
    control_hz: float = 20.0
    duration_s: float = 0.0  # 0 = until interrupt
    comms_overrides: dict[str, Any] = field(default_factory=dict)


@dataclass
class InferenceResult:
    steps: int
    video_path: Path | None
    transport: str


class InferenceRunner:
    """加载场景 → 启通讯 → 循环：收动作 / 步进 / 发观测 / 录像。"""

    def __init__(
        self,
        project_root: Path,
        scene_path: Path,
        *,
        options: InferenceOptions,
        inference_cfg: dict[str, Any] | None = None,
    ) -> None:
        self.project_root = Path(project_root)
        self.scene_path = Path(scene_path)
        self.options = options
        self.inference_cfg = dict(inference_cfg or {})
        self.backend = None
        self.camera: MultiCameraCapture | None = None
        self.comms: RosDataComms | None = None
        self._joint_names: list[str] = []
        self._joint_qadr: list[int] = []
        self._joint_aids: list[int] = []
        self._last_ee: dict[str, np.ndarray] = {}
        self._last_grip = (0.0, 0.0)
        self._viewer = None

    def run(self) -> InferenceResult:
        self._connect_backend()
        self._setup_camera()
        self._setup_joints()
        self._setup_comms()
        assert self.backend is not None and self.comms is not None

        video_writer = None
        video_path = self.options.video_path
        if self.options.save_video:
            video_path = video_path or self._default_video_path()
            video_writer = _VideoWriter(
                video_path,
                fps=float(self.options.video_fps),
                camera_name=self.options.video_camera,
            )
            print(f"[inference] 录像 → {video_path}")

        hz = max(1.0, float(self.options.control_hz))
        dt = 1.0 / hz
        duration = float(self.options.duration_s)
        t0 = time.time()
        steps = 0
        print(
            f"[inference] 开始  viz={self.options.visualize}  "
            f"state={self.comms.cfg.state_mode}  action={self.comms.cfg.action_mode}  "
            f"transport={self.comms.transport}"
        )
        print("  提示: Ctrl+C 结束；有可视化时关闭 MuJoCo 窗口也会结束。")

        try:
            while True:
                loop_t = time.time()
                if duration > 0 and (loop_t - t0) >= duration:
                    break
                if self._viewer is not None and hasattr(self._viewer, "is_running"):
                    if not self._viewer.is_running():
                        break

                self.comms.spin_once()
                cmd = self.comms.poll_action()
                self._apply_command(cmd)

                obs = self._build_observation()
                self.comms.publish_observation(obs)

                if video_writer is not None:
                    frame = self._pick_video_frame(obs.cameras)
                    if frame is not None:
                        video_writer.write(frame)

                if self._viewer is not None:
                    self._viewer.sync()

                steps += 1
                sleep = dt - (time.time() - loop_t)
                if sleep > 0:
                    time.sleep(sleep)
        except KeyboardInterrupt:
            print("\n[inference] 用户中断")
        finally:
            if video_writer is not None:
                video_writer.close()
                print(f"[inference] 视频已保存: {video_path}")
            transport = self.comms.transport if self.comms else "—"
            self.close()
        return InferenceResult(steps=steps, video_path=video_path if self.options.save_video else None, transport=transport)

    def close(self) -> None:
        if self.comms is not None:
            self.comms.stop()
            self.comms = None
        if self.camera is not None:
            self.camera.close()
            self.camera = None
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None
        if self.backend is not None:
            try:
                self.backend.disconnect()
            except Exception:
                pass
            self.backend = None

    # —— setup ——
    def _connect_backend(self) -> None:
        from teleop_birth.scene.plugins.scene import load_scene_yaml
        from teleop_birth.ui.flows.browse_scenes import _build_scene_model

        scene_cfg = load_scene_yaml(self.scene_path)
        adapter = str(
            (scene_cfg.get("backend") or {}).get("type") or scene_cfg.get("type") or ""
        )
        # 统一：先编译 model，再包一层轻量 backend 适配
        if "logistics" in adapter or "artixon" in adapter:
            self.backend = _ArtixonInferenceBackend(
                self.project_root, self.scene_path, render=False
            )
        else:
            self.backend = _DualURInferenceBackend(
                self.project_root,
                self.scene_path,
                scene_cfg,
                inference_cfg=self.inference_cfg,
                render=False,
            )
        self.backend.connect()
        # 可视化单独挂 passive viewer（与 backend.render 解耦，便于无界面也录像）
        if self.options.visualize:
            try:
                from mujoco import viewer as mj_viewer

                self._viewer = mj_viewer.launch_passive(self.backend.model, self.backend.data)
            except Exception as exc:  # noqa: BLE001
                print(f"[inference] viewer 不可用: {exc}；继续无头模式")
                self._viewer = None

    def _setup_camera(self) -> None:
        from teleop_birth.scene.plugins.scene import load_scene_yaml
        from teleop_birth.scene.recording.camera import MultiCameraConfig

        scene_cfg = load_scene_yaml(self.scene_path)
        recording_rel = str(self.inference_cfg.get("recording") or "scene/recording.yaml")
        recording: dict = {}
        rec_path = self.project_root / "configs" / recording_rel
        if rec_path.is_file():
            loaded = OmegaConf.to_container(OmegaConf.load(rec_path), resolve=True)
            if isinstance(loaded, dict):
                recording = loaded
        cam_cfg = MultiCameraConfig.from_scene(
            self.project_root, scene_cfg, recording=recording
        )
        cam_cfg.enabled = True
        cam_cfg.every_n_steps = 1
        assert self.backend is not None
        self.camera = MultiCameraCapture(self.backend.model, self.backend.data, cam_cfg)

    def _setup_joints(self) -> None:
        assert self.backend is not None
        names, qadr, aids = self.backend.list_arm_joints()
        self._joint_names = names
        self._joint_qadr = qadr
        self._joint_aids = aids
        fb = self.backend.get_feedback()
        self._last_ee = {
            EXTREMITY_LEFT_EE: np.asarray(fb.get(EXTREMITY_LEFT_EE, identity_pose()), dtype=np.float64),
            EXTREMITY_RIGHT_EE: np.asarray(fb.get(EXTREMITY_RIGHT_EE, identity_pose()), dtype=np.float64),
        }

    def _setup_comms(self) -> None:
        from teleop_birth.scene.plugins.scene import load_scene_yaml

        scene_cfg = load_scene_yaml(self.scene_path)
        rel = str(self.inference_cfg.get("comms") or "plugins/comms/ros_bridge.yaml")
        scene_over = scene_cfg.get("comms") if isinstance(scene_cfg.get("comms"), dict) else {}
        overrides = {**(scene_over or {}), **(self.options.comms_overrides or {})}
        cfg = load_comms_config(self.project_root, rel=rel, overrides=overrides or None)
        # 无可视化且用户未强制 ros 时，允许 mock（避免无 ROS 环境直接失败）
        self.comms = RosDataComms(cfg)
        self.comms.start()

    def _default_video_path(self) -> Path:
        out = self.project_root / str(self.inference_cfg.get("output_dir") or "logs/inference")
        out.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return out / f"{self.scene_path.stem}_{stamp}.mp4"

    def _build_observation(self) -> ObservationBundle:
        assert self.backend is not None and self.camera is not None
        mujoco.mj_forward(self.backend.model, self.backend.data)
        cams = self.camera.capture_rgb()
        fb = self.backend.get_feedback()
        jpos = None
        if self._joint_qadr:
            jpos = np.array(
                [float(self.backend.data.qpos[a]) for a in self._joint_qadr],
                dtype=np.float64,
            )
        grip = self.backend.read_grippers()
        return ObservationBundle(
            timestamp_ns=time.time_ns(),
            cameras=cams,
            joint_names=list(self._joint_names),
            joint_positions=jpos,
            ee_left=np.asarray(fb.get(EXTREMITY_LEFT_EE, identity_pose()), dtype=np.float64),
            ee_right=np.asarray(fb.get(EXTREMITY_RIGHT_EE, identity_pose()), dtype=np.float64),
            left_gripper=float(grip[0]),
            right_gripper=float(grip[1]),
        )

    def _pick_video_frame(self, cameras: dict[str, np.ndarray]) -> np.ndarray | None:
        name = self.options.video_camera
        if name in cameras:
            return cameras[name]
        if cameras:
            return next(iter(cameras.values()))
        return None

    def _apply_command(self, cmd: ActionCommand | None) -> None:
        assert self.backend is not None and self.comms is not None
        if cmd is None:
            if self.comms.cfg.on_idle == "hold":
                self._hold_step()
            else:
                self.backend.idle_step()
            return

        if cmd.mode == "joint" or self.comms.cfg.action_mode == "joint":
            if cmd.joint_positions is not None:
                self.backend.apply_joints(
                    cmd.joint_names or self._joint_names,
                    np.asarray(cmd.joint_positions, dtype=np.float64),
                )
            lg = self._last_grip[0] if cmd.left_gripper is None else float(cmd.left_gripper)
            rg = self._last_grip[1] if cmd.right_gripper is None else float(cmd.right_gripper)
            self._last_grip = (lg, rg)
            self.backend.apply_grippers(lg, rg)
            self.backend.idle_step()
            return

        # EE mode
        left = self._last_ee[EXTREMITY_LEFT_EE]
        right = self._last_ee[EXTREMITY_RIGHT_EE]
        active = {EXTREMITY_LEFT_EE: False, EXTREMITY_RIGHT_EE: False}
        if cmd.ee_left is not None and cmd.left_active:
            left = np.asarray(cmd.ee_left, dtype=np.float64)
            active[EXTREMITY_LEFT_EE] = True
            self._last_ee[EXTREMITY_LEFT_EE] = left
        if cmd.ee_right is not None and cmd.right_active:
            right = np.asarray(cmd.ee_right, dtype=np.float64)
            active[EXTREMITY_RIGHT_EE] = True
            self._last_ee[EXTREMITY_RIGHT_EE] = right
        lg = self._last_grip[0] if cmd.left_gripper is None else float(cmd.left_gripper)
        rg = self._last_grip[1] if cmd.right_gripper is None else float(cmd.right_gripper)
        self._last_grip = (lg, rg)
        target = SparseTarget(
            timestamp_ns=cmd.timestamp_ns or time.time_ns(),
            extremities={EXTREMITY_LEFT_EE: left, EXTREMITY_RIGHT_EE: right},
            active=active,
            left_gripper=lg,
            right_gripper=rg,
        )
        self.backend.apply(target)

    def _hold_step(self) -> None:
        """无新动作：用上次末端目标再 apply 一次（保持姿态）。"""
        assert self.backend is not None
        target = SparseTarget(
            timestamp_ns=time.time_ns(),
            extremities=dict(self._last_ee),
            active={EXTREMITY_LEFT_EE: True, EXTREMITY_RIGHT_EE: True},
            left_gripper=self._last_grip[0],
            right_gripper=self._last_grip[1],
        )
        self.backend.apply(target)


# —— 轻量 backend 适配 ——

class _DualURInferenceBackend:
    def __init__(
        self,
        root: Path,
        scene_path: Path,
        scene_cfg: dict[str, Any],
        *,
        inference_cfg: dict[str, Any],
        render: bool,
    ) -> None:
        self.root = root
        self.scene_path = scene_path
        self.scene_yaml = scene_cfg
        self.inference_cfg = inference_cfg
        self.render = render
        self.inner = None
        self.model = None
        self.data = None

    def connect(self) -> None:
        from teleop_birth.robot.sim.mujoco_dual_ur import MujocoDualURBackend

        backend_rel = str(self.inference_cfg.get("backend") or "device/backend/mujoco_dual_ur.yaml")
        ik_rel = str(self.inference_cfg.get("ik") or "algorithm/ik/dls.yaml")
        ee_rel = str(self.inference_cfg.get("end_effector") or "device/end_effector/gripper.yaml")
        paths_rel = str(self.inference_cfg.get("paths") or "device/paths/default.yaml")
        backend_cfg = OmegaConf.to_container(
            OmegaConf.load(self.root / "configs" / backend_rel), resolve=True
        )
        ik_cfg = OmegaConf.to_container(OmegaConf.load(self.root / "configs" / ik_rel), resolve=True)
        ee_cfg = OmegaConf.to_container(OmegaConf.load(self.root / "configs" / ee_rel), resolve=True)
        paths = OmegaConf.to_container(OmegaConf.load(self.root / "configs" / paths_rel), resolve=True)
        assert isinstance(backend_cfg, dict) and isinstance(ik_cfg, dict)
        assert isinstance(ee_cfg, dict) and isinstance(paths, dict)
        scene = dict(backend_cfg.get("scene") or {})
        if self.scene_yaml.get("desktop_base"):
            scene["desktop_base"] = dict(self.scene_yaml["desktop_base"])
        # props from scene
        from teleop_birth.ui.flows.browse_scenes import _resolve_scene_props

        scene["props"] = _resolve_scene_props(self.root, self.scene_yaml)
        scene["project_root"] = str(self.root)
        menagerie = self.root / str(paths.get("menagerie", "third_party/mujoco_menagerie"))
        ik = dict(ik_cfg)
        ik["position_only"] = False
        self.inner = MujocoDualURBackend(
            menagerie=menagerie,
            scene_cfg=scene,
            ik_cfg=ik,
            ee_cfg=ee_cfg,
            render=False,
            sim_substeps=int(backend_cfg.get("sim_substeps", 4)),
        )
        self.inner.connect()
        self.model = self.inner.model
        self.data = self.inner.data

    def disconnect(self) -> None:
        if self.inner is not None:
            self.inner.disconnect()

    def get_feedback(self):
        return self.inner.get_feedback()

    def apply(self, target: SparseTarget):
        return self.inner.apply(target)

    def read_grippers(self) -> tuple[float, float]:
        assert self.inner is not None
        lg = float(self.inner.left_ee.read().gripper_openness) if self.inner.left_ee else 0.0
        rg = float(self.inner.right_ee.read().gripper_openness) if self.inner.right_ee else 0.0
        return lg, rg

    def apply_grippers(self, left: float, right: float) -> None:
        from teleop_birth.robot.ee.base import EECommand

        assert self.inner is not None
        if self.inner.left_ee:
            self.inner.left_ee.apply(EECommand(gripper_openness=left))
        if self.inner.right_ee:
            self.inner.right_ee.apply(EECommand(gripper_openness=right))

    def list_arm_joints(self) -> tuple[list[str], list[int], list[int]]:
        assert self.inner is not None and self.model is not None
        names: list[str] = []
        qadr: list[int] = []
        aids: list[int] = []
        for aid in list(self.inner._left_arm_aids) + list(self.inner._right_arm_aids):
            j = int(self.model.actuator_trnid[aid, 0])
            jn = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) or f"j{j}"
            names.append(jn)
            qadr.append(int(self.model.jnt_qposadr[j]))
            aids.append(int(aid))
        return names, qadr, aids

    def apply_joints(self, names: list[str], positions: np.ndarray) -> None:
        assert self.model is not None and self.data is not None
        name_to_q = {n: float(p) for n, p in zip(names, positions.reshape(-1))}
        # 按 actuator 写 ctrl + qpos
        for aid in list(self.inner._left_arm_aids) + list(self.inner._right_arm_aids):
            j = int(self.model.actuator_trnid[aid, 0])
            jn = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j) or ""
            if jn in name_to_q:
                q = name_to_q[jn]
                self.data.ctrl[aid] = q
                self.data.qpos[self.model.jnt_qposadr[j]] = q
        mujoco.mj_forward(self.model, self.data)

    def idle_step(self) -> None:
        assert self.inner is not None and self.model is not None and self.data is not None
        for _ in range(int(self.inner.sim_substeps)):
            mujoco.mj_step(self.model, self.data)


class _ArtixonInferenceBackend:
    def __init__(self, root: Path, scene_path: Path, *, render: bool) -> None:
        self.root = root
        self.scene_path = scene_path
        self.render = render
        self.inner = None
        self.model = None
        self.data = None
        self._hold_left = identity_pose()
        self._hold_right = identity_pose()

    def connect(self) -> None:
        from teleop_birth.robot.arm.artixon.backend import DualArtixOnBackend
        from teleop_birth.scene.plugins.scene import load_scene_yaml
        from teleop_birth.ui.flows.browse_scenes import _resolve_scene_props

        scene_cfg = load_scene_yaml(self.scene_path)
        module = (scene_cfg.get("backend") or {}).get("module_config") or (
            "configs/robots/artixon_logistics_scene.yaml"
        )
        base = OmegaConf.to_container(OmegaConf.load(self.root / str(module)), resolve=True)
        assert isinstance(base, dict)
        if scene_cfg.get("desktop_base"):
            base["desktop_base"] = dict(scene_cfg["desktop_base"])
        if "packages" in scene_cfg:
            base["packages"] = list(scene_cfg.get("packages") or [])
        base["props"] = _resolve_scene_props(self.root, scene_cfg)
        base["project_root"] = str(self.root)
        self.inner = DualArtixOnBackend(scene_cfg=base, render=False)
        self.inner.connect()
        self.model = self.inner.model
        self.data = self.inner.data
        self._hold_left = self.inner.tcp_pose("left")
        self._hold_right = self.inner.tcp_pose("right")

    def disconnect(self) -> None:
        if self.inner is not None:
            self.inner.disconnect()

    def get_feedback(self):
        assert self.inner is not None
        return {
            EXTREMITY_LEFT_EE: self.inner.tcp_pose("left"),
            EXTREMITY_RIGHT_EE: self.inner.tcp_pose("right"),
        }

    def apply(self, target: SparseTarget):
        assert self.inner is not None
        left = target.extremities.get(EXTREMITY_LEFT_EE, self._hold_left)
        right = target.extremities.get(EXTREMITY_RIGHT_EE, self._hold_right)
        if target.is_active(EXTREMITY_LEFT_EE):
            self._hold_left = np.asarray(left, dtype=np.float64)
        if target.is_active(EXTREMITY_RIGHT_EE):
            self._hold_right = np.asarray(right, dtype=np.float64)
        self.inner.set_gripper("left", target.left_gripper)
        self.inner.set_gripper("right", target.right_gripper)
        self.inner.step_to_poses(
            left=self._hold_left if target.is_active(EXTREMITY_LEFT_EE) else None,
            right=self._hold_right if target.is_active(EXTREMITY_RIGHT_EE) else None,
            left_gripper=target.left_gripper,
            right_gripper=target.right_gripper,
        )

    def read_grippers(self) -> tuple[float, float]:
        return 0.0, 0.0

    def apply_grippers(self, left: float, right: float) -> None:
        assert self.inner is not None
        self.inner.set_gripper("left", left)
        self.inner.set_gripper("right", right)

    def list_arm_joints(self) -> tuple[list[str], list[int], list[int]]:
        assert self.inner is not None and self.model is not None
        names: list[str] = []
        qadr: list[int] = []
        aids: list[int] = []
        for side in ("left", "right"):
            for jn in self.inner.name_map["arm_joints"][side]:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                if jid < 0:
                    continue
                names.append(jn)
                qadr.append(int(self.model.jnt_qposadr[jid]))
            for an in self.inner.name_map["arm_actuators"][side]:
                aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, an)
                if aid >= 0:
                    aids.append(int(aid))
        return names, qadr, aids

    def apply_joints(self, names: list[str], positions: np.ndarray) -> None:
        assert self.model is not None and self.data is not None
        for n, p in zip(names, positions.reshape(-1)):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            if jid < 0:
                continue
            self.data.qpos[self.model.jnt_qposadr[jid]] = float(p)
            # 同步 ctrl
            for aid in range(self.model.nu):
                if int(self.model.actuator_trnid[aid, 0]) == jid:
                    self.data.ctrl[aid] = float(p)
                    break
        mujoco.mj_forward(self.model, self.data)

    def idle_step(self) -> None:
        assert self.model is not None and self.data is not None
        for _ in range(4):
            mujoco.mj_step(self.model, self.data)


class _VideoWriter:
    def __init__(self, path: Path, *, fps: float, camera_name: str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.fps = float(fps)
        self.camera_name = camera_name
        self._writer = None
        self._size = None

    def write(self, rgb: np.ndarray) -> None:
        frame = np.asarray(rgb, dtype=np.uint8)
        if frame.ndim != 3:
            return
        h, w = frame.shape[:2]
        if self._writer is None:
            self._size = (w, h)
            self._writer = self._open(w, h)
            self._kind = "cv2"
        if self._writer is None:
            return
        if getattr(self, "_kind", "cv2") == "imageio":
            self._writer.append_data(frame)
            return
        try:
            import cv2

            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self._writer.write(bgr)
        except Exception:
            self._writer.write(frame)

    def _open(self, w: int, h: int):
        try:
            import cv2

            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._kind = "cv2"
            return cv2.VideoWriter(str(self.path), fourcc, self.fps, (w, h))
        except Exception as exc:  # noqa: BLE001
            print(f"[inference] OpenCV VideoWriter 不可用: {exc}；改用 imageio")
        try:
            import imageio.v2 as imageio

            self._kind = "imageio"
            return imageio.get_writer(str(self.path), fps=self.fps)
        except Exception as exc:  # noqa: BLE001
            print(f"[inference] 无法创建视频写入: {exc}")
            return None

    def close(self) -> None:
        if self._writer is None:
            return
        try:
            if hasattr(self._writer, "release"):
                self._writer.release()
            else:
                self._writer.close()
        except Exception:
            pass
        self._writer = None
