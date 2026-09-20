"""场景搭建 MuJoCo 预览会话：可与 PyQt 并用，支持重建 + 离屏相机渲染。"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from teleop_birth.ui.scene_builder.draft import SceneDraft
from teleop_birth.ui.scene_builder.gripper_util import apply_gripper_to_scene_cfg

_SESSION: Optional["PreviewSession"] = None
_LOCK = threading.Lock()


def get_preview_session() -> Optional["PreviewSession"]:
    return _SESSION


def set_preview_session(session: Optional["PreviewSession"]) -> None:
    global _SESSION
    with _LOCK:
        _SESSION = session


class PreviewSession:
    """后台 passive viewer；草稿变更可重建；提供离屏 RGB 给工坊相机页。"""

    def __init__(self, project_root: Path, draft: SceneDraft) -> None:
        self.project_root = Path(project_root)
        self.draft = draft
        self._rebuild_event = threading.Event()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_error: str | None = None
        self.status = "idle"
        self._model = None
        self._data = None
        self._renderer = None
        self._state_lock = threading.RLock()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="scene-preview")
        self._thread.start()
        set_preview_session(self)
        time.sleep(0.35)

    def stop(self) -> None:
        self._stop.set()
        self._rebuild_event.set()
        set_preview_session(None)
        with self._state_lock:
            self._renderer = None
            self._model = None
            self._data = None

    def request_rebuild(self, draft: SceneDraft | None = None) -> None:
        if draft is not None:
            self.draft = draft
        self.invalidate()
        self._rebuild_event.set()

    def invalidate(self) -> None:
        """丢弃已编译模型，下次 ensure_compiled / viewer 重建会重新编译。"""
        with self._state_lock:
            self._renderer = None
            self._model = None
            self._data = None

    def sync_draft(self, draft: SceneDraft) -> bool:
        """仅更新草稿并离屏重编译（不重启 MuJoCo viewer，避免与 Qt 抢 GL 卡死）。"""
        self.draft = draft
        self.invalidate()
        return self.ensure_compiled()

    def ensure_compiled(self) -> bool:
        """无 viewer 时也可编译，供仅工坊/离屏渲染。"""
        with self._state_lock:
            if self._model is not None:
                return True
        try:
            model, data = self._compile()
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)
            print(f"  MuJoCo 编译失败: {exc}")
            return False
        with self._state_lock:
            self._model, self._data = model, data
            self._renderer = None
            self._last_error = None
        return True

    def render_rgb(
        self,
        cam: dict[str, Any] | None = None,
        *,
        width: int = 640,
        height: int = 480,
    ) -> np.ndarray | None:
        """按相机 pos/lookat 离屏渲染；失败返回 None。"""
        import mujoco

        if not self.ensure_compiled():
            return None
        with self._state_lock:
            model, data = self._model, self._data
            if model is None or data is None:
                return None
            mujoco.mj_forward(model, data)
            if (
                self._renderer is None
                or int(getattr(self._renderer, "height", 0)) != height
                or int(getattr(self._renderer, "width", 0)) != width
            ):
                self._renderer = mujoco.Renderer(model, height=height, width=width)
            renderer = self._renderer
            mjcam = mujoco.MjvCamera()
            mujoco.mjv_defaultFreeCamera(model, mjcam)
            c = dict(cam or {})
            lookat = np.asarray(c.get("lookat") or [0.35, 0.0, 0.45], dtype=np.float64)
            pos = np.asarray(c.get("pos") or [0.55, -0.75, 1.15], dtype=np.float64)
            if "offset_xyz" in c and c.get("type") == "site":
                # 腕部：用近似世界位姿（偏移当自由相机）
                pos = np.asarray(c.get("offset_xyz") or pos, dtype=np.float64)
                lookat = pos + np.array([0.15, 0.0, -0.05])
            mjcam.lookat[:] = lookat
            delta = pos - lookat
            dist = float(np.linalg.norm(delta))
            mjcam.distance = max(dist, 0.05)
            horiz = max(float(np.hypot(delta[0], delta[1])), 1e-6)
            mjcam.azimuth = float(np.degrees(np.arctan2(delta[1], delta[0])))
            mjcam.elevation = float(np.degrees(np.arctan2(delta[2], horiz)))
            fovy = float(c.get("fovy_deg") or c.get("fovy") or 45.0)
            # Renderer uses model cam or free cam
            renderer.update_scene(data, camera=mjcam)
            rgb = renderer.render()
            return np.asarray(rgb).copy()

    def _loop(self) -> None:
        try:
            import mujoco
            from mujoco import viewer
        except Exception as exc:  # noqa: BLE001
            self._last_error = str(exc)
            self.status = "error"
            print(f"  MuJoCo 预览不可用: {exc}")
            return

        while not self._stop.is_set():
            try:
                model, data = self._compile()
            except Exception as exc:  # noqa: BLE001
                self._last_error = str(exc)
                self.status = "error"
                print(f"  MuJoCo 预览编译失败: {exc}")
                self._rebuild_event.clear()
                self._rebuild_event.wait(timeout=2.0)
                continue

            with self._state_lock:
                self._model, self._data = model, data
                self._renderer = None

            self.status = "running"
            self._rebuild_event.clear()
            # 挂载 base 可视化（contact=青 / urdf=品红）
            try:
                from teleop_birth.ui.session.base_frame_viz import get_base_frame_overlay

                scene_for_viz = _scene_cfg_for_viz(root, draft)
                get_base_frame_overlay().set_from_scene(scene_for_viz)
                get_base_frame_overlay().set_visible(True)
            except Exception:
                pass
            try:
                with viewer.launch_passive(model, data) as v:
                    print("  MuJoCo viewer 已启动（与 PyQt 工坊并用；改物品后点「刷新预览」）…")
                    print("  [base] 青=桌面接触 base，品红=转换后 URDF base，RGB=XYZ")
                    while v.is_running() and not self._stop.is_set():
                        if self._rebuild_event.is_set():
                            break
                        mujoco.mj_forward(model, data)
                        try:
                            from teleop_birth.ui.session.base_frame_viz import (
                                apply_viewer_overlays,
                            )

                            apply_viewer_overlays(v)
                        except Exception:
                            pass
                        v.sync()
                        time.sleep(0.02)
            except Exception as exc:  # noqa: BLE001
                self._last_error = str(exc)
                print(f"  MuJoCo viewer 异常: {exc}")
                # viewer 失败仍保留 model 供离屏渲染
                while not self._stop.is_set() and not self._rebuild_event.is_set():
                    time.sleep(0.1)
            if self._stop.is_set():
                break
            print("  正在按草稿重建 MuJoCo 预览…")
        self.status = "stopped"

    def _compile(self) -> tuple[Any, Any]:
        import mujoco

        draft = self.draft
        root = self.project_root
        if draft.robot_id == "ur5e":
            model = _compile_ur5e(root, draft)
        elif draft.robot_id in ("artixon_arm_6a", "artixon_6a"):
            model = _compile_artixon(root, draft)
        else:
            raise ValueError(
                f"机器人「{draft.robot_id}」尚未接入场景预览，"
                f"请改用 UR5e 或 ArtiXon Arm-6A（不要用占位 stub 生成空白场景）。"
            )

        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        nprop = len(draft.props or [])
        print(f"  预览编译完成: props={nprop} cameras={len(draft.cameras or [])}")
        return model, data


def _compile_ur5e(root: Path, draft: SceneDraft):
    from omegaconf import OmegaConf

    from teleop_birth.robot.arm.ur5e.builder import build_dual_ur5e_2f85

    backend = OmegaConf.to_container(
        OmegaConf.load(root / "configs/device/backend/mujoco_dual_ur.yaml"),
        resolve=True,
    )
    paths = OmegaConf.to_container(
        OmegaConf.load(root / "configs/device/paths/default.yaml"),
        resolve=True,
    )
    assert isinstance(backend, dict) and isinstance(paths, dict)
    scene = dict(backend["scene"])
    scene["desktop_base"] = {
        "arm_id": draft.desktop_base_arm,
        "table_top_z": draft.table_top_z,
        "xy": list(draft.base_xy),
    }
    scene["project_root"] = str(root)
    # 搭建预览：不预置场景物品，只使用草稿 props（可为空）
    scene["props"] = list(draft.props or [])
    scene["show_base_frames"] = True
    menagerie = root / str(paths.get("menagerie", "third_party/mujoco_menagerie"))
    apply_gripper_to_scene_cfg(root, scene, draft.gripper_id, menagerie=menagerie)
    warn = scene.pop("_gripper_warn", None)
    applied = scene.pop("_gripper_applied", None)
    if warn:
        print(f"  {warn}")
    elif applied:
        print(f"  已挂载夹爪: {applied} → {scene.get('gripper_xml')}")
    model, _ = build_dual_ur5e_2f85(menagerie, scene)
    return model


def _scene_cfg_for_viz(root: Path, draft: SceneDraft) -> dict[str, Any]:
    """给 base overlay 用的场景片段（含 base_x / desktop_base）。"""
    from omegaconf import OmegaConf

    backend = OmegaConf.to_container(
        OmegaConf.load(root / "configs/device/backend/mujoco_dual_ur.yaml"),
        resolve=True,
    )
    scene: dict[str, Any] = {}
    if isinstance(backend, dict):
        scene = dict(backend.get("scene") or {})
    scene["desktop_base"] = {
        "arm_id": draft.desktop_base_arm,
        "table_top_z": draft.table_top_z,
        "xy": list(draft.base_xy),
    }
    if draft.robot_id.startswith("artixon"):
        scene["base_x"] = None  # 单安装点
        scene["desktop_base"]["arm_id"] = "artixon_6a"
    return scene


def _compile_artixon(root: Path, draft: SceneDraft):
    from omegaconf import OmegaConf

    from teleop_birth.robot.arm.artixon.builder import build_logistics_scene

    scene_path = root / "configs/robots/artixon_logistics_scene.yaml"
    scene = OmegaConf.to_container(OmegaConf.load(scene_path), resolve=True)
    assert isinstance(scene, dict)
    scene["desktop_base"] = {
        "arm_id": "artixon_6a",
        "table_top_z": draft.table_top_z,
        "xy": list(draft.base_xy),
    }
    scene["project_root"] = str(root)
    # 场景搭建：空工作台（无传送带、无预置包裹），只留桌+双臂，物品由草稿 props 添加
    scene["packages"] = []
    scene["include_conveyors"] = False
    scene["props"] = list(draft.props or [])
    model, _ = build_logistics_scene(scene)
    return model


def _compile_empty_humanoid(root: Path, draft: SceneDraft):
    import mujoco

    from teleop_birth.scene.props.catalog import PropSpec
    from teleop_birth.scene.props.spawner import PropSpawner

    spec = mujoco.MjSpec()
    spec.modelname = f"preview_{draft.robot_id}"
    spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8])
    spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[5, 5, 0.1],
        rgba=[0.35, 0.38, 0.40, 1],
    )
    body = spec.worldbody.add_body(name="humanoid_proxy", pos=[0, 0, 0.55])
    body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_CAPSULE,
        size=[0.12, 0.28],
        rgba=[0.55, 0.6, 0.75, 1],
    )
    head = body.add_body(name="head_proxy", pos=[0, 0, 0.42])
    head.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1], rgba=[0.85, 0.75, 0.65, 1])
    body.add_site(name="base", pos=[0, 0, -0.55], size=[0.02])

    if draft.props:
        specs = [PropSpec.from_dict(dict(p)) for p in draft.props]
        PropSpawner(project_root=root).attach_all(spec, specs)

    return spec.compile()
