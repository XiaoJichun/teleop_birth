"""从 Hydra/OmegaConf 配置组装遥操全链路（无硬编码参数）。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any

from omegaconf import DictConfig, OmegaConf

from birth_teleop.backend import G1BackendStub, MockDualArmBackend, MujocoDualURBackend, RobotBackend
from birth_teleop.mapping import DualArmMapper, HumanoidMapper, Mapper
from birth_teleop.pipeline import TeleopLoop
from birth_teleop.record import SessionRecorder
from birth_teleop.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, identity_pose
from birth_teleop.xr import KeyboardClutchMockXRSource, XRSource, XRoboSource
import numpy as np


def _to_dict(cfg: Any) -> dict[str, Any]:
    if isinstance(cfg, DictConfig):
        return OmegaConf.to_container(cfg, resolve=True)  # type: ignore[return-value]
    if isinstance(cfg, dict):
        return dict(cfg)
    raise TypeError(f"期望 dict/DictConfig，得到 {type(cfg)}")


def _require(cfg: dict[str, Any], key: str) -> Any:
    if key not in cfg:
        raise KeyError(f"配置缺少必需字段: {key}")
    return cfg[key]


def build_source(cfg: DictConfig | dict, project_root: Path) -> XRSource:
    c = _to_dict(cfg)
    kind = str(_require(c, "type")).lower()
    if kind == "mock":
        src = KeyboardClutchMockXRSource(
            hz=float(_require(c, "hz")),
            radius=float(_require(c, "radius")),
        )
        src.left_grip = float(_require(c, "left_grip"))
        src.right_grip = float(_require(c, "right_grip"))
        return src
    if kind == "xrobo":
        from birth_teleop.xr.env import apply_xrobo_library_path
        from birth_teleop.xr.sources import check_xrobo_sdk_available

        apply_xrobo_library_path(project_root)
        ok, msg = check_xrobo_sdk_available(project_root)
        if not ok:
            raise ImportError(msg)
        return XRoboSource(
            include_body=bool(_require(c, "include_body")),
            setup_config=str(_require(c, "setup_config")),
            project_root=project_root,
        )
    raise ValueError(f"未知 source.type: {kind}")


def build_mapper(cfg: DictConfig | dict, *, scale_override: float | None = None) -> Mapper:
    c = _to_dict(cfg)
    kind = str(_require(c, "type")).lower()

    def _dual_kwargs(src: dict[str, Any], scale: float) -> dict[str, Any]:
        ws = src.get("workspace_center")
        return dict(
            scale=scale,
            filter_alpha=float(_require(src, "filter_alpha")),
            map_head=bool(_require(src, "map_head")),
            grip_threshold=float(_require(src, "grip_threshold")),
            clutch_mode=str(src.get("clutch_mode", "long_press_grip")),
            hold_s=float(src.get("hold_s", 0.35)),
            position_only=bool(src.get("position_only", True)),
            max_step_m=float(src.get("max_step_m", 0.03)),
            workspace_center=dict(ws) if isinstance(ws, dict) else None,
            workspace_radius_m=float(src.get("workspace_radius_m", 0.55)),
            reset_buttons_left=list(src.get("reset_buttons_left") or ["y", "left_menu"]),
            reset_buttons_right=list(src.get("reset_buttons_right") or ["b", "right_menu"]),
        )

    if kind == "dual_arm":
        scale = float(scale_override if scale_override is not None else _require(c, "scale"))
        return DualArmMapper(**_dual_kwargs(c, scale))
    if kind == "humanoid":
        dual_cfg = dict(_require(c, "dual_arm"))
        scale = float(
            scale_override if scale_override is not None else _require(dual_cfg, "scale")
        )
        dual = DualArmMapper(**_dual_kwargs(dual_cfg, scale))
        return HumanoidMapper(
            dual_arm=dual,
            body_joint_indices=dict(_require(c, "body_joint_indices")),
            tracker_left_keywords=list(_require(c, "tracker_left_keywords")),
            tracker_right_keywords=list(_require(c, "tracker_right_keywords")),
        )
    raise ValueError(f"未知 mapper.type: {kind}")


def build_backend(
    cfg: DictConfig | dict,
    ee_cfg: DictConfig | dict,
    *,
    menagerie: Path,
    render: bool,
) -> RobotBackend:
    c = _to_dict(cfg)
    e = _to_dict(ee_cfg)
    kind = str(_require(c, "type")).lower()

    if kind == "mock_dual_arm":
        return MockDualArmBackend(
            left0=np.asarray(_require(c, "left_ee0"), dtype=np.float64),
            right0=np.asarray(_require(c, "right_ee0"), dtype=np.float64),
            head0=np.asarray(_require(c, "head0"), dtype=np.float64),
        )

    if kind == "g1_stub":
        return G1BackendStub(mode=str(_require(c, "mode")))

    if kind == "mujoco_dual_ur":
        scene = dict(_require(c, "scene"))
        ik = dict(_require(c, "ik"))
        return MujocoDualURBackend(
            menagerie=menagerie,
            scene_cfg=scene,
            ik_cfg=ik,
            ee_cfg=e,
            render=bool(render if render is not None else c.get("render", False)),
            sim_substeps=int(c.get("sim_substeps", 1)),
        )

    raise ValueError(f"未知 backend.type: {kind}")


def build_recorder(runtime: dict[str, Any], paths: dict[str, Any], project_root: Path) -> SessionRecorder | None:
    if not bool(runtime.get("log_enabled", False)):
        return None
    log_path = str(runtime.get("log_path") or "").strip()
    if not log_path:
        logs_dir = project_root / str(_require(paths, "logs_dir"))
        logs_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path = str(logs_dir / f"teleop_{stamp}.jsonl")
    return SessionRecorder(log_path)


def build_teleop_loop(cfg: DictConfig, project_root: Path) -> tuple[TeleopLoop, dict[str, Any]]:
    """返回 (loop, meta)；meta 含 duration 等运行信息。"""
    runtime = _to_dict(cfg.runtime)
    paths = _to_dict(cfg.paths)

    menagerie = (project_root / str(_require(paths, "menagerie"))).resolve()
    source = build_source(cfg.source, project_root=project_root)
    mapper = build_mapper(cfg.mapper, scale_override=float(runtime["scale"]))
    backend = build_backend(
        cfg.backend,
        cfg.end_effector,
        menagerie=menagerie,
        render=bool(runtime["render"]),
    )
    recorder = build_recorder(runtime, paths, project_root)

    print_every = int(runtime["print_every"])

    def on_step(obs, target, state):
        if on_step.i % print_every == 0:
            le = state.extremities.get(EXTREMITY_LEFT_EE, identity_pose())
            re = state.extremities.get(EXTREMITY_RIGHT_EE, identity_pose())
            xl = obs.left_controller
            xr = obs.right_controller
            print(
                f"[{on_step.i:04d}] "
                f"robot L=({le[0]:+.3f},{le[1]:+.3f},{le[2]:+.3f}) "
                f"R=({re[0]:+.3f},{re[1]:+.3f},{re[2]:+.3f}) "
                f"xrL=({xl[0]:+.3f},{xl[1]:+.3f},{xl[2]:+.3f}) "
                f"active=({target.is_active(EXTREMITY_LEFT_EE)},"
                f"{target.is_active(EXTREMITY_RIGHT_EE)}) "
                f"stream={'OK' if getattr(obs, 'stream_ok', True) else 'DOWN'} "
                f"loop={on_step.last_loop_ms:.1f}ms"
            )
            if on_step.prev_xl is not None:
                d = float(np.linalg.norm(xl[:3] - on_step.prev_xl[:3]))
                if d < 1e-4 and on_step.i > print_every * 4:
                    if not on_step.stale_warned:
                        print(
                            "[hint] XR 位姿几乎不动：请在 XRoboToolkit 打开 Send，"
                            "并确认状态为 WORKING；移动手柄后 xrL 应变。"
                        )
                        on_step.stale_warned = True
                elif d >= 1e-4:
                    on_step.stale_warned = False
            on_step.prev_xl = xl.copy()
        on_step.i += 1

    on_step.i = 0
    on_step.prev_xl = None
    on_step.stale_warned = False
    on_step.last_loop_ms = 0.0

    loop = TeleopLoop(
        source=source,
        mapper=mapper,
        backend=backend,
        hz=float(runtime["hz"]),
        latency_window=int(runtime["latency_window"]),
        recorder=recorder,
        on_step=on_step,
    )
    meta = {
        "duration_s": float(runtime["duration_s"]),
        "interactive_confirm": bool(runtime.get("interactive_confirm", False)),
        "log_path": getattr(recorder, "path", None),
    }
    return loop, meta
