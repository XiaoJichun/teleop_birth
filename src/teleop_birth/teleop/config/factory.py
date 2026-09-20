"""从 Hydra 配置组装遥操全链路（模块化工厂）。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from omegaconf import DictConfig, OmegaConf

from teleop_birth.robot import (
    G1BackendStub,
    MockDualArmBackend,
    MujocoDualURBackend,
    RobotBackend,
)
from teleop_birth.teleop.mapping import DualArmMapper, HumanoidMapper, Mapper
from teleop_birth.teleop.pipeline import TeleopLoop
from teleop_birth.teleop.plot import attach_trace, plotter_from_cfg
from teleop_birth.teleop.record import SessionRecorder
from teleop_birth.robot.io import BackendAsEndpoint, EndpointBackendAdapter
from teleop_birth.teleop.schema import EXTREMITY_LEFT_EE, EXTREMITY_RIGHT_EE, identity_pose
from teleop_birth.teleop.xr import KeyboardClutchMockXRSource, XRSource, XRoboSource


def _pose_delta(prev: np.ndarray | None, pose: np.ndarray) -> tuple[float, float]:
    if prev is None:
        return 0.0, 0.0
    d_mm = float(np.linalg.norm(pose[:3] - prev[:3]) * 1000.0)
    dot = abs(float(np.dot(prev[3:], pose[3:])))
    d_deg = float(np.degrees(2.0 * np.arccos(float(np.clip(dot, -1.0, 1.0)))))
    return d_mm, d_deg


def shared_loop_hz(project_root: Path) -> float:
    """控制环频率的唯一入口。场景和机型配置不要再各自写 hz。"""
    path = project_root / "configs" / "algorithm" / "runtime" / "default.yaml"
    raw = OmegaConf.load(path)
    return float(raw.hz)


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
        from teleop_birth.teleop.xr.env import apply_xrobo_library_path
        from teleop_birth.teleop.xr.sources import check_xrobo_sdk_available

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
            max_step_rad=float(src.get("max_step_rad", 0.25)),
            workspace_center=dict(ws) if isinstance(ws, dict) else None,
            workspace_radius_m=float(src.get("workspace_radius_m", 0.55)),
            reset_buttons_left=list(src.get("reset_buttons_left") or ["y", "left_menu"]),
            reset_buttons_right=list(src.get("reset_buttons_right") or ["b", "right_menu"]),
            xr_align=str(src.get("xr_align", "openxr_yup_to_zup")),
            xr_align_matrix=src.get("xr_align_matrix"),
            arm_profile=src.get("arm_profile"),
            xr_mapping=dict(src["xr_mapping"]) if isinstance(src.get("xr_mapping"), dict) else None,
            controller_quat_xyzw=src.get("controller_quat_xyzw"),
            grip_to_tcp=src.get("grip_to_tcp"),
            grip_to_tcp_matrix=src.get("grip_to_tcp_matrix"),
            grip_to_tcp_mapping=(
                dict(src["grip_to_tcp_mapping"])
                if isinstance(src.get("grip_to_tcp_mapping"), dict)
                else None
            ),
            gripper_gamma=float(src.get("gripper_gamma", 0.65)),
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
    ik_cfg: DictConfig | dict | None,
    *,
    menagerie: Path,
    render: bool,
) -> RobotBackend:
    """组装 embodiment，并包成 EndpointBackendAdapter（双端口对外）。"""
    c = _to_dict(cfg)
    e = _to_dict(ee_cfg)
    ik = _to_dict(ik_cfg) if ik_cfg is not None else {}
    kind = str(_require(c, "type")).lower()
    port_kind = str(c.get("kind") or "sim")

    if kind == "mock_dual_arm":
        raw: RobotBackend = MockDualArmBackend(
            left0=np.asarray(_require(c, "left_ee0"), dtype=np.float64),
            right0=np.asarray(_require(c, "right_ee0"), dtype=np.float64),
            head0=np.asarray(_require(c, "head0"), dtype=np.float64),
        )
        port_kind = "mock"
    elif kind == "g1_stub":
        raw = G1BackendStub(mode=str(_require(c, "mode")))
        port_kind = "real"
    elif kind == "mujoco_dual_ur":
        scene = dict(_require(c, "scene"))
        raw = MujocoDualURBackend(
            menagerie=menagerie,
            scene_cfg=scene,
            ik_cfg=ik,
            ee_cfg=e,
            render=bool(render if render is not None else c.get("render", False)),
            sim_substeps=int(c.get("sim_substeps", 1)),
        )
        port_kind = "sim"
    else:
        raise ValueError(f"未知 backend.type: {kind}")

    # 统一暴露 StateReceiver + ActionPublisher
    return EndpointBackendAdapter(BackendAsEndpoint(raw, kind=port_kind))


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
    runtime = _to_dict(cfg.runtime)
    paths = _to_dict(cfg.paths)

    menagerie = (project_root / str(_require(paths, "menagerie"))).resolve()
    source = build_source(cfg.source, project_root=project_root)
    mapper = build_mapper(cfg.mapper, scale_override=float(runtime["scale"]))
    ik_cfg = cfg.ik if hasattr(cfg, "ik") else {}

    # 会话计划：注入物品到 scene 后再 build
    from teleop_birth.teleop.plan_rounds import (
        TeleopRoundManager,
        inject_plan_props_into_scene,
        load_plan_from_env,
    )

    plan = load_plan_from_env()
    backend_cfg = cfg.backend
    if plan and str(_to_dict(backend_cfg).get("type", "")).lower() == "mujoco_dual_ur":
        # 可变拷贝 scene
        b = _to_dict(backend_cfg)
        scene = dict(b.get("scene") or {})
        scene = inject_plan_props_into_scene(scene, plan, project_root)
        b["scene"] = scene
        from omegaconf import OmegaConf

        backend_cfg = OmegaConf.create(b)

    backend = build_backend(
        backend_cfg,
        cfg.end_effector,
        ik_cfg,
        menagerie=menagerie,
        render=bool(runtime["render"]),
    )
    recorder = build_recorder(runtime, paths, project_root)
    plotter = plotter_from_cfg(
        runtime.get("plot"),
        project_root=project_root,
        loop_hz=float(runtime["hz"]),
    )
    attach_trace(backend, plotter)

    print_every = int(runtime["print_every"])
    ik_name = str(getattr(cfg, "ik", {}).get("solver", "dls") if hasattr(cfg, "ik") else "dls")
    try:
        ik_name = str(_to_dict(cfg.ik).get("solver", "dls"))
    except Exception:
        pass

    round_mgr: TeleopRoundManager | None = None
    if plan:
        round_mgr = TeleopRoundManager(plan, project_root)

    def on_step(obs, target, state):
        if round_mgr is not None:
            if not getattr(on_step, "_plan_attached", False):
                round_mgr.attach(backend)
                on_step._plan_attached = True  # type: ignore[attr-defined]
            round_mgr.on_obs(obs)
        if on_step.i % print_every == 0:
            le = state.extremities.get(EXTREMITY_LEFT_EE, identity_pose())
            re = state.extremities.get(EXTREMITY_RIGHT_EE, identity_pose())
            xl = obs.left_controller
            xr = obs.right_controller
            d_mm, d_deg = _pose_delta(on_step.prev_xr, xr)
            on_step.prev_xr = xr.copy()
            print(
                f"[{on_step.i:04d}] "
                f"robot L=({le[0]:+.3f},{le[1]:+.3f},{le[2]:+.3f}) "
                f"R=({re[0]:+.3f},{re[1]:+.3f},{re[2]:+.3f}) "
                f"xrL=({xl[0]:+.3f},{xl[1]:+.3f},{xl[2]:+.3f}) "
                f"xrR=({xr[0]:+.3f},{xr[1]:+.3f},{xr[2]:+.3f}) "
                f"dR={d_mm:.0f}mm/{d_deg:.0f}deg "
                f"grip=({obs.buttons.left_grip:.2f},{obs.buttons.right_grip:.2f}) "
                f"active=({target.is_active(EXTREMITY_LEFT_EE)},"
                f"{target.is_active(EXTREMITY_RIGHT_EE)}) "
                f"ik={ik_name} "
                f"stream={'OK' if getattr(obs, 'stream_ok', True) else 'DOWN'} "
                f"loop={on_step.last_loop_ms:.1f}ms"
            )
        on_step.i += 1

    on_step.i = 0
    on_step.prev_xr = None
    on_step.last_loop_ms = 0.0
    on_step._plan_attached = False  # type: ignore[attr-defined]

    loop = TeleopLoop(
        source=source,
        mapper=mapper,
        backend=backend,
        hz=float(runtime["hz"]),
        latency_window=int(runtime["latency_window"]),
        recorder=recorder,
        on_step=on_step,
        trace=plotter,
    )
    meta = {
        "duration_s": float(runtime["duration_s"]),
        "interactive_confirm": bool(runtime.get("interactive_confirm", False)),
        "log_path": getattr(recorder, "path", None),
        "ik_solver": ik_name,
        "teleop_plan": bool(plan),
    }
    return loop, meta
