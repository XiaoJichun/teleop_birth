"""Build dual UR5e + Robotiq 2F-85 MuJoCo model — 全部几何/命名来自 scene_cfg。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List, Tuple

import mujoco


def _require(cfg: dict[str, Any], key: str) -> Any:
    if key not in cfg:
        raise KeyError(f"scene 配置缺少字段: {key}")
    return cfg[key]


def resolve_menagerie(menagerie: str | Path, scene_cfg: dict[str, Any]) -> Path:
    root = Path(menagerie)
    ur = root / str(_require(scene_cfg, "ur5e_xml"))
    if not ur.is_file():
        raise FileNotFoundError(
            f"UR 模型不存在:\n  {ur}\n请确认 paths.menagerie 与 backend.scene.ur5e_xml。"
        )
    grip_rel = str(_require(scene_cfg, "gripper_xml"))
    grip = Path(grip_rel)
    if not grip.is_file():
        grip = root / grip_rel
    if not grip.is_file():
        raise FileNotFoundError(
            f"夹爪模型不存在:\n  {grip}\n请确认 catalog / gripper_xml。"
        )
    return root


def _gripper_path(menagerie_root: Path, scene_cfg: dict[str, Any]) -> Path:
    grip_rel = str(_require(scene_cfg, "gripper_xml"))
    p = Path(grip_rel)
    if p.is_file():
        return p
    return menagerie_root / grip_rel


def _integrator(name: str) -> int:
    mapping = {
        "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
        "euler": mujoco.mjtIntegrator.mjINT_EULER,
        "rk4": mujoco.mjtIntegrator.mjINT_RK4,
    }
    if name not in mapping:
        raise ValueError(f"未知 integrator: {name}，可选 {list(mapping)}")
    return mapping[name]


def _cone(name: str) -> int:
    mapping = {
        "elliptic": mujoco.mjtCone.mjCONE_ELLIPTIC,
        "pyramidal": mujoco.mjtCone.mjCONE_PYRAMIDAL,
    }
    if name not in mapping:
        raise ValueError(f"未知 cone: {name}，可选 {list(mapping)}")
    return mapping[name]


def build_dual_ur5e_2f85(
    menagerie: str | Path,
    scene_cfg: dict[str, Any],
) -> Tuple[mujoco.MjModel, dict]:
    """Compile dual-arm scene from YAML scene_cfg."""
    root = resolve_menagerie(menagerie, scene_cfg)
    base_height = float(_require(scene_cfg, "base_height"))
    # 可选：用桌面 base 插件统一 mount_z（换臂时桌面高度不变）
    db = dict(scene_cfg.get("desktop_base") or {})
    if db.get("enabled", False) or db.get("arm_id"):
        from teleop_birth.teleop.frame.desktop_base import (
            get_desktop_base,
            mount_origin_z,
            table_top_z_from_scene,
        )

        arm_id = str(db.get("arm_id") or "ur5e")
        # 若已给 table_top_z 则直接用；否则从 table 字段反推，再算 mount
        if "table_top_z" in db:
            top = float(db["table_top_z"])
        else:
            # 先按旧公式估桌面，再反算：对 ur5e 通常 mount≈旧 base_height
            top = table_top_z_from_scene(scene_cfg)
        base_height = mount_origin_z(top, get_desktop_base(arm_id))
        if "origin_above_bottom_m" in db or "pedestal_height_m" in db:
            from teleop_birth.teleop.frame.desktop_base import DesktopBaseSpec

            spec = get_desktop_base(arm_id)
            base_height = mount_origin_z(
                top,
                DesktopBaseSpec(
                    id=spec.id,
                    title=spec.title,
                    origin_above_bottom_m=float(
                        db.get("origin_above_bottom_m", spec.origin_above_bottom_m)
                    ),
                    pedestal_height_m=float(db.get("pedestal_height_m", spec.pedestal_height_m)),
                    base_body=spec.base_body,
                    notes=spec.notes,
                ),
            )

    base_x = float(_require(scene_cfg, "base_x"))
    left_prefix = str(_require(scene_cfg, "left_prefix"))
    right_prefix = str(_require(scene_cfg, "right_prefix"))
    grip_prefix = str(_require(scene_cfg, "gripper_prefix"))
    attach_site = str(_require(scene_cfg, "gripper_attach_site"))
    arm_suffixes: List[str] = list(_require(scene_cfg, "arm_actuator_suffixes"))
    grip_suffix = str(_require(scene_cfg, "gripper_actuator_suffix"))
    tcp_suffix = str(_require(scene_cfg, "tcp_site_suffix"))

    light = dict(_require(scene_cfg, "light"))
    ground = dict(_require(scene_cfg, "ground"))
    table = dict(_require(scene_cfg, "table"))

    scene = mujoco.MjSpec()
    scene.modelname = str(_require(scene_cfg, "model_name"))
    scene.option.integrator = _integrator(str(_require(scene_cfg, "integrator")))
    scene.option.cone = _cone(str(_require(scene_cfg, "cone")))
    scene.option.impratio = float(_require(scene_cfg, "impratio"))

    scene.worldbody.add_light(
        pos=list(_require(light, "pos")),
        dir=list(_require(light, "dir")),
        diffuse=list(_require(light, "diffuse")),
    )
    scene.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=list(_require(ground, "size")),
        rgba=list(_require(ground, "rgba")),
    )
    table_pos = list(_require(table, "pos_xy")) + [
        base_height + float(_require(table, "z_offset_from_base"))
    ]
    scene.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=list(_require(table, "size")),
        pos=table_pos,
        rgba=list(_require(table, "rgba")),
    )

    ur_rel = str(_require(scene_cfg, "ur5e_xml"))
    grip_path = _gripper_path(root, scene_cfg)
    for x, pref in [(-base_x, left_prefix), (base_x, right_prefix)]:
        arm = mujoco.MjSpec.from_file(str(root / ur_rel))
        grip = mujoco.MjSpec.from_file(str(grip_path))
        arm.attach(grip, prefix=grip_prefix, site=arm.site(attach_site))
        frame = scene.worldbody.add_frame(pos=[x, 0.0, base_height])
        scene.attach(arm, prefix=pref, frame=frame)

    # 桌面接触 base（青）与转换后 URDF base（品红）标记
    if bool(scene_cfg.get("show_base_frames", True)):
        try:
            from teleop_birth.ui.session.base_frame_viz import add_base_frame_geoms_to_spec

            n_viz = add_base_frame_geoms_to_spec(scene, scene_cfg)
            if n_viz:
                print(f"  [base] 已加入 {n_viz} 个 base 坐标系标记（青=接触 / 品红=URDF）")
        except Exception as exc:  # noqa: BLE001
            print(f"  [base] 标记跳过: {exc}")

    prop_names: List[str] = []
    raw_props = scene_cfg.get("props") or []
    if raw_props:
        from teleop_birth.scene.props.catalog import PropSpec
        from teleop_birth.scene.props.spawner import PropSpawner

        project_root = Path(scene_cfg.get("project_root") or Path(menagerie).resolve().parents[1])
        specs = [
            p if isinstance(p, PropSpec) else PropSpec.from_dict(dict(p))
            for p in raw_props
        ]
        prop_names = PropSpawner(project_root=project_root).attach_all(scene, specs)

    model = scene.compile()
    name_map = {
        "left_arm_actuators": [f"{left_prefix}{s}" for s in arm_suffixes],
        "right_arm_actuators": [f"{right_prefix}{s}" for s in arm_suffixes],
        "left_gripper_actuator": f"{left_prefix}{grip_suffix}",
        "right_gripper_actuator": f"{right_prefix}{grip_suffix}",
        "left_tcp_site": f"{left_prefix}{tcp_suffix}",
        "right_tcp_site": f"{right_prefix}{tcp_suffix}",
        "left_home": list(_require(scene_cfg, "left_home")),
        "right_home": list(_require(scene_cfg, "right_home")),
        "settle_steps": int(_require(scene_cfg, "settle_steps")),
        "prop_names": prop_names,
    }
    return model, name_map


def actuator_ids(model: mujoco.MjModel, names: list[str]) -> list[int]:
    ids = []
    for name in names:
        i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if i < 0:
            raise KeyError(f"actuator not found: {name}")
        ids.append(i)
    return ids


def site_id(model: mujoco.MjModel, name: str) -> int:
    i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if i < 0:
        raise KeyError(f"site not found: {name}")
    return i
