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
    grip = root / str(_require(scene_cfg, "gripper_xml"))
    if not ur.is_file() or not grip.is_file():
        raise FileNotFoundError(
            f"模型文件不存在:\n  {ur}\n  {grip}\n"
            "请确认 paths.menagerie 与 backend.scene.*_xml 配置正确。"
        )
    return root


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
    grip_rel = str(_require(scene_cfg, "gripper_xml"))
    for x, pref in [(-base_x, left_prefix), (base_x, right_prefix)]:
        arm = mujoco.MjSpec.from_file(str(root / ur_rel))
        grip = mujoco.MjSpec.from_file(str(root / grip_rel))
        arm.attach(grip, prefix=grip_prefix, site=arm.site(attach_site))
        frame = scene.worldbody.add_frame(pos=[x, 0.0, base_height])
        scene.attach(arm, prefix=pref, frame=frame)

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
