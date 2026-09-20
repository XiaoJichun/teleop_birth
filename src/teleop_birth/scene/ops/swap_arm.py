"""场景内无损换臂：桌面接触点固定，只换 embodiment / 腕部相机挂点。

约定（与 desktop_base 插件一致）：
  T_world_contact = Trans(xy, table_top_z) @ Rot_z(yaw)   ← 换臂时不变
  T_contact_urdf  = Trans(0, 0, pedestal + …)              ← 随 arm_id 变
  T_world_urdf    = T_world_contact @ T_contact_urdf

因此「base 固定在机械臂底座与桌面交汇处」= 保留 table_top_z / xy / yaw，
仅改 desktop_base.arm_id，并同步 type / backend / grippers / 腕部 site。
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from teleop_birth.teleop.frame.desktop_base import get_desktop_base, mount_origin_z


# robot_id → desktop_bases.yaml 中的 arm_id
_DESKTOP_ARM = {
    "ur5e": "ur5e",
    "artixon_arm_6a": "artixon_6a",
    "artixon_6a": "artixon_6a",
    "franka_panda": "franka_panda",
}

# 各 ready 臂的场景模板字段（不含接触点 / 物品）
_ARM_TEMPLATES: dict[str, dict[str, Any]] = {
    "ur5e": {
        "type": "dual_ur5e",
        "backend": {"type": "mujoco_dual_ur", "robot_id": "ur5e"},
        "grippers": [
            {"id": "robotiq_2f85", "side": "left"},
            {"id": "robotiq_2f85", "side": "right"},
        ],
        "wrist_sites": {"left": "L_g_pinch", "right": "R_g_pinch"},
        "sensors": [],
        "physics": {"anti_penetration": False},
        "collect_policy": "pick_place",
        "module_config": None,
    },
    "artixon_arm_6a": {
        "type": "logistics_x2arm",
        "backend": {
            "type": "logistics_x2arm",
            "robot_id": "artixon_arm_6a",
            "module_config": "configs/robots/artixon_logistics_scene.yaml",
        },
        "grippers": [
            {"id": "artixon_parallel", "side": "left", "builtin": True},
            {"id": "artixon_parallel", "side": "right", "builtin": True},
        ],
        "wrist_sites": {"left": "L_tcp_pinch", "right": "R_tcp_pinch"},
        "sensors": [],  # DIGIT 可选；用户场景默认不强制
        "physics": {"anti_penetration": True, "min_separation_m": 0.08},
        "collect_policy": "pick_place",
        "module_config": "configs/robots/artixon_logistics_scene.yaml",
    },
}


@dataclass(frozen=True)
class ContactPose:
    """桌面接触系（换臂不变量）。"""

    table_top_z: float
    xy: tuple[float, float]
    yaw_rad: float


@dataclass(frozen=True)
class SwapResult:
    scene: dict[str, Any]
    contact: ContactPose
    old_arm_id: str
    new_arm_id: str
    old_robot_id: str
    new_robot_id: str
    mount_z_old: float
    mount_z_new: float


def desktop_arm_id(robot_id: str) -> str:
    return _DESKTOP_ARM.get(robot_id, robot_id)


def supported_robot_ids() -> list[str]:
    return sorted(_ARM_TEMPLATES)


def extract_contact(scene: dict[str, Any]) -> ContactPose:
    db = dict(scene.get("desktop_base") or {})
    mount = dict(scene.get("mount") or {})
    top = db.get("table_top_z")
    if top is None:
        top = mount.get("table_top_z")
    if top is None:
        top = scene.get("table_top_z")
    if top is None:
        top = 0.42
    xy_raw = db.get("xy") or [0.0, 0.0]
    xy = (float(xy_raw[0]), float(xy_raw[1]))
    yaw = float(db.get("yaw_rad") or 0.0)
    return ContactPose(table_top_z=float(top), xy=xy, yaw_rad=yaw)


def infer_robot_id(scene: dict[str, Any]) -> str:
    backend = dict(scene.get("backend") or {})
    rid = str(backend.get("robot_id") or "").strip()
    if rid:
        return rid
    arm = str((scene.get("desktop_base") or {}).get("arm_id") or "").strip()
    if arm in ("artixon_6a", "artixon_arm_6a"):
        return "artixon_arm_6a"
    if arm == "ur5e":
        return "ur5e"
    stype = str(scene.get("type") or "")
    if "artixon" in stype or stype == "logistics_x2arm":
        return "artixon_arm_6a"
    if "ur" in stype:
        return "ur5e"
    return "ur5e"


def _remap_cameras(
    cameras: list[dict[str, Any]],
    *,
    wrist_sites: dict[str, str],
) -> list[dict[str, Any]]:
    """保留第三人称位姿；腕部相机改挂到新臂 TCP site。"""
    out: list[dict[str, Any]] = []
    seen_wrist: set[str] = set()
    for cam in cameras:
        c = deepcopy(cam)
        role = str(c.get("role") or "")
        name = str(c.get("name") or "")
        if role == "third_person" or c.get("type") == "world":
            out.append(c)
            continue
        if role == "wrist" or "wrist" in name:
            side = "left" if "left" in name.lower() else "right"
            site = wrist_sites.get(side)
            if not site:
                continue
            c["type"] = "site"
            c["role"] = "wrist"
            c["name"] = f"wrist_{side}"
            c["mount"] = c.get("mount") or "wrist_camera_bracket"
            c["camera_asset"] = c.get("camera_asset") or "realsense_d405"
            c["site"] = site
            out.append(c)
            seen_wrist.add(side)
            continue
        out.append(c)

    # 原场景无腕部相机时，按模板补默认挂点
    if not seen_wrist:
        return _default_cameras(wrist_sites)
    return out


def _default_cameras(wrist_sites: dict[str, str]) -> list[dict[str, Any]]:
    cams: list[dict[str, Any]] = [
        {
            "name": "third_person",
            "type": "world",
            "role": "third_person",
            "camera_asset": "realsense_d455",
            "pos": [0.55, -0.75, 1.15],
            "lookat": [0.15, 0.35, 0.45],
        }
    ]
    if wrist_sites.get("left"):
        cams.append(
            {
                "name": "wrist_left",
                "type": "site",
                "role": "wrist",
                "camera_asset": "realsense_d405",
                "mount": "wrist_camera_bracket",
                "site": wrist_sites["left"],
            }
        )
    if wrist_sites.get("right"):
        cams.append(
            {
                "name": "wrist_right",
                "type": "site",
                "role": "wrist",
                "camera_asset": "realsense_d405",
                "mount": "wrist_camera_bracket",
                "site": wrist_sites["right"],
            }
        )
    return cams


def swap_scene_robot(
    scene: dict[str, Any],
    *,
    new_robot_id: str,
    title_suffix: bool = True,
) -> SwapResult:
    """返回换臂后的场景 dict；接触点 (table_top_z, xy, yaw) 不变。"""
    if new_robot_id not in _ARM_TEMPLATES:
        raise KeyError(
            f"暂不支持换到 {new_robot_id}；可选: {supported_robot_ids()}"
        )
    tmpl = _ARM_TEMPLATES[new_robot_id]
    contact = extract_contact(scene)
    old_robot = infer_robot_id(scene)
    old_arm = str((scene.get("desktop_base") or {}).get("arm_id") or desktop_arm_id(old_robot))
    new_arm = desktop_arm_id(new_robot_id)

    out = deepcopy(scene)
    out["type"] = tmpl["type"]
    out["backend"] = dict(tmpl["backend"])
    out["grippers"] = [dict(g) for g in tmpl["grippers"]]
    out["sensors"] = [dict(s) for s in tmpl.get("sensors") or []]
    if tmpl.get("physics") is not None:
        out["physics"] = dict(tmpl["physics"])

    # 相机：保留第三人称，腕部改 site
    old_cams = list(scene.get("cameras") or [])
    if old_cams:
        out["cameras"] = _remap_cameras(old_cams, wrist_sites=tmpl["wrist_sites"])
    else:
        out["cameras"] = _default_cameras(tmpl["wrist_sites"])
    out["camera"] = {
        "wrist_asset": "realsense_d405",
        "third_person_asset": "realsense_d455",
        "default_mount": "wrist_camera_bracket",
    }

    # 接触点固定；只换 arm_id
    out["desktop_base"] = {
        "arm_id": new_arm,
        "table_top_z": contact.table_top_z,
        "xy": [contact.xy[0], contact.xy[1]],
        "yaw_rad": contact.yaw_rad,
    }
    mount = dict(out.get("mount") or {})
    mount["kind"] = mount.get("kind") or "table"
    mount["table_top_z"] = contact.table_top_z
    out["mount"] = mount

    # frame：保留原 arm_profile（仿真桌面 Z-up）
    frame = dict(out.get("frame") or {})
    frame.setdefault("arm_profile", "mujoco_table_zup")
    out["frame"] = frame

    # collect：保留任务；策略可随臂微调
    collect = dict(out.get("collect") or {})
    collect.setdefault("policy", tmpl.get("collect_policy") or "pick_place")
    collect.setdefault("arm", "right")
    out["collect"] = collect

    teleop = dict(out.get("teleop") or {})
    teleop["enabled"] = True
    if new_robot_id.startswith("artixon"):
        teleop.setdefault("config", "configs/robots/artixon_teleop.yaml")
    else:
        teleop.pop("config", None)
    out["teleop"] = teleop

    # 标题 / 元数据
    if title_suffix:
        base_title = str(scene.get("title") or scene.get("id") or "scene")
        # 去掉旧的「换臂」后缀再追加
        for marker in ("｜换臂→", "|换臂→"):
            if marker in base_title:
                base_title = base_title.split(marker)[0].rstrip()
        out["title"] = f"{base_title}｜换臂→{new_robot_id}"
    out["swapped_utc"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    out["swap_note"] = (
        f"desktop contact fixed: table_top_z={contact.table_top_z}, "
        f"xy={list(contact.xy)}, yaw={contact.yaw_rad}; "
        f"arm_id {old_arm}→{new_arm}"
    )

    # props / prop_specs / id 保持原样（调用方可改 id）
    out.setdefault("props", list(scene.get("props") or []))
    if scene.get("prop_specs"):
        out["prop_specs"] = deepcopy(scene["prop_specs"])

    mz_old = mount_origin_z(contact.table_top_z, get_desktop_base(old_arm))
    mz_new = mount_origin_z(contact.table_top_z, get_desktop_base(new_arm))
    return SwapResult(
        scene=out,
        contact=contact,
        old_arm_id=old_arm,
        new_arm_id=new_arm,
        old_robot_id=old_robot,
        new_robot_id=new_robot_id,
        mount_z_old=mz_old,
        mount_z_new=mz_new,
    )


def write_scene_yaml(path: Path, scene: dict[str, Any]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(scene, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return path
