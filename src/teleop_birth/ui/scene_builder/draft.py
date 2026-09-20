"""场景草稿：机器人 + 夹爪 + 桌面 base + 物品 + 相机 → 可保存 YAML。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

import yaml

SceneMode = Literal["semantic", "catalog"]
MountKind = Literal["table", "empty"]  # arm→桌子；人形→空白


@dataclass
class SceneDraft:
    """交互式搭建中的场景草稿。"""

    mode: SceneMode
    robot_id: str
    robot_title: str
    robot_category: str
    mount_kind: MountKind
    gripper_id: str = ""
    gripper_title: str = ""
    desktop_base_arm: str = "ur5e"
    table_top_z: float = 0.42
    base_xy: list[float] = field(default_factory=lambda: [0.0, 0.0])
    props: list[dict[str, Any]] = field(default_factory=list)
    cameras: list[dict[str, Any]] = field(default_factory=list)
    frame: dict[str, Any] = field(default_factory=lambda: {"arm_profile": "mujoco_table_zup"})
    notes: str = ""

    def to_scene_yaml(self, scene_id: str, title: str | None = None) -> dict[str, Any]:
        stype = "dual_ur5e" if self.robot_id == "ur5e" else (
            "logistics_x2arm" if self.robot_id.startswith("artixon") else "custom"
        )
        cams = list(self.cameras) or [
            {
                "name": "third_person",
                "type": "world",
                "role": "third_person",
                "camera_asset": "realsense_d455",
                "pos": [0.55, -0.75, 1.15],
                "lookat": [0.15, 0.35, 0.45],
            },
            {
                "name": "wrist_right",
                "type": "site",
                "role": "wrist",
                "camera_asset": "realsense_d405",
                "mount": "wrist_camera_bracket",
                "site": "R_g_pinch",
            },
        ]
        grippers = []
        if self.gripper_id:
            grippers = [
                {"id": self.gripper_id, "side": "right"},
                {"id": self.gripper_id, "side": "left"},
            ]
        return {
            "id": scene_id,
            "type": stype,
            "title": title or f"{self.robot_title} + {self.gripper_title or 'ee'}",
            "build_mode": self.mode,
            "backend": {
                "type": "mujoco_dual_ur" if stype == "dual_ur5e" else (
                    "logistics_x2arm" if stype == "logistics_x2arm" else "custom"
                ),
                "robot_id": self.robot_id,
            },
            "mount": {
                "kind": self.mount_kind,
                "table_top_z": self.table_top_z if self.mount_kind == "table" else None,
            },
            "grippers": grippers,
            "camera": {
                "wrist_asset": "realsense_d405",
                "third_person_asset": "realsense_d455",
                "default_mount": "wrist_camera_bracket",
            },
            "cameras": cams,
            "props": [p.get("name") for p in self.props if p.get("name")],
            "prop_specs": list(self.props),
            "frame": dict(self.frame),
            "desktop_base": {
                "arm_id": self.desktop_base_arm or self.robot_id,
                "table_top_z": self.table_top_z,
                "xy": list(self.base_xy),
                "yaw_rad": 0.0,
            },
            "teleop": {"enabled": True},
            "collect": {
                "policy": "pick_place",
                "task": "right_pick_red_cube_to_blue_plate",
                "arm": "right",
            },
            "notes": self.notes,
            "created_utc": datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
        }


def user_scenes_dir(project_root: Path) -> Path:
    d = Path(project_root) / "configs" / "scenes" / "user"
    d.mkdir(parents=True, exist_ok=True)
    return d


def save_scene_draft(
    project_root: Path,
    draft: SceneDraft,
    *,
    scene_id: str,
    title: str | None = None,
) -> Path:
    sid = "".join(c if c.isalnum() or c in "-_" else "_" for c in scene_id.strip()) or "scene"
    path = user_scenes_dir(project_root) / f"{sid}.yaml"
    data = draft.to_scene_yaml(sid, title=title)
    path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    # 同步自定义物品到 props 目录，便于后续采集/随机化按名解析
    props_dir = Path(project_root) / "configs" / "scene" / "props" / "user"
    props_dir.mkdir(parents=True, exist_ok=True)
    for p in draft.props:
        name = str(p.get("name") or "").strip()
        if not name:
            continue
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in name)
        (props_dir / f"{safe}.yaml").write_text(
            yaml.safe_dump(p, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
    return path


def list_built_scenes(project_root: Path) -> list[Path]:
    """已搭建场景 = configs/scenes/*.yaml + user/。"""
    root = Path(project_root) / "configs" / "scenes"
    out: list[Path] = []
    if root.is_dir():
        out.extend(sorted(root.glob("*.yaml")))
        user = root / "user"
        if user.is_dir():
            out.extend(sorted(user.glob("*.yaml")))
    return out
