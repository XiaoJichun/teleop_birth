"""桌面机械臂 base 统一：把各 URDF 的 base 落在桌面接触点。

约定（所有臂共用同一套「桌面接触坐标系」）：
  - 接触点 = 底座与桌面贴合处（可加 pedestal）
  - 接触系轴与房间系一致：+X 右、+Y 前/桌深、+Z 上
  - 不同 URDF 只差「原点相对接触点」的固定齐次变换 T_contact←urdf

  T_world_contact = Trans(x, y, table_top_z) @ Rot_z(yaw)
  T_contact_urdf  = Trans(0, 0, pedestal + origin_above_bottom)   # URDF 原点在接触系里
  T_world_urdf    = T_world_contact @ T_contact_urdf

换臂：只换 DesktopBaseSpec / arm_id；桌子与场景控制逻辑可不变。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import yaml

__all__ = [
    "DesktopBaseSpec",
    "get_desktop_base",
    "list_desktop_bases",
    "load_desktop_bases",
    "mount_origin_z",
    "mount_pose_xyz",
    "register_desktop_base",
    "table_top_z_from_scene",
    "T_contact_from_urdf",
    "T_contact_urdf",
    "T_urdf_contact",
    "T_urdf_from_contact",
    "T_world_contact",
    "T_world_urdf",
    "mount_pose7",
]


@dataclass(frozen=True)
class DesktopBaseSpec:
    """一台桌面臂的「URDF 原点 ↔ 桌面接触」几何。"""

    id: str
    title: str
    # URDF base 原点相对底座底面、沿安装后竖直向上的距离 (m)
    origin_above_bottom_m: float = 0.0
    # 底座与桌面之间的垫块/基座高度 (m)；0 = 直接坐桌
    pedestal_height_m: float = 0.0
    # 默认 base_link 名（真机/仿真查 body 时用）
    base_body: str = "base"
    notes: str = ""

    def origin_z(self, table_top_z: float) -> float:
        return mount_origin_z(table_top_z, self)

    def lift_m(self) -> float:
        """接触点 → URDF 原点沿接触系 +Z 的平移。"""
        return float(self.pedestal_height_m) + float(self.origin_above_bottom_m)


def mount_origin_z(table_top_z: float, spec: DesktopBaseSpec | str) -> float:
    """URDF base 原点应放在世界系的 Z。"""
    s = get_desktop_base(spec) if isinstance(spec, str) else spec
    return float(table_top_z) + s.lift_m()


def mount_pose_xyz(
    table_top_z: float,
    xy: Sequence[float],
    spec: DesktopBaseSpec | str,
) -> list[float]:
    """返回 [x, y, z] 挂载位姿平移。"""
    x, y = float(xy[0]), float(xy[1])
    return [x, y, mount_origin_z(table_top_z, spec)]


def _rotz(yaw_rad: float) -> np.ndarray:
    c, s = float(np.cos(yaw_rad)), float(np.sin(yaw_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def T_contact_urdf(spec: DesktopBaseSpec | str) -> np.ndarray:
    """T_contact←urdf：接触系下 URDF 原点的位姿（只沿 +Z 抬升，轴一致）。

    p_contact = T_contact_urdf @ p_urdf
    T_world_urdf = T_world_contact @ T_contact_urdf
    """
    s = get_desktop_base(spec) if isinstance(spec, str) else spec
    T = np.eye(4, dtype=np.float64)
    T[2, 3] = s.lift_m()
    return T


def T_urdf_contact(spec: DesktopBaseSpec | str) -> np.ndarray:
    """T_urdf←contact = inv(T_contact_urdf)：把接触系点变到 URDF 原点系。"""
    return np.linalg.inv(T_contact_urdf(spec))


# 别名（旧名 / 英文短语）
T_urdf_from_contact = T_contact_urdf
T_contact_from_urdf = T_urdf_contact


def T_world_contact(
    table_top_z: float,
    xy: Sequence[float],
    *,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    """桌面接触系在世界系中的位姿（轴与房间一致，原点在接触点）。"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _rotz(float(yaw_rad))
    T[0, 3] = float(xy[0])
    T[1, 3] = float(xy[1])
    T[2, 3] = float(table_top_z)
    return T


def T_world_urdf(
    table_top_z: float,
    xy: Sequence[float],
    spec: DesktopBaseSpec | str,
    *,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    """URDF base 原点在世界系中应处的齐次位姿（换臂只换 spec）。"""
    return T_world_contact(table_top_z, xy, yaw_rad=yaw_rad) @ T_contact_urdf(spec)


def mount_pose7(
    table_top_z: float,
    xy: Sequence[float],
    spec: DesktopBaseSpec | str,
    *,
    yaw_rad: float = 0.0,
) -> np.ndarray:
    """挂载用 7 维 pose（xyz + quat xyzw），与 schema 一致。"""
    from teleop_birth.teleop.schema import matrix_to_pose

    return matrix_to_pose(T_world_urdf(table_top_z, xy, spec, yaw_rad=yaw_rad))


def table_top_z_from_scene(scene_cfg: Mapping[str, Any]) -> float:
    """从常见 scene 字段推断桌面顶面高度。

    支持：
      - table_top_z
      - desktop_base.table_top_z
      - table_z + table_size[2]（artixon）
      - base_height + table.z_offset_from_base + table.size[2]（dual UR）
    """
    if "table_top_z" in scene_cfg:
        return float(scene_cfg["table_top_z"])

    db = dict(scene_cfg.get("desktop_base") or {})
    if "table_top_z" in db:
        return float(db["table_top_z"])

    if "table_z" in scene_cfg:
        table_z = float(scene_cfg["table_z"])
        size = list(scene_cfg.get("table_size") or [0.55, 0.42, 0.02])
        return table_z + float(size[2])

    # dual UR 风格：必须显式有 table 或 base_height，避免空 scene 落到 0.02
    has_table = "table" in scene_cfg and isinstance(scene_cfg.get("table"), Mapping)
    has_base = "base_height" in scene_cfg
    if has_table or has_base:
        base_height = float(scene_cfg.get("base_height", 0.0))
        table = dict(scene_cfg.get("table") or {})
        z_off = float(table.get("z_offset_from_base", 0.0))
        size = list(table.get("size") or [0.6, 0.4, 0.02])
        return base_height + z_off + float(size[2])

    raise KeyError(
        "无法推断 table_top_z：请在 scene 写 table_top_z / table+base_height / desktop_base.table_top_z"
    )


_REGISTRY: dict[str, DesktopBaseSpec] = {}
_LOADED = False


def register_desktop_base(spec: DesktopBaseSpec) -> DesktopBaseSpec:
    _REGISTRY[spec.id] = spec
    return spec


def get_desktop_base(arm_id: str) -> DesktopBaseSpec:
    _ensure()
    if arm_id not in _REGISTRY:
        raise KeyError(f"未知桌面 base 剖面: {arm_id}（已注册 {list(_REGISTRY)}）")
    return _REGISTRY[arm_id]


def list_desktop_bases() -> list[DesktopBaseSpec]:
    _ensure()
    return list(_REGISTRY.values())


def load_desktop_bases(path: Path | str) -> list[DesktopBaseSpec]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    items = raw.get("desktop_bases") or raw
    out: list[DesktopBaseSpec] = []
    if not isinstance(items, Mapping):
        raise TypeError(f"desktop_bases YAML 应为 mapping: {path}")
    for aid, cfg in items.items():
        if not isinstance(cfg, Mapping):
            continue
        s = DesktopBaseSpec(
            id=str(aid),
            title=str(cfg.get("title") or aid),
            origin_above_bottom_m=float(cfg.get("origin_above_bottom_m", 0.0)),
            pedestal_height_m=float(cfg.get("pedestal_height_m", 0.0)),
            base_body=str(cfg.get("base_body") or "base"),
            notes=str(cfg.get("notes") or ""),
        )
        register_desktop_base(s)
        out.append(s)
    return out


def _ensure() -> None:
    global _LOADED
    if _LOADED and _REGISTRY:
        return
    register_desktop_base(
        DesktopBaseSpec(
            id="ur5e",
            title="UR5e（menagerie，原点≈底座底面）",
            origin_above_bottom_m=0.0,
            pedestal_height_m=0.0,
            base_body="base",
            notes="dual_ur：T_contact_urdf≈I；mount_z ≈ table_top",
        )
    )
    register_desktop_base(
        DesktopBaseSpec(
            id="artixon_6a",
            title="Artixon Arm-6A",
            origin_above_bottom_m=0.0,
            pedestal_height_m=0.02,
            base_body="arm_base_link",
            notes="接触系→URDF：沿 +Z 抬 0.02 m（垫块）",
        )
    )
    register_desktop_base(
        DesktopBaseSpec(
            id="franka_panda",
            title="Franka Panda（占位）",
            origin_above_bottom_m=0.0,
            pedestal_height_m=0.0,
            base_body="link0",
            notes="接 menagerie 后按实测改 origin_above_bottom_m",
        )
    )
    here = Path(__file__).resolve()
    for cand in (
        here.parents[4] / "configs" / "plugins" / "frame" / "desktop_bases.yaml",
        Path.cwd() / "configs" / "plugins" / "frame" / "desktop_bases.yaml",
    ):
        if cand.is_file():
            load_desktop_bases(cand)
            break
    _LOADED = True
