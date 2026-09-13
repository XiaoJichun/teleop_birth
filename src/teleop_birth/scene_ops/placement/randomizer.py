"""同场景物品布局随机化（对齐 Genie Sim layout 扰动思想）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from teleop_birth.scene_ops.props.catalog import PropSpec
from teleop_birth.scene_ops.placement.pose_controller import PropPoseController


@dataclass
class TableWorkspace:
    """桌面可放置区域（世界系，z 为物体底部中心高度）。"""

    x_min: float = -0.05
    x_max: float = 0.32
    y_min: float = 0.32
    y_max: float = 0.52
    z: float = 0.445
    min_separation_m: float = 0.14
    yaw_random: bool = True
    # 可达性球：相对该点过远的采样丢弃（米）
    reach_center: tuple[float, float] = (0.22, 0.42)
    reach_radius_m: float = 0.28


@dataclass
class LayoutRandomizer:
    workspace: TableWorkspace = field(default_factory=TableWorkspace)
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng())

    @classmethod
    def from_table_cfg(
        cls,
        table: dict,
        base_height: float,
        seed: int | None = None,
        *,
        arm: str = "right",
    ) -> "LayoutRandomizer":
        """根据 scene.table + 操作臂估计可达桌面区域。"""
        size = list(table.get("size") or [0.55, 0.35, 0.02])
        pos_xy = list(table.get("pos_xy") or [0.0, 0.4])
        z_off = float(table.get("z_offset_from_base", -0.02))
        hx, hy, hz = float(size[0]), float(size[1]), float(size[2])
        top_z = float(base_height) + z_off + hz
        # 右臂：桌面右前；左臂：左前；避免贴基座 y 过小
        if str(arm).lower().startswith("l"):
            x_min, x_max = pos_xy[0] - 0.28, pos_xy[0] - 0.02
            reach = (pos_xy[0] - 0.18, pos_xy[1] + 0.02)
        else:
            x_min, x_max = pos_xy[0] + 0.10, pos_xy[0] + 0.32
            reach = (pos_xy[0] + 0.20, pos_xy[1] + 0.02)
        y_min = pos_xy[1] - 0.06
        y_max = pos_xy[1] + 0.10
        # 仍夹在桌面半尺寸内
        x_min = max(x_min, pos_xy[0] - hx + 0.12)
        x_max = min(x_max, pos_xy[0] + hx - 0.12)
        y_min = max(y_min, pos_xy[1] - hy + 0.14)
        y_max = min(y_max, pos_xy[1] + hy - 0.12)
        ws = TableWorkspace(
            x_min=float(x_min),
            x_max=float(x_max),
            y_min=float(y_min),
            y_max=float(y_max),
            z=top_z + 0.03,
            min_separation_m=0.12,
            reach_center=(float(reach[0]), float(reach[1])),
            reach_radius_m=0.22,
        )
        rng = np.random.default_rng(seed)
        return cls(workspace=ws, rng=rng)

    def _in_reach(self, xy: np.ndarray) -> bool:
        c = np.asarray(self.workspace.reach_center, dtype=np.float64)
        return float(np.linalg.norm(xy[:2] - c)) <= float(self.workspace.reach_radius_m)

    def sample_layout(self, props: Sequence[PropSpec], max_tries: int = 120) -> dict[str, dict]:
        """为每个 prop 采样不碰撞、且在操作臂可达区内的 xy。"""
        placed: list[tuple[str, np.ndarray]] = []
        out: dict[str, dict] = {}
        for spec in props:
            pos = None
            quat = None
            for _ in range(max_tries):
                x = float(self.rng.uniform(self.workspace.x_min, self.workspace.x_max))
                y = float(self.rng.uniform(self.workspace.y_min, self.workspace.y_max))
                z = float(self.workspace.z)
                if str(spec.kind).lower() in ("plate", "disc", "disk"):
                    z = self.workspace.z - 0.015 + float(spec.size[1])
                elif str(spec.kind).lower() in ("box", "cube", "block"):
                    z = self.workspace.z - 0.015 + float(spec.size[2])
                cand = np.array([x, y, z], dtype=np.float64)
                if not self._in_reach(cand):
                    continue
                if all(np.linalg.norm(cand[:2] - p[:2]) >= self.workspace.min_separation_m for _, p in placed):
                    pos = cand
                    break
            if pos is None:
                # 回落到可达中心附近
                cx, cy = self.workspace.reach_center
                jitter = self.rng.uniform(-0.04, 0.04, size=2)
                z = self.workspace.z - 0.015 + float(spec.size[2] if len(spec.size) > 2 else spec.size[-1])
                pos = np.array([cx + jitter[0], cy + jitter[1], z], dtype=np.float64)
            if self.workspace.yaw_random:
                yaw = float(self.rng.uniform(-np.pi, np.pi))
                half = 0.5 * yaw
                quat = [0.0, 0.0, float(np.sin(half)), float(np.cos(half))]
            else:
                w, x, y, z = spec.quat_wxyz
                quat = [x, y, z, w]
            placed.append((spec.name, pos))
            out[spec.name] = {"pos": pos.tolist(), "quat_xyzw": quat}
        return out

    def apply(
        self,
        controller: PropPoseController,
        props: Sequence[PropSpec],
        layout: dict[str, dict] | None = None,
    ) -> dict[str, dict]:
        layout = layout or self.sample_layout(props)
        controller.set_many(layout)
        return layout
