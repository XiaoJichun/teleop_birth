"""把 PropSpec 挂到 MuJoCo MjSpec（程序化几何或 MJCF 附件）。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import mujoco
import numpy as np

from teleop_birth.scene.props.catalog import PropSpec


class PropSpawner:
    """场景物品生成器：在 compile 前写入 worldbody。"""

    def __init__(self, project_root: Path | None = None) -> None:
        self.project_root = Path(project_root) if project_root else Path.cwd()

    def attach_all(self, scene: mujoco.MjSpec, props: Sequence[PropSpec]) -> list[str]:
        names: list[str] = []
        for spec in props:
            self.attach_one(scene, spec)
            names.append(spec.name)
        return names

    def attach_one(self, scene: mujoco.MjSpec, spec: PropSpec) -> str:
        kind = str(spec.kind).lower()
        if kind == "mjcf":
            return self._attach_mjcf(scene, spec)
        return self._attach_primitive(scene, spec)

    def _attach_primitive(self, scene: mujoco.MjSpec, spec: PropSpec) -> str:
        name = _safe_body_name(spec.name)
        body = scene.worldbody.add_body(
            name=name,
            pos=list(spec.pos),
            quat=list(spec.quat_wxyz),
        )
        if spec.freejoint:
            body.add_freejoint(name=f"{name}_free")

        kind = str(spec.kind).lower()
        mass = float(max(spec.mass, 1e-4))
        if kind in ("bag", "pouch", "sack", "mailer", "polybag"):
            # 快递袋：单椭球软包（不再加顶部封口条，避免看起来像多出一根「长木条」）
            size = [float(x) for x in spec.size]
            while len(size) < 3:
                size.append(0.02)
            rx, ry, rz = size[0], size[1], size[2]
            body.add_geom(
                name=f"{name}_pouch",
                type=mujoco.mjtGeom.mjGEOM_ELLIPSOID,
                size=[rx, ry, rz],
                rgba=list(spec.rgba),
                friction=list(spec.friction),
                mass=mass,
                condim=4,
            )
            return name

        geom_type, size = self._geom_from_kind(spec)
        body.add_geom(
            name=f"{name}_geom",
            type=geom_type,
            size=list(size),
            rgba=list(spec.rgba),
            friction=list(spec.friction),
            mass=mass,
            condim=4,
        )
        return name

    def _attach_mjcf(self, scene: mujoco.MjSpec, spec: PropSpec) -> str:
        if not spec.mjcf_path:
            raise ValueError(f"{spec.name}: kind=mjcf 需要 mjcf_path")
        path = Path(spec.mjcf_path)
        if not path.is_file():
            path = self.project_root / spec.mjcf_path
        if not path.is_file():
            raise FileNotFoundError(f"MJCF 不存在: {spec.mjcf_path}")
        child = mujoco.MjSpec.from_file(str(path))
        frame = scene.worldbody.add_frame(pos=list(spec.pos), quat=list(spec.quat_wxyz))
        # prefix 保证名字不撞；用户可用 labels 记住语义名
        scene.attach(child, prefix=f"{spec.name}_", frame=frame)
        return spec.name

    @staticmethod
    def _geom_from_kind(spec: PropSpec) -> tuple[int, list[float]]:
        kind = str(spec.kind).lower()
        size = [float(x) for x in spec.size]
        if kind in ("box", "cube", "block"):
            if len(size) != 3:
                raise ValueError(f"{spec.name}: box size 需要 3 个数 (half-extents)")
            return mujoco.mjtGeom.mjGEOM_BOX, size
        if kind in ("cylinder", "plate", "disc", "disk"):
            if len(size) != 2:
                raise ValueError(f"{spec.name}: cylinder/plate size 需要 [radius, half-height]")
            return mujoco.mjtGeom.mjGEOM_CYLINDER, size
        if kind in ("sphere", "ball"):
            if len(size) < 1:
                raise ValueError(f"{spec.name}: sphere size 需要 [radius]")
            return mujoco.mjtGeom.mjGEOM_SPHERE, [size[0], 0.0, 0.0]
        if kind in ("bag", "pouch", "sack", "mailer", "polybag"):
            # 兼容：单 geom 路径用椭球（复合袋在 _attach_primitive 处理）
            if len(size) != 3:
                raise ValueError(f"{spec.name}: bag size 需要 3 个数 (半轴)")
            return mujoco.mjtGeom.mjGEOM_ELLIPSOID, size
        if kind in ("ellipsoid", "ellipse"):
            if len(size) != 3:
                raise ValueError(f"{spec.name}: ellipsoid size 需要 3 个数")
            return mujoco.mjtGeom.mjGEOM_ELLIPSOID, size
        raise ValueError(f"未知 prop.kind: {spec.kind}")

    @staticmethod
    def summarize(props: Sequence[PropSpec]) -> list[dict[str, Any]]:
        return [p.to_dict() for p in props]


def _safe_body_name(name: str) -> str:
    """MuJoCo body 名需 ASCII；中文语义放在 labels 里。"""
    raw = str(name or "obj")
    safe = "".join(c if c.isascii() and (c.isalnum() or c in "-_") else "_" for c in raw)
    safe = "_".join(p for p in safe.split("_") if p) or "obj"
    return safe[:48]
