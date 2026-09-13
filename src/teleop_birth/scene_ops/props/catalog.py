"""物品目录：内置程序化资产 + YAML/MJCF 扩展（便于移植到其它仿真）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

import yaml


@dataclass
class PropSpec:
    """单个可生成物品的规格。

    kind:
      - box / cylinder / plate — 程序化几何
      - mjcf — 从文件加载（库内物品）
    """

    name: str
    kind: str
    rgba: list[float] = field(default_factory=lambda: [0.8, 0.8, 0.8, 1.0])
    size: list[float] = field(default_factory=lambda: [0.025, 0.025, 0.025])
    mass: float = 0.05
    friction: list[float] = field(default_factory=lambda: [1.0, 0.005, 0.0001])
    freejoint: bool = True
    # 初始位姿（世界系）；采集时可由 LayoutRandomizer 覆盖
    pos: list[float] = field(default_factory=lambda: [0.0, 0.4, 0.45])
    quat_wxyz: list[float] = field(default_factory=lambda: [1.0, 0.0, 0.0, 0.0])
    # mjcf 相对 project_root 或绝对路径
    mjcf_path: Optional[str] = None
    # 语义标签（采集指令用）
    labels: dict[str, str] = field(default_factory=dict)
    # plate: 内半径提示（放置成功判定）
    place_radius_m: Optional[float] = None
    # 采集时每步钉住位姿（容器类）
    anchor: bool = False

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PropSpec":
        known = {f.name for f in cls.__dataclass_fields__.values()}  # type: ignore[attr-defined]
        kwargs = {k: v for k, v in data.items() if k in known}
        return cls(**kwargs)

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "kind": self.kind,
            "rgba": list(self.rgba),
            "size": list(self.size),
            "mass": self.mass,
            "friction": list(self.friction),
            "freejoint": self.freejoint,
            "pos": list(self.pos),
            "quat_wxyz": list(self.quat_wxyz),
            "mjcf_path": self.mjcf_path,
            "labels": dict(self.labels),
            "place_radius_m": self.place_radius_m,
            "anchor": self.anchor,
        }


class PropCatalog:
    """物品库：内置红色方块 / 蓝色盘子，并支持从 configs/scene/props 加载。"""

    def __init__(self, library_dir: Path | None = None) -> None:
        self.library_dir = Path(library_dir) if library_dir else None
        self._specs: dict[str, PropSpec] = {}
        self._register_builtins()
        if self.library_dir and self.library_dir.is_dir():
            self.load_directory(self.library_dir)

    def _register_builtins(self) -> None:
        self.register(
            PropSpec(
                name="red_cube",
                kind="box",
                rgba=[0.85, 0.12, 0.12, 1.0],
                size=[0.025, 0.025, 0.025],
                mass=0.04,
                pos=[0.12, 0.38, 0.45],
                labels={"color": "red", "shape": "cube", "role": "grasp"},
            )
        )
        self.register(
            PropSpec(
                name="blue_plate",
                kind="plate",
                rgba=[0.15, 0.35, 0.85, 1.0],
                # cylinder: [radius, half-height]
                size=[0.07, 0.008],
                mass=0.08,
                freejoint=False,
                pos=[-0.10, 0.42, 0.43],
                place_radius_m=0.055,
                anchor=True,
                labels={"color": "blue", "shape": "plate", "role": "container"},
            )
        )

    def register(self, spec: PropSpec) -> None:
        self._specs[spec.name] = spec

    def get(self, name: str) -> PropSpec:
        if name not in self._specs:
            raise KeyError(f"物品不在目录中: {name}；已有 {sorted(self._specs)}")
        return self._specs[name]

    def list_names(self) -> list[str]:
        return sorted(self._specs)

    def resolve_many(self, names: Iterable[str]) -> list[PropSpec]:
        return [self.get(n) for n in names]

    def load_directory(self, directory: Path) -> int:
        """加载目录下 *.yaml 物品定义；同名覆盖内置。"""
        count = 0
        for path in sorted(Path(directory).glob("*.yaml")):
            with path.open(encoding="utf-8") as fp:
                data = yaml.safe_load(fp) or {}
            if not isinstance(data, dict):
                continue
            if "name" not in data:
                data["name"] = path.stem
            self.register(PropSpec.from_dict(data))
            count += 1
        return count

    def default_demo_props(self) -> list[PropSpec]:
        """红色方块 + 蓝色盘子（需求 1 默认场景）。"""
        return self.resolve_many(["red_cube", "blue_plate"])
