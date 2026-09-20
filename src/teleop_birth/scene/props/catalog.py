"""物品目录：内置程序化资产 + YAML/MJCF 扩展（便于移植到其它仿真）。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional

import yaml

_CONFIG_REL = "configs/scene/model_library.yaml"
_DEFAULT_ROOT_REL = "configs/scene/props"


@dataclass
class PropSpec:
    """单个可生成物品的规格。

    kind:
      - box / cylinder / plate / sphere / bag — 程序化几何
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


def resolve_library_dirs(project_root: Path) -> list[Path]:
    """按 model_library.yaml 解析要扫描的目录（override > default + extra）。"""
    root = Path(project_root)
    cfg_path = root / _CONFIG_REL
    data: dict[str, Any] = {}
    if cfg_path.is_file():
        loaded = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        if isinstance(loaded, dict):
            data = loaded

    dirs: list[Path] = []
    override = str(data.get("override_root") or "").strip()
    default_rel = str(data.get("default_root") or _DEFAULT_ROOT_REL).strip()
    primary = Path(override) if override else Path(default_rel)
    if not primary.is_absolute():
        primary = root / primary
    dirs.append(primary)

    for extra in data.get("extra_roots") or []:
        p = Path(str(extra))
        if not p.is_absolute():
            p = root / p
        if p not in dirs:
            dirs.append(p)
    return dirs


def save_library_override(project_root: Path, override_root: str | Path | None) -> Path:
    """写入 model_library.yaml 的 override_root；传 None/空串清除覆盖。"""
    root = Path(project_root)
    cfg_path = root / _CONFIG_REL
    data: dict[str, Any] = {}
    if cfg_path.is_file():
        loaded = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) or {}
        if isinstance(loaded, dict):
            data = loaded
    data.setdefault("default_root", _DEFAULT_ROOT_REL)
    data.setdefault("extra_roots", [])
    if override_root is None or str(override_root).strip() == "":
        data["override_root"] = ""
    else:
        p = Path(override_root)
        try:
            data["override_root"] = str(p.resolve().relative_to(root.resolve()))
        except ValueError:
            data["override_root"] = str(p.resolve())
    cfg_path.parent.mkdir(parents=True, exist_ok=True)
    cfg_path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return cfg_path


class PropCatalog:
    """物品库：内置红色方块 / 蓝色盘子，并支持从模型库目录加载。"""

    def __init__(self, library_dir: Path | None = None) -> None:
        self.library_dir = Path(library_dir) if library_dir else None
        self.library_dirs: list[Path] = [self.library_dir] if self.library_dir else []
        self._specs: dict[str, PropSpec] = {}
        self._register_builtins()
        for d in self.library_dirs:
            if d and d.is_dir():
                self.load_directory(d)

    @classmethod
    def from_project(cls, project_root: Path) -> "PropCatalog":
        """读取 configs/scene/model_library.yaml，加载默认/覆盖/额外目录。"""
        dirs = resolve_library_dirs(project_root)
        cat = cls(library_dir=None)
        cat.library_dirs = dirs
        cat.library_dir = dirs[0] if dirs else None
        for d in dirs:
            if d.is_dir():
                cat.load_directory(d)
        return cat

    def _register_builtins(self) -> None:
        self.register(
            PropSpec(
                name="red_cube",
                kind="box",
                rgba=[0.85, 0.12, 0.12, 1.0],
                size=[0.025, 0.025, 0.025],
                mass=0.028,
                friction=[3.5, 0.15, 0.01],
                pos=[0.12, 0.38, 0.45],
                labels={
                    "zh": "红色方块",
                    "color": "red",
                    "shape": "cube",
                    "role": "grasp",
                },
            )
        )
        self.register(
            PropSpec(
                name="blue_plate",
                kind="plate",
                rgba=[0.15, 0.35, 0.85, 1.0],
                size=[0.07, 0.008],
                mass=0.08,
                freejoint=False,
                pos=[-0.10, 0.42, 0.43],
                place_radius_m=0.055,
                anchor=True,
                labels={
                    "zh": "蓝色盘子",
                    "color": "blue",
                    "shape": "plate",
                    "role": "container",
                },
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

    def list_entries(self, *, skip_prefixes: tuple[str, ...] = ("sem_",)) -> list[tuple[str, str]]:
        """返回 (id, 显示名)；跳过语义临时件与模板。"""
        out: list[tuple[str, str]] = []
        for name in self.list_names():
            if name.startswith("_") or name == "_template":
                continue
            if any(name.startswith(p) for p in skip_prefixes):
                continue
            spec = self._specs[name]
            labels = spec.labels or {}
            zh = str(labels.get("zh") or "").strip()
            if not zh:
                builtin_zh = {
                    "red_cube": "红色方块",
                    "blue_plate": "蓝色盘子",
                    "green_cube": "绿色方块",
                    "yellow_cylinder": "黄色圆柱",
                    "gray_box": "灰色盒子",
                    "white_sphere": "白色小球",
                }
                zh = builtin_zh.get(name, "")
            if not zh:
                color = str(labels.get("color") or "")
                shape = str(labels.get("shape") or "")
                role = str(labels.get("role") or "")
                parts = [p for p in (color, shape, role) if p]
                zh = " ".join(parts) if parts else name
            display = f"{zh}  [{name}]" if zh != name else name
            out.append((name, display))
        return out

    def resolve_many(self, names: Iterable[str]) -> list[PropSpec]:
        return [self.get(n) for n in names]

    def load_directory(self, directory: Path) -> int:
        """加载目录下 *.yaml 物品定义（含 user/ 子目录）；同名覆盖内置。跳过 _ 开头模板。"""
        count = 0
        root = Path(directory)
        if not root.is_dir():
            return 0
        paths = sorted(root.glob("*.yaml")) + sorted(root.glob("user/*.yaml"))
        for path in paths:
            if path.name.startswith("_"):
                continue
            with path.open(encoding="utf-8") as fp:
                data = yaml.safe_load(fp) or {}
            if not isinstance(data, dict):
                continue
            if "name" not in data:
                data["name"] = path.stem
            name = str(data.get("name") or "")
            if name.startswith("_"):
                continue
            self.register(PropSpec.from_dict(data))
            count += 1
        return count

    def default_demo_props(self) -> list[PropSpec]:
        """红色方块 + 蓝色盘子（需求 1 默认场景）。"""
        return self.resolve_many(["red_cube", "blue_plate"])
