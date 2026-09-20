"""握把体轴 → 夹爪体轴矩阵 M（§5.4）。

与 R_a 同构：按「握把语义轴落到夹爪哪根轴」生成 M；
换 home（尖端朝下 / 朝前）只换 profile，不改代码。

约定（OpenXR grip）：+X 右、+Y 上、−Z 瞄准。
夹爪体轴：+Z = 接近轴（尖端方向）。
v_tool = M @ v_grip。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Mapping, Optional, Sequence

import numpy as np
import yaml

from teleop_birth.teleop.frame.axes import parse_axis, validate_ra

__all__ = [
    "GripTcpProfile",
    "build_M",
    "build_M_from_dict",
    "default_grip_to_tcp_M",
    "get_grip_tcp_profile",
    "list_grip_tcp_profiles",
    "load_grip_tcp_profiles",
    "register_grip_tcp_profile",
    "resolve_grip_tcp_M",
    "home_R_tip_down",
    "home_R_tip_forward",
]


def build_M(
    *,
    grip_right_to: str | Sequence[float],
    grip_up_to: str | Sequence[float],
    grip_aim_to: str | Sequence[float],
    atol: float = 1e-6,
) -> np.ndarray:
    """按「握把语义 → 夹爪体轴」生成 M。

    grip_right_to  ← 握把 +X 落到工具哪根轴
    grip_up_to     ← 握把 +Y
    grip_aim_to    ← 握把瞄准 (−Z) 落到工具哪根轴（通常是接近轴 +Z）

    因此：
      M @ e_x = right_to
      M @ e_y = up_to
      M @ (−e_z) = aim_to  ⇒  第 3 列 = −aim_to
    """
    c0 = parse_axis(grip_right_to)
    c1 = parse_axis(grip_up_to)
    aim = parse_axis(grip_aim_to)
    c2 = -aim
    M = np.column_stack([c0, c1, c2])
    validate_ra(M, atol=atol)  # 同正交 + det=+1
    return M


def build_M_from_dict(mapping: Mapping[str, str | Sequence[float]], **kwargs) -> np.ndarray:
    """键：right/up/aim（或 grip_right/grip_up/grip_aim）。"""

    def _get(*keys: str) -> str | Sequence[float]:
        for k in keys:
            if k in mapping:
                return mapping[k]
        raise KeyError(f"缺少握把→夹爪映射键，需要其一: {keys}")

    return build_M(
        grip_right_to=_get("right", "grip_right", "grip_right_to"),
        grip_up_to=_get("up", "grip_up", "grip_up_to"),
        grip_aim_to=_get("aim", "grip_aim", "grip_aim_to", "forward"),
        **kwargs,
    )


def default_grip_to_tcp_M() -> np.ndarray:
    """默认：平握 + 尖端朝下 home（历史写死矩阵）。"""
    return build_M(grip_right_to="+y", grip_up_to="+x", grip_aim_to="+z")


def home_R_tip_down() -> np.ndarray:
    """演示用：接近轴朝世界 −Z（尖端朝下）。"""
    return np.diag([1.0, -1.0, -1.0]).astype(np.float64)


def home_R_tip_forward() -> np.ndarray:
    """演示用：接近轴朝世界 +Y（MuJoCo 桌深 / 尖端朝前）。"""
    # 列 = 工具 X/Y/Z 在房间系：X 仍右，Y 朝上(−Z 房间？)，Z 朝前(+Y)
    # 工具 +X = 房间 +X；工具 +Y = 房间 +Z；工具 +Z = 房间 +Y
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class GripTcpProfile:
    """握把→夹爪体轴剖面（换 home 朝向只换 id）。"""

    id: str
    title: str
    grip_right: str | Sequence[float] = "+y"
    grip_up: str | Sequence[float] = "+x"
    grip_aim: str | Sequence[float] = "+z"
    matrix: Optional[tuple[tuple[float, ...], ...]] = None
    home_kind: str = "tip_down"  # tip_down | tip_forward | custom
    notes: str = ""
    tags: tuple[str, ...] = field(default_factory=tuple)

    def M(self) -> np.ndarray:
        if self.matrix is not None:
            return np.asarray(self.matrix, dtype=np.float64).reshape(3, 3)
        return build_M(
            grip_right_to=self.grip_right,
            grip_up_to=self.grip_up,
            grip_aim_to=self.grip_aim,
        )

    def home_R(self) -> np.ndarray:
        if self.home_kind == "tip_forward":
            return home_R_tip_forward()
        return home_R_tip_down()

    def mapper_overrides(self) -> dict:
        out: dict = {"grip_to_tcp": self.id}
        M = self.M()
        out["grip_to_tcp_matrix"] = M.tolist()
        return out


_REGISTRY: dict[str, GripTcpProfile] = {}


def register_grip_tcp_profile(profile: GripTcpProfile) -> GripTcpProfile:
    _REGISTRY[profile.id] = profile
    return profile


def get_grip_tcp_profile(profile_id: str) -> GripTcpProfile:
    if profile_id not in _REGISTRY:
        raise KeyError(
            f"未知握把→夹爪剖面: {profile_id}（已注册 {list(_REGISTRY)}）"
        )
    return _REGISTRY[profile_id]


def list_grip_tcp_profiles() -> list[GripTcpProfile]:
    return list(_REGISTRY.values())


def load_grip_tcp_profiles(path: Path | str) -> list[GripTcpProfile]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    items = raw.get("profiles") or raw
    out: list[GripTcpProfile] = []
    if isinstance(items, dict):
        iterable = items.items()
    else:
        iterable = ((cfg.get("id"), cfg) for cfg in items)
    for pid, cfg in iterable:
        if not pid or not isinstance(cfg, dict):
            continue
        mat = cfg.get("matrix")
        matrix = None
        if mat is not None:
            matrix = tuple(tuple(float(x) for x in row) for row in mat)
        p = GripTcpProfile(
            id=str(pid),
            title=str(cfg.get("title") or pid),
            grip_right=cfg.get("grip_right", cfg.get("right", "+y")),
            grip_up=cfg.get("grip_up", cfg.get("up", "+x")),
            grip_aim=cfg.get("grip_aim", cfg.get("aim", "+z")),
            matrix=matrix,
            home_kind=str(cfg.get("home_kind") or "tip_down"),
            notes=str(cfg.get("notes") or ""),
            tags=tuple(cfg.get("tags") or ()),
        )
        register_grip_tcp_profile(p)
        out.append(p)
    return out


def resolve_grip_tcp_M(
    name: str | None = None,
    matrix: Sequence[Sequence[float]] | None = None,
    *,
    profile_id: str | None = None,
    mapping: Mapping[str, str | Sequence[float]] | None = None,
) -> np.ndarray:
    """优先级：显式矩阵 > 语义 mapping > profile id / 具名预设 > 默认 tip_down。"""
    if matrix is not None:
        M = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        validate_ra(M)
        return M
    if mapping is not None:
        return build_M_from_dict(mapping)
    pid = profile_id or name
    if pid:
        if pid in ("default", "tip_down", "openxr_grip_tip_down"):
            return default_grip_to_tcp_M()
        if pid in _REGISTRY:
            return get_grip_tcp_profile(pid).M()
        raise KeyError(f"未知 grip_to_tcp: {pid}")
    return default_grip_to_tcp_M()


def _bootstrap() -> None:
    register_grip_tcp_profile(
        GripTcpProfile(
            id="tip_down",
            title="尖端朝下 home（默认桌面抓取）",
            grip_right="+y",
            grip_up="+x",
            grip_aim="+z",
            home_kind="tip_down",
            notes="平握手柄 ↔ 朝下夹爪；与历史 default_grip_to_tcp_M 等价",
            tags=("default",),
        )
    )
    register_grip_tcp_profile(
        GripTcpProfile(
            id="tip_forward",
            title="尖端朝前 home（水平接近）",
            # 瞄准 → 接近轴 +Z；右手系要求「上」映到 −Y（不能 right/up/aim 全取正轴）
            grip_right="+x",
            grip_up="-y",
            grip_aim="+z",
            home_kind="tip_forward",
            notes="握把与夹爪都大致朝前：M=diag(1,-1,-1)；拧腕仍绕接近轴",
            tags=("horizontal",),
        )
    )
    register_grip_tcp_profile(
        GripTcpProfile(
            id="identity",
            title="恒等（消融）",
            matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            home_kind="tip_down",
            notes="不做体轴翻译，只适合调试",
            tags=("ablation",),
        )
    )
    for cand in (
        Path(__file__).resolve().parents[4] / "configs" / "plugins" / "frame" / "grip_to_tcp.yaml",
        Path.cwd() / "configs" / "plugins" / "frame" / "grip_to_tcp.yaml",
    ):
        if cand.is_file():
            try:
                load_grip_tcp_profiles(cand)
            except Exception:
                pass
            break


_bootstrap()
