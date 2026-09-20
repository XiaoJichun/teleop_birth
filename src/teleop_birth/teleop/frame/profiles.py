"""手柄 → 机械臂房间系：即插即用机型剖面（仿真 / 真机共用）。

换臂只换 profile id；Mapper / TeleopLoop 不动。
仿真与真机共用同一套 R_a 与 controller 修正，只换 Backend / RobotEndpoint。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal, Mapping, Optional, Sequence

import numpy as np
import yaml

from teleop_birth.teleop.frame.axes import build_ra, build_ra_from_dict, validate_ra

Kind = Literal["sim", "real", "mock"]

__all__ = [
    "ArmFrameProfile",
    "FrameMapPort",
    "get_arm_profile",
    "list_arm_profiles",
    "load_arm_profiles",
    "register_arm_profile",
    "resolve_align_matrix",
]


@dataclass(frozen=True)
class ArmFrameProfile:
    """一台（类）机械臂在「房间系」里的轴向约定。"""

    id: str
    title: str
    # OpenXR 右/上/前 → 该臂房间系去向（语义轴或向量）
    xr_right: str | tuple[float, float, float] = "right"
    xr_up: str | tuple[float, float, float] = "up"
    xr_forward: str | tuple[float, float, float] = "forward"
    # 可选：整矩阵覆盖（设了就忽略上面的语义映射）
    xr_align_matrix: Optional[tuple[tuple[float, ...], ...]] = None
    controller_quat_xyzw: Optional[tuple[float, float, float, float]] = None
    # 该剖面可挂到哪些 backend.type（仿真 / 真机接口预留）
    sim_backends: tuple[str, ...] = ()
    real_backends: tuple[str, ...] = ()
    notes: str = ""

    def ra(self) -> np.ndarray:
        if self.xr_align_matrix is not None:
            R = np.asarray(self.xr_align_matrix, dtype=np.float64).reshape(3, 3)
            validate_ra(R)
            return R
        return build_ra(
            xr_right_to=self.xr_right,
            xr_up_to=self.xr_up,
            xr_forward_to=self.xr_forward,
        )

    def as_mapper_kwargs(self) -> dict[str, Any]:
        """直接喂给 DualArmMapper / resolve_xr_align。"""
        out: dict[str, Any] = {
            "xr_align": self.id,
            "xr_align_matrix": self.ra().tolist(),
        }
        if self.controller_quat_xyzw is not None:
            out["controller_quat_xyzw"] = list(self.controller_quat_xyzw)
        return out

    def supports(self, backend_type: str, *, kind: Kind | None = None) -> bool:
        bt = str(backend_type).lower()
        if kind == "sim":
            return bt in {x.lower() for x in self.sim_backends} or not self.sim_backends
        if kind == "real":
            return bt in {x.lower() for x in self.real_backends} or not self.real_backends
        sims = {x.lower() for x in self.sim_backends}
        reals = {x.lower() for x in self.real_backends}
        if not sims and not reals:
            return True
        return bt in sims or bt in reals


@dataclass
class FrameMapPort:
    """仿真↔真机无缝衔接端口：只暴露轴向对齐，不绑 Backend。

    Teleop 侧始终消费 SparseTarget（房间系）；换 sim/real 只换 RobotEndpoint。
    """

    profile: ArmFrameProfile
    kind: Kind = "sim"

    def align_matrix(self) -> np.ndarray:
        return self.profile.ra()

    def controller_quat_xyzw(self) -> Optional[list[float]]:
        q = self.profile.controller_quat_xyzw
        return list(q) if q is not None else None

    def mapper_overrides(self) -> dict[str, Any]:
        return self.profile.as_mapper_kwargs()


_REGISTRY: dict[str, ArmFrameProfile] = {}


def register_arm_profile(profile: ArmFrameProfile) -> ArmFrameProfile:
    _REGISTRY[profile.id] = profile
    return profile


def get_arm_profile(profile_id: str) -> ArmFrameProfile:
    if profile_id not in _REGISTRY:
        # 惰性加载默认 YAML
        _ensure_builtins()
    if profile_id not in _REGISTRY:
        raise KeyError(f"未知臂坐标剖面: {profile_id}（已注册 {[p.id for p in list_arm_profiles()]}）")
    return _REGISTRY[profile_id]


def list_arm_profiles() -> list[ArmFrameProfile]:
    _ensure_builtins()
    return list(_REGISTRY.values())


def load_arm_profiles(path: Path | str) -> list[ArmFrameProfile]:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    items = raw.get("profiles") or raw
    out: list[ArmFrameProfile] = []
    if not isinstance(items, Mapping):
        raise TypeError(f"臂剖面 YAML 应为 mapping: {path}")
    for pid, cfg in items.items():
        if not isinstance(cfg, Mapping):
            continue
        p = ArmFrameProfile(
            id=str(pid),
            title=str(cfg.get("title") or pid),
            xr_right=_axis_field(cfg.get("xr_right", "right")),
            xr_up=_axis_field(cfg.get("xr_up", "up")),
            xr_forward=_axis_field(cfg.get("xr_forward", "forward")),
            xr_align_matrix=_matrix_field(cfg.get("xr_align_matrix")),
            controller_quat_xyzw=_quat_field(cfg.get("controller_quat_xyzw")),
            sim_backends=tuple(str(x) for x in (cfg.get("sim_backends") or [])),
            real_backends=tuple(str(x) for x in (cfg.get("real_backends") or [])),
            notes=str(cfg.get("notes") or ""),
        )
        register_arm_profile(p)
        out.append(p)
    return out


def resolve_align_matrix(
    name: str | None = None,
    matrix: Sequence[Sequence[float]] | None = None,
    *,
    profile_id: str | None = None,
    mapping: Mapping[str, str | Sequence[float]] | None = None,
) -> np.ndarray:
    """统一入口：矩阵 > 语义 mapping > profile > 具名预设。"""
    if matrix is not None:
        R = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
        validate_ra(R)
        return R
    if mapping is not None:
        return build_ra_from_dict(mapping)
    if profile_id:
        return get_arm_profile(profile_id).ra()
    key = (name or "openxr_yup_to_zup").lower().strip()
    if key == "identity":
        return np.eye(3, dtype=np.float64)
    _ensure_builtins()
    if key in _REGISTRY:
        return _REGISTRY[key].ra()
    # 兼容旧预设名
    if key in ("openxr_yup_to_zup", "openxr_yup_to_mujoco_zup"):
        return get_arm_profile("mujoco_table_zup").ra()
    raise ValueError(f"未知 xr_align / 臂剖面: {name}")


def _axis_field(v: Any) -> str | tuple[float, float, float]:
    if isinstance(v, str):
        return v
    if isinstance(v, (list, tuple)) and len(v) == 3:
        return (float(v[0]), float(v[1]), float(v[2]))
    raise TypeError(f"轴向字段应为字符串或长度 3 的列表: {v!r}")


def _matrix_field(v: Any) -> Optional[tuple[tuple[float, ...], ...]]:
    if v is None:
        return None
    M = np.asarray(v, dtype=np.float64).reshape(3, 3)
    return tuple(tuple(float(x) for x in row) for row in M)


def _quat_field(v: Any) -> Optional[tuple[float, float, float, float]]:
    if v is None:
        return None
    q = tuple(float(x) for x in v)
    if len(q) != 4:
        raise ValueError(f"controller_quat_xyzw 需要 4 个数: {v}")
    return q  # type: ignore[return-value]


_BUILTINS_LOADED = False


def _ensure_builtins() -> None:
    global _BUILTINS_LOADED
    if _BUILTINS_LOADED and _REGISTRY:
        return
    # 内置默认（无 YAML 也能跑）
    register_arm_profile(
        ArmFrameProfile(
            id="mujoco_table_zup",
            title="MuJoCo 桌面 Z-up（+Y 朝桌深）",
            xr_right="+x",
            xr_up="+z",
            xr_forward="+y",
            sim_backends=("mujoco_dual_ur", "logistics_x2arm", "mock_dual_arm"),
            real_backends=("ur_realtime", "artixon_realtime"),
            notes="默认 OpenXR→MuJoCo 桌面房间系；仿真/真机共用",
        )
    )
    register_arm_profile(
        ArmFrameProfile(
            id="openxr_yup_to_zup",
            title="兼容别名：openxr_yup_to_zup",
            xr_right="+x",
            xr_up="+z",
            xr_forward="+y",
            notes="与历史 YAML mapper.xr_align 同名",
        )
    )
    register_arm_profile(
        ArmFrameProfile(
            id="identity",
            title="消融：不换轴",
            xr_align_matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            notes="仅调试；前/上会对不齐",
        )
    )
    # 尝试加载仓库 YAML
    here = Path(__file__).resolve()
    for cand in (
        here.parents[4] / "configs" / "plugins" / "frame" / "arm_profiles.yaml",
        Path.cwd() / "configs" / "plugins" / "frame" / "arm_profiles.yaml",
    ):
        if cand.is_file():
            load_arm_profiles(cand)
            break
    _BUILTINS_LOADED = True
