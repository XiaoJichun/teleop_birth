"""坐标系插件：臂映射 / R_a 生成 / 桌面 base 统一 / PyQt 可视化入口。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np

from teleop_birth.scene.plugins.base import Plugin, PluginContext, register
from teleop_birth.teleop.frame.axes import build_ra, build_ra_from_dict
from teleop_birth.teleop.frame.desktop_base import (
    DesktopBaseSpec,
    T_contact_urdf,
    T_world_contact,
    T_world_urdf,
    get_desktop_base,
    list_desktop_bases,
    load_desktop_bases,
    mount_origin_z,
    mount_pose7,
    mount_pose_xyz,
    table_top_z_from_scene,
)
from teleop_birth.teleop.frame.grip_tcp import (
    GripTcpProfile,
    build_M,
    build_M_from_dict,
    get_grip_tcp_profile,
    list_grip_tcp_profiles,
    load_grip_tcp_profiles,
    resolve_grip_tcp_M,
)
from teleop_birth.teleop.frame.profiles import (
    ArmFrameProfile,
    FrameMapPort,
    get_arm_profile,
    list_arm_profiles,
    load_arm_profiles,
    resolve_align_matrix,
)

__all__ = [
    "ArmFrameMapPlugin",
    "XrAlignPlugin",
    "WristOrientPlugin",
    "GripToTcpPlugin",
    "DesktopBasePlugin",
    "FrameVizPlugin",
]


@register
class ArmFrameMapPlugin(Plugin):
    """手柄 → 不同机械臂房间系映射（即插即用）。

    仿真换臂 / 真机换臂：只换 profile id；接口 FrameMapPort 对 sim/real 相同。
    """

    id = "arm_frame_map"
    title = "手柄→机械臂坐标映射"

    def validate(self, ctx: PluginContext) -> list[str]:
        errs: list[str] = []
        frame = dict(ctx.scene_cfg.get("frame") or {})
        pid = frame.get("arm_profile") or ctx.extras.get("arm_profile")
        if pid:
            try:
                get_arm_profile(str(pid))
            except KeyError as e:
                errs.append(str(e))
        return errs

    def attach(self, ctx: PluginContext, mount=None) -> None:
        frame = dict(ctx.scene_cfg.get("frame") or {})
        pid = str(frame.get("arm_profile") or "mujoco_table_zup")
        kind = str(frame.get("kind") or ctx.extras.get("backend_kind") or "sim")
        if kind not in ("sim", "real", "mock"):
            kind = "sim"
        port = FrameMapPort(profile=get_arm_profile(pid), kind=kind)  # type: ignore[arg-type]
        ctx.extras["frame_map"] = port
        ctx.extras["xr_align_matrix"] = port.align_matrix().tolist()
        ctx.name_map["arm_profile"] = pid

    def for_arm(self, arm_profile_id: str, *, kind: str = "sim") -> FrameMapPort:
        k = kind if kind in ("sim", "real", "mock") else "sim"
        return FrameMapPort(profile=get_arm_profile(arm_profile_id), kind=k)  # type: ignore[arg-type]

    def list_available(self) -> list[dict[str, Any]]:
        return [
            {
                "id": p.id,
                "title": p.title,
                "sim_backends": list(p.sim_backends),
                "real_backends": list(p.real_backends),
                "notes": p.notes,
            }
            for p in list_arm_profiles()
        ]

    def load_yaml(self, path: Path) -> list[ArmFrameProfile]:
        return load_arm_profiles(path)


@register
class XrAlignPlugin(Plugin):
    """给定映射需求，生成 OpenXR→房间系 R_a。"""

    id = "xr_align"
    title = "OpenXR→房间系 R_a 生成"

    def build(
        self,
        *,
        xr_right_to: str | Sequence[float] = "+x",
        xr_up_to: str | Sequence[float] = "+z",
        xr_forward_to: str | Sequence[float] = "+y",
        mapping: Mapping[str, str | Sequence[float]] | None = None,
        profile_id: str | None = None,
        matrix: Sequence[Sequence[float]] | None = None,
    ):
        if matrix is not None or profile_id or mapping is not None:
            return resolve_align_matrix(matrix=matrix, profile_id=profile_id, mapping=mapping)
        return build_ra(
            xr_right_to=xr_right_to,
            xr_up_to=xr_up_to,
            xr_forward_to=xr_forward_to,
        )

    def from_dict(self, mapping: Mapping[str, str | Sequence[float]]):
        return build_ra_from_dict(mapping)

    def attach(self, ctx: PluginContext, mount=None) -> None:
        frame = dict(ctx.scene_cfg.get("frame") or {})
        if "xr_mapping" in frame:
            R = build_ra_from_dict(dict(frame["xr_mapping"]))
            ctx.extras["xr_align_matrix"] = R.tolist()
            ctx.name_map["xr_align_source"] = "xr_mapping"
        elif frame.get("arm_profile"):
            R = get_arm_profile(str(frame["arm_profile"])).ra()
            ctx.extras["xr_align_matrix"] = R.tolist()
            ctx.name_map["xr_align_source"] = str(frame["arm_profile"])


@register
class DesktopBasePlugin(Plugin):
    """统一桌面臂 base：各 URDF 原点经 SE(3) 落到桌面接触点，便于换臂。

    接触系与房间系轴一致（+X 右、+Y 前、+Z 上）；不同臂只差 T_contact_urdf。
    """

    id = "desktop_base"
    title = "桌面机械臂 base 对齐"

    def validate(self, ctx: PluginContext) -> list[str]:
        errs: list[str] = []
        db = dict(ctx.scene_cfg.get("desktop_base") or {})
        arm_id = db.get("arm_id") or db.get("id")
        if arm_id:
            try:
                get_desktop_base(str(arm_id))
            except KeyError as e:
                errs.append(str(e))
        return errs

    def attach(self, ctx: PluginContext, mount=None) -> None:
        db = dict(ctx.scene_cfg.get("desktop_base") or {})
        if not db and not ctx.scene_cfg.get("table_z") and "table" not in ctx.scene_cfg:
            return
        arm_id = str(db.get("arm_id") or db.get("id") or "ur5e")
        spec = get_desktop_base(arm_id)
        # 允许场景覆盖数值
        if any(k in db for k in ("origin_above_bottom_m", "pedestal_height_m")):
            spec = DesktopBaseSpec(
                id=spec.id,
                title=spec.title,
                origin_above_bottom_m=float(db.get("origin_above_bottom_m", spec.origin_above_bottom_m)),
                pedestal_height_m=float(db.get("pedestal_height_m", spec.pedestal_height_m)),
                base_body=str(db.get("base_body") or spec.base_body),
                notes=spec.notes,
            )
        try:
            top = table_top_z_from_scene(ctx.scene_cfg)
        except Exception:
            top = float(db.get("table_top_z", 0.44))
        xy = list(db.get("xy") or db.get("pos_xy") or [0.0, 0.0])
        if len(xy) < 2:
            xy = [0.0, 0.0]
        yaw = float(db.get("yaw_rad", db.get("yaw", 0.0)))
        z = mount_origin_z(top, spec)
        Twc = T_world_contact(top, xy, yaw_rad=yaw)
        Tcu = T_contact_urdf(spec)
        Twu = Twc @ Tcu
        ctx.extras["desktop_base"] = spec
        ctx.extras["table_top_z"] = top
        ctx.extras["mount_origin_z"] = z
        ctx.extras["T_world_contact"] = Twc
        ctx.extras["T_contact_urdf"] = Tcu
        ctx.extras["T_world_urdf"] = Twu
        ctx.extras["T_world_base"] = Twu  # 别名：换臂后统一的「base」即 URDF 原点挂载
        ctx.name_map["desktop_base_arm"] = arm_id
        ctx.name_map["mount_origin_z"] = z
        ctx.name_map["table_top_z"] = top
        ctx.name_map["T_world_base"] = Twu.tolist()
        ctx.name_map["T_contact_urdf"] = Tcu.tolist()

    def mount_xyz(self, table_top_z: float, xy: Sequence[float], arm_id: str) -> list[float]:
        return mount_pose_xyz(table_top_z, xy, arm_id)

    def mount_T(
        self,
        table_top_z: float,
        xy: Sequence[float],
        arm_id: str,
        *,
        yaw_rad: float = 0.0,
    ) -> np.ndarray:
        """返回 T_world_urdf（4×4）。"""
        return T_world_urdf(table_top_z, xy, arm_id, yaw_rad=yaw_rad)

    def mount_pose(
        self,
        table_top_z: float,
        xy: Sequence[float],
        arm_id: str,
        *,
        yaw_rad: float = 0.0,
    ) -> np.ndarray:
        return mount_pose7(table_top_z, xy, arm_id, yaw_rad=yaw_rad)

    def list_available(self) -> list[dict[str, Any]]:
        return [
            {
                "id": s.id,
                "title": s.title,
                "origin_above_bottom_m": s.origin_above_bottom_m,
                "pedestal_height_m": s.pedestal_height_m,
                "base_body": s.base_body,
                "lift_m": s.lift_m(),
                "notes": s.notes,
            }
            for s in list_desktop_bases()
        ]

    def load_yaml(self, path: Path) -> list[DesktopBaseSpec]:
        return load_desktop_bases(path)


@register
class WristOrientPlugin(Plugin):
    """第 2 步姿态：房间 yaw + 夹爪体轴拧腕（与 RelativeClutch / 文档同一套）。"""

    id = "wrist_orient"
    title = "腕部姿态映射（yaw + tool）"

    def validate(self, ctx: PluginContext) -> list[str]:
        return []

    def attach(self, ctx: PluginContext, mount=None) -> None:
        from teleop_birth.teleop.frame.orientation import WristOrientationMap

        frame = dict(ctx.scene_cfg.get("frame") or {})
        R_a = None
        if ctx.extras.get("xr_align_matrix") is not None:
            R_a = np.asarray(ctx.extras["xr_align_matrix"], dtype=float).reshape(3, 3)
        elif frame.get("arm_profile"):
            R_a = get_arm_profile(str(frame["arm_profile"])).ra()
        M = ctx.extras.get("grip_to_tcp_M")
        if M is None:
            M = resolve_grip_tcp_M(
                frame.get("grip_to_tcp"),
                frame.get("grip_to_tcp_matrix"),
                profile_id=frame.get("grip_to_tcp"),
                mapping=frame.get("grip_to_tcp_mapping"),
            )
        mapper = WristOrientationMap(
            R_a=R_a,
            controller_quat_xyzw=frame.get("controller_quat_xyzw"),
            M=M,
            position_only=bool(frame.get("position_only", False)),
        )
        ctx.extras["wrist_orient"] = mapper
        ctx.name_map["wrist_orient"] = self.id

    def build(
        self,
        *,
        R_a=None,
        controller_quat_xyzw=None,
        M=None,
        position_only: bool = False,
    ):
        from teleop_birth.teleop.frame.orientation import WristOrientationMap

        return WristOrientationMap(
            R_a=R_a,
            controller_quat_xyzw=controller_quat_xyzw,
            M=M,
            position_only=position_only,
        )

    def map_demo(
        self,
        *,
        yaw_deg: float = 0.0,
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        R_a=None,
        M=None,
        grip_to_tcp: str | None = None,
        R_r0=None,
    ):
        """用「绕手柄轴转多少度」演示第 2 步（便于 UI）。

        pitch_deg — 绕手柄 +Y（上）→ 进入房间竖直转 ψ
        roll_deg  — 绕手柄 +X → 经 M 侧倾
        yaw_deg   — 绕手柄 +Z → 经 M 拧腕（瞄准相关）
        """
        from teleop_birth.teleop.frame.orientation import WristOrientationMap, rot_from_aa

        if M is None and grip_to_tcp:
            M = resolve_grip_tcp_M(profile_id=grip_to_tcp)
        mapper = self.build(R_a=R_a, M=M)
        w_demo = np.array(
            [np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg)],
            dtype=np.float64,
        )
        R_xr0 = np.eye(3)
        R_xr1 = rot_from_aa(w_demo)
        if R_r0 is None:
            if grip_to_tcp:
                try:
                    R_r0 = get_grip_tcp_profile(grip_to_tcp).home_R()
                except KeyError:
                    R_r0 = np.diag([1.0, -1.0, -1.0])
            else:
                R_r0 = np.diag([1.0, -1.0, -1.0])  # tip_down home
        return mapper.map(R_xr0=R_xr0, R_xr1=R_xr1, R_r0=R_r0)


@register
class GripToTcpPlugin(Plugin):
    """握把体轴 → 夹爪体轴 M（尖端朝下 / 朝前等）。与 frame_viz 共用界面。"""

    id = "grip_to_tcp"
    title = "握把→夹爪体轴 M"

    def validate(self, ctx: PluginContext) -> list[str]:
        errs: list[str] = []
        frame = dict(ctx.scene_cfg.get("frame") or {})
        pid = frame.get("grip_to_tcp")
        if pid:
            try:
                get_grip_tcp_profile(str(pid))
            except KeyError as e:
                errs.append(str(e))
        return errs

    def attach(self, ctx: PluginContext, mount=None) -> None:
        frame = dict(ctx.scene_cfg.get("frame") or {})
        pid = frame.get("grip_to_tcp") or "tip_down"
        M = resolve_grip_tcp_M(
            str(pid),
            frame.get("grip_to_tcp_matrix"),
            profile_id=str(pid),
            mapping=frame.get("grip_to_tcp_mapping"),
        )
        ctx.extras["grip_to_tcp_M"] = M
        ctx.name_map["grip_to_tcp"] = str(pid)

    def build(
        self,
        *,
        grip_right_to: str | Sequence[float] = "+y",
        grip_up_to: str | Sequence[float] = "+x",
        grip_aim_to: str | Sequence[float] = "+z",
        mapping: Mapping[str, str | Sequence[float]] | None = None,
        profile_id: str | None = None,
        matrix: Sequence[Sequence[float]] | None = None,
    ):
        if matrix is not None or profile_id or mapping is not None:
            return resolve_grip_tcp_M(matrix=matrix, profile_id=profile_id, mapping=mapping)
        return build_M(
            grip_right_to=grip_right_to,
            grip_up_to=grip_up_to,
            grip_aim_to=grip_aim_to,
        )

    def from_dict(self, mapping: Mapping[str, str | Sequence[float]]):
        return build_M_from_dict(mapping)

    def list_available(self) -> list[dict[str, Any]]:
        return [
            {
                "id": p.id,
                "title": p.title,
                "home_kind": p.home_kind,
                "notes": p.notes,
            }
            for p in list_grip_tcp_profiles()
        ]

    def load_yaml(self, path: Path) -> list[GripTcpProfile]:
        return load_grip_tcp_profiles(path)


@register
class FrameVizPlugin(Plugin):
    """共用坐标系转换界面：① R_a  ② 腕部姿态 R_t  ③ 握把→夹爪 M。"""

    id = "frame_viz"
    title = "坐标系转换可视化 (PyQt / 终端)"

    def validate(self, ctx: PluginContext) -> list[str]:
        return []

    def attach(self, ctx: PluginContext, mount=None) -> None:
        ctx.extras["frame_viz"] = self.id

    def launch(
        self,
        *,
        profile_id: Optional[str] = None,
        mapping: Optional[Mapping[str, str]] = None,
        grip_to_tcp: Optional[str] = None,
        tab: str = "axes",
    ) -> int:
        from teleop_birth.teleop.frame.viz import run_frame_viz

        return run_frame_viz(
            profile_id=profile_id,
            mapping=mapping,
            grip_to_tcp=grip_to_tcp,
            tab=tab,
        )
