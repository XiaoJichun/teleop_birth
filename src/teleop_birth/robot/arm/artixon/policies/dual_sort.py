"""双臂物流主策略（对齐视频）：

1. 左臂从入站传送带拿起 → 放到作业区
2. 左右臂分列两侧夹住 → 抬起 → 盒子原地翻面（双臂不绕圈交叉）
3. 右臂沿桌面把盒子推到出站传送带

传送带与桌面齐平、平行一字型。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from teleop_birth.robot.arm.artixon.collection.grasp_geometry import (
    DEFAULT_PAD_OFFSET_XYZ,
    gripper_close_for_width,
    pinch_pose_for_object,
    top_down_quat_xyzw,
)
from teleop_birth.robot.arm.artixon.collection.motion import DualScriptedMotion
from teleop_birth.robot.arm.artixon.collection.sticky import StickyGrasp


OnStep = Callable[[dict[str, np.ndarray], float, float, str], None]


@dataclass
class PolicyContext:
    props: object
    motion: DualScriptedMotion
    sticky: StickyGrasp
    work_xy: tuple[float, float]
    in_xy: tuple[float, float]
    out_xy: tuple[float, float]
    table_top_z: float
    label_up: bool
    package: str
    package_half_xyz: tuple[float, float, float] = (0.045, 0.025, 0.020)


def _ease(i: int, n: int) -> float:
    s = i / max(n, 1)
    return s * s * (3.0 - 2.0 * s)


def _z_obj(ctx: PolicyContext) -> float:
    return float(ctx.table_top_z + ctx.package_half_xyz[2] + 0.001)


def _g_close(ctx: PolicyContext) -> float:
    w = 2.0 * min(ctx.package_half_xyz[0], ctx.package_half_xyz[1])
    return gripper_close_for_width(w, pad_skin_m=0.022)


def _pinch(xy, z, q=None) -> np.ndarray:
    if q is None:
        q = top_down_quat_xyzw()
    return pinch_pose_for_object(
        np.array([xy[0], xy[1], z], dtype=np.float64),
        quat_xyzw=q,
        grasp_offset_xyz=DEFAULT_PAD_OFFSET_XYZ,
    )


def _force_attach(ctx: PolicyContext, side: str, g: float) -> None:
    if ctx.sticky.attached and ctx.sticky.holding_side == side:
        return
    if ctx.sticky.attached:
        ctx.sticky.force_release()
    ee = ctx.motion.backend.tcp_pose(side)
    ctx.sticky.update(side, ee, g)
    if ctx.sticky.attached:
        return
    ctx.sticky.anti.on_grasp(ctx.package)
    ctx.sticky._held_quat = ctx.props.get_pose(ctx.package)[3:].copy()
    ctx.sticky.holding_side = side
    mid = ctx.sticky.pad_center(side, ee)
    ctx.props.set_pose(ctx.package, pos=mid, quat_xyzw=ctx.sticky._held_quat)


def _park_arms(ctx: PolicyContext, *, on_step: OnStep | None, stage: str) -> None:
    """双臂收到作业区外侧上方，避免互相干扰。"""
    q = top_down_quat_xyzw()
    z = _z_obj(ctx) + 0.16
    work = np.asarray(ctx.work_xy, dtype=np.float64)
    ctx.motion.go_to(
        left=_pinch(work + np.array([-0.02, 0.16]), z, q),
        right=_pinch(work + np.array([-0.02, -0.16]), z, q),
        left_g=0.0,
        right_g=0.0,
        on_step=(lambda e, lg, rg: on_step(e, lg, rg, stage)) if on_step else None,
    )


def _left_fetch_to_work(ctx: PolicyContext, *, on_step: OnStep | None) -> None:
    q = top_down_quat_xyzw()
    g = _g_close(ctx)
    z = _z_obj(ctx)
    src = np.asarray(ctx.in_xy, dtype=np.float64)
    # 以实际包裹位置为准（允许轻微抖动）
    pkg = ctx.props.get_pose(ctx.package)
    if float(np.linalg.norm(pkg[:2] - src)) < 0.12:
        src = pkg[:2].copy()
    work = np.asarray(ctx.work_xy, dtype=np.float64)
    z_app = z + 0.14
    z_grasp = z + 0.01
    z_carry = z + 0.07
    ctx.sticky.anti.begin_approach(ctx.package)

    def cb(ees, lg, rg, stage: str):
        if on_step:
            on_step(ees, lg, rg, stage)
        ctx.sticky.update("left", ees["left"], lg)

    # 右臂先停在外侧，避免挡路
    ctx.motion.go_to(
        right=_pinch(work + np.array([0.0, -0.18]), z_app, q),
        right_g=0.0,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "R_hold_out"),
    )

    ctx.motion.go_to(
        left=_pinch(src, z_app, q), left_g=0.0,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_approach"),
    )
    ctx.motion.go_to(
        left=_pinch(src, z_grasp, q), left_g=0.0, left_g0=0.0,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_descend"),
    )
    ctx.motion.go_to(
        left=_pinch(src, z_grasp, q), left_g=g, left_g0=0.0,
        n_steps=max(24, ctx.motion.steps_per_segment // 2),
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_squeeze"),
    )
    _force_attach(ctx, "left", g)

    ctx.motion.go_to(
        left=_pinch(src, z_carry, q), left_g=g, left_g0=g,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_lift"),
    )
    # 经中间点再进作业区，轨迹更顺
    mid = 0.5 * (src + work)
    ctx.motion.go_to(
        left=_pinch(mid, z_carry, q), left_g=g, left_g0=g,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_transit"),
    )
    ctx.motion.go_to(
        left=_pinch(work, z_carry, q), left_g=g, left_g0=g,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_to_work"),
    )
    # 先放到桌面高度再松爪，避免松开瞬间盒子从空中瞬移落地
    ctx.motion.go_to(
        left=_pinch(work, z_grasp, q), left_g=g, left_g0=g,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "L_place_down"),
    )
    quat_now = ctx.props.get_pose(ctx.package)[3:].copy()
    ctx.props.set_pose(
        ctx.package,
        pos=[float(work[0]), float(work[1]), z],
        quat_xyzw=quat_now,
    )
    ctx.sticky.force_release()
    ctx.sticky.anti.end_approach(ctx.package)
    ctx.motion.go_to(
        left=_pinch(work, z_grasp, q), left_g=0.0, left_g0=g,
        n_steps=max(20, ctx.motion.steps_per_segment // 2),
        on_step=lambda e, lg, rg: (
            ctx.props.set_pose(
                ctx.package,
                pos=[float(work[0]), float(work[1]), z],
                quat_xyzw=quat_now,
            ),
            cb(e, lg, rg, "L_release"),
        )[-1],
    )
    ctx.motion.go_to(
        left=_pinch(work + np.array([0.0, 0.14]), z_app, q), left_g=0.0,
        on_step=lambda e, lg, rg: (
            ctx.props.set_pose(
                ctx.package,
                pos=[float(work[0]), float(work[1]), z],
                quat_xyzw=quat_now,
            ),
            cb(e, lg, rg, "L_ready_flip"),
        )[-1],
    )


def _dual_flip(ctx: PolicyContext, *, on_step: OnStep | None) -> None:
    """左右臂始终在 +Y / -Y 两侧夹住；盒子原地翻面（不绕圈交叉）。

    注意：on_grasp 会关掉包裹碰撞，必须每步用运动学钉住位姿，否则会掉穿桌面。
    """
    q = top_down_quat_xyzw()
    g = _g_close(ctx)
    work = np.asarray(ctx.work_xy, dtype=np.float64)
    z = _z_obj(ctx)
    z_hold = z + 0.012
    z_lift = z + 0.09
    gap = float(ctx.package_half_xyz[1] + 0.045)
    left_xy = work + np.array([0.0, gap])
    right_xy = work + np.array([0.0, -gap])
    q0 = np.asarray(ctx.props.get_pose(ctx.package)[3:], dtype=np.float64)
    q1 = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    held_quat = q0.copy()
    held_z = z
    ctx.sticky.force_release()
    ctx.sticky.anti.begin_approach(ctx.package)

    def pin(pos_xyz, quat) -> None:
        ctx.props.set_pose(
            ctx.package,
            pos=[float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])],
            quat_xyzw=quat,
        )

    def cb(ees, lg, rg, stage: str, *, pin_z: float | None = None, quat=None):
        # 夹持期间始终钉在作业区中心（双指中点），避免单臂 sticky 把盒子拽歪/掉穿
        if pin_z is not None:
            pin([work[0], work[1], pin_z], quat if quat is not None else held_quat)
        if on_step:
            on_step(ees, lg, rg, stage)

    pin([work[0], work[1], z], q0)

    ctx.motion.go_to(
        left=_pinch(left_xy, z_hold + 0.10, q),
        right=_pinch(right_xy, z_hold + 0.10, q),
        left_g=0.0, right_g=0.0,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "flip_approach", pin_z=z, quat=q0),
    )
    ctx.motion.go_to(
        left=_pinch(left_xy, z_hold, q),
        right=_pinch(right_xy, z_hold, q),
        left_g=0.0, right_g=0.0,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "flip_descend", pin_z=z, quat=q0),
    )
    ctx.motion.go_to(
        left=_pinch(left_xy, z_hold, q),
        right=_pinch(right_xy, z_hold, q),
        left_g=g, right_g=g, left_g0=0.0, right_g0=0.0,
        n_steps=max(24, ctx.motion.steps_per_segment // 2),
        on_step=lambda e, lg, rg: cb(e, lg, rg, "flip_squeeze", pin_z=z, quat=q0),
    )
    ctx.sticky.anti.on_grasp(ctx.package)
    held_quat = q0.copy()
    held_z = z_hold

    # 抬起：双臂同步上升，盒子跟高度走（不用单臂 sticky）
    n_lift = max(28, ctx.motion.steps_per_segment)
    for i in range(1, n_lift + 1):
        s = _ease(i, n_lift)
        zz = (1.0 - s) * z_hold + s * z_lift
        held_z = zz
        pin([work[0], work[1], zz], held_quat)
        ees = ctx.motion.backend.step_to_poses(
            left=_pinch(left_xy, zz, q),
            right=_pinch(right_xy, zz, q),
            left_gripper=g,
            right_gripper=g,
        )
        cb(ees, g, g, "flip_lift", pin_z=zz, quat=held_quat)

    # 原地翻面：双臂不换边，盒子姿态插值到面单朝上
    n = max(48, int(ctx.motion.steps_per_segment * 1.2))
    for i in range(1, n + 1):
        s = _ease(i, n)
        quat = q0 if s < 0.5 else q1
        held_quat = quat
        zz = z_lift + 0.025 * np.sin(np.pi * s)
        held_z = zz
        ly = left_xy + np.array([0.015 * np.sin(np.pi * s), 0.0])
        ry = right_xy + np.array([-0.015 * np.sin(np.pi * s), 0.0])
        pin([work[0], work[1], zz], quat)
        ees = ctx.motion.backend.step_to_poses(
            left=_pinch(ly, zz, q),
            right=_pinch(ry, zz, q),
            left_gripper=g,
            right_gripper=g,
        )
        cb(ees, g, g, "flip_rotate", pin_z=zz, quat=quat)

    # 放回桌面后再松爪、开碰撞（顺序不能反）
    n_place = max(28, ctx.motion.steps_per_segment)
    for i in range(1, n_place + 1):
        s = _ease(i, n_place)
        zz = (1.0 - s) * held_z + s * z
        pin([work[0], work[1], zz], q1)
        ees = ctx.motion.backend.step_to_poses(
            left=_pinch(left_xy, max(zz, z_hold), q),
            right=_pinch(right_xy, max(zz, z_hold), q),
            left_gripper=g,
            right_gripper=g,
        )
        cb(ees, g, g, "flip_place", pin_z=zz, quat=q1)

    pin([work[0], work[1], z], q1)
    ctx.sticky.anti.on_release(ctx.package)
    ctx.sticky.force_release()

    # 松爪后退
    ctx.motion.go_to(
        left=_pinch(left_xy, z_hold + 0.08, q),
        right=_pinch(right_xy, z_hold + 0.08, q),
        left_g=0.0, right_g=0.0, left_g0=g, right_g0=g,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "flip_open", pin_z=z, quat=q1),
    )
    ctx.sticky.anti.end_approach(ctx.package)
    pin([work[0], work[1], z], q1)

    ctx.motion.go_to(
        left=_pinch(work + np.array([0.0, 0.18]), z + 0.14, q),
        right=_pinch(work + np.array([0.0, -0.10]), z + 0.10, q),
        left_g=0.0, right_g=0.0,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "post_flip_clear", pin_z=z, quat=q1),
    )


def _right_slide_to_out(ctx: PolicyContext, *, on_step: OnStep | None) -> None:
    q = top_down_quat_xyzw()
    work = np.asarray(ctx.work_xy, dtype=np.float64)
    out = np.asarray(ctx.out_xy, dtype=np.float64)
    z = _z_obj(ctx)
    z_push = float(ctx.table_top_z + max(0.028, ctx.package_half_xyz[2] * 0.75))
    behind = float(ctx.package_half_xyz[1] + 0.06)
    ctx.sticky.anti.begin_approach(ctx.package)

    def cb(ees, lg, rg, stage: str):
        if on_step:
            on_step(ees, lg, rg, stage)

    start = ctx.props.get_pose(ctx.package)[:2].copy()
    if float(np.linalg.norm(start - work)) > 0.10:
        start = work.copy()
        ctx.props.set_pose(
            ctx.package,
            pos=[start[0], start[1], z],
            quat_xyzw=ctx.props.get_pose(ctx.package)[3:],
        )

    # 推板在盒子 +Y 侧，向 -Y 推出站
    prep = start + np.array([0.0, behind])
    ctx.motion.go_to(
        right=_pinch(prep, z_push + 0.08, q), right_g=0.2,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "R_prep"),
    )
    ctx.motion.go_to(
        right=_pinch(prep, z_push, q), right_g=0.2, right_g0=0.2,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "R_contact"),
    )

    n = max(55, int(ctx.motion.steps_per_segment * 1.4))
    quat = ctx.props.get_pose(ctx.package)[3:].copy()
    for i in range(1, n + 1):
        s = _ease(i, n)
        xy = (1.0 - s) * start + s * out
        arm_xy = xy + np.array([0.0, behind * (1.0 - 0.4 * s)])
        ctx.props.set_pose(ctx.package, pos=[float(xy[0]), float(xy[1]), z], quat_xyzw=quat)
        ees = ctx.motion.backend.step_to_poses(
            right=_pinch(arm_xy, z_push, q), right_gripper=0.2,
        )
        cb(ees, 0.0, 0.2, "R_slide_out")

    ctx.motion.go_to(
        right=_pinch(out + np.array([0.0, 0.10]), z_push + 0.10, q),
        right_g=0.0, right_g0=0.2,
        on_step=lambda e, lg, rg: cb(e, lg, rg, "R_retreat"),
    )
    ctx.sticky.anti.end_approach(ctx.package)
    ctx.props.set_pose(ctx.package, pos=[float(out[0]), float(out[1]), z], quat_xyzw=quat)


def run_pick_flip_place(ctx: PolicyContext, *, on_step: OnStep | None = None) -> str:
    _park_arms(ctx, on_step=on_step, stage="home_clear")
    _left_fetch_to_work(ctx, on_step=on_step)
    _dual_flip(ctx, on_step=on_step)
    _right_slide_to_out(ctx, on_step=on_step)
    _park_arms(ctx, on_step=on_step, stage="done_park")
    return "pick_flip_place"


def run_feed_and_push(ctx: PolicyContext, *, on_step: OnStep | None = None) -> str:
    return run_pick_flip_place(ctx, on_step=on_step)


def run_flip_then_push(ctx: PolicyContext, *, on_step: OnStep | None = None) -> str:
    return run_pick_flip_place(ctx, on_step=on_step)
