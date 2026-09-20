"""双 Arm-6A 物流工位：对齐参考视频布局（入站区 → 作业桌 → 出站传送带）。

桌上仅保留 2 个快递盒；指爪厚网格隐藏，改用外侧薄垫，消除视觉穿模。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import mujoco

from teleop_birth.robot.arm.artixon.paths import arm_urdf_path

ARM_JOINTS = [f"arm_joint{i}" for i in range(1, 7)]
GRIP_LEFT = "arm_gripper_left_joint"
GRIP_RIGHT = "arm_gripper_right_joint"
TCP_SITE = "tcp_pinch"


def _add_position_actuator(spec, *, name, joint, kp, kv, ctrlrange, forcerange) -> None:
    a = spec.add_actuator(name=name, target=joint)
    a.trntype = mujoco.mjtTrn.mjTRN_JOINT
    a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
    a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
    a.gainprm[0] = kp
    a.biasprm[1] = -kp
    a.biasprm[2] = -kv
    a.ctrlrange[:] = ctrlrange
    a.forcerange[:] = forcerange


def _prepare_arm_spec(urdf: Path, *, tcp_pos: list[float]) -> mujoco.MjSpec:
    """Arm spec：弱化厚指网、加薄指垫，TCP 放在指缝中心。"""
    arm = mujoco.MjSpec.from_file(str(urdf.resolve()))
    for jn in ARM_JOINTS:
        j = arm.joint(jn)
        lo, hi = float(j.range[0]), float(j.range[1])
        if hi <= lo:
            lo, hi = -3.14, 3.14
        _add_position_actuator(
            arm,
            name=f"act_{jn}",
            joint=jn,
            kp=120.0,
            kv=12.0,
            ctrlrange=[lo, hi],
            forcerange=[-150.0, 150.0],
        )
    for jn, cr in (
        (GRIP_LEFT, [0.0, 0.04875]),
        (GRIP_RIGHT, [-0.04875, 0.0]),
    ):
        _add_position_actuator(
            arm,
            name=f"act_{jn}",
            joint=jn,
            kp=800.0,
            kv=40.0,
            ctrlrange=cr,
            forcerange=[-200.0, 200.0],
        )

    bodies = {b.name: b for b in arm.bodies}
    # 厚 mesh 完全隐藏并关碰撞（体积大会视觉穿模）；只用薄垫表示夹持
    for b in arm.bodies:
        bname = b.name or ""
        if "gripper_left_link" not in bname and "gripper_right_link" not in bname:
            continue
        for g in b.geoms:
            g.contype = 0
            g.conaffinity = 0
            rgba = [float(x) for x in g.rgba]
            rgba[3] = 0.0  # 不可见
            g.rgba[:] = rgba

    left = bodies.get("arm_gripper_left_link")
    right = bodies.get("arm_gripper_right_link")
    if left is not None:
        left.add_geom(
            name="finger_pad_l",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.014, 0.0035, 0.016],
            pos=[0.028, -0.014, 0.0],
            rgba=[0.1, 0.1, 0.1, 1.0],
            contype=0,
            conaffinity=0,
        )
        # DIGIT 触觉传感器壳体示意（内侧）
        left.add_geom(
            name="digit_shell_l",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.010, 0.006, 0.014],
            pos=[0.028, -0.022, 0.0],
            rgba=[0.15, 0.55, 0.85, 0.85],
            contype=0,
            conaffinity=0,
        )
    if right is not None:
        right.add_geom(
            name="finger_pad_r",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.014, 0.0035, 0.016],
            pos=[0.028, 0.014, 0.0],
            rgba=[0.1, 0.1, 0.1, 1.0],
            contype=0,
            conaffinity=0,
        )
        right.add_geom(
            name="digit_shell_r",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[0.010, 0.006, 0.014],
            pos=[0.028, 0.022, 0.0],
            rgba=[0.15, 0.55, 0.85, 0.85],
            contype=0,
            conaffinity=0,
        )

    parent = bodies.get("arm_gripper_base_link") or bodies["arm_link6"]
    parent.add_site(name=TCP_SITE, pos=list(tcp_pos), size=[0.006, 0.006, 0.006])
    return arm


def _add_conveyor(wb, *, name: str, pos: list[float], size: list[float], rgba: list[float]) -> None:
    """传送带 + 两侧护栏（护栏贴在长边外侧）。"""
    body = wb.add_body(name=name, pos=pos)
    body.add_geom(
        name=f"{name}_belt",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=size,
        rgba=rgba,
        friction=[1.5, 0.01, 0.001],
        contype=1,
        conaffinity=3,
    )
    sx, sy, sz = size
    # 长边沿 X → 护栏在 ±Y；长边沿 Y → 护栏在 ±X（带子翻转 90° 后）
    if sx >= sy:
        for suffix, ypos in (("rail_a", sy + 0.01), ("rail_b", -(sy + 0.01))):
            body.add_geom(
                name=f"{name}_{suffix}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[sx * 0.85, 0.008, 0.02],
                pos=[0, ypos, 0.01],
                rgba=[0.55, 0.08, 0.08, 1.0],
                contype=0,
                conaffinity=0,
            )
    else:
        for suffix, xpos in (("rail_a", sx + 0.01), ("rail_b", -(sx + 0.01))):
            body.add_geom(
                name=f"{name}_{suffix}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[0.008, sy * 0.85, 0.02],
                pos=[xpos, 0, 0.01],
                rgba=[0.55, 0.08, 0.08, 1.0],
                contype=0,  # 护栏仅视觉，避免挡推送
                conaffinity=0,
            )


def build_logistics_scene(scene_cfg: dict[str, Any] | None = None) -> tuple[mujoco.MjModel, dict]:
    cfg = dict(scene_cfg or {})
    urdf = Path(cfg.get("urdf") or arm_urdf_path())
    if not urdf.is_file():
        raise FileNotFoundError(urdf)

    tcp_pos = [float(x) for x in (cfg.get("tcp_pos") or [0.08, 0.0, 0.0])]
    table_z = float(cfg.get("table_z", 0.42))
    table_size = [float(x) for x in (cfg.get("table_size") or [0.55, 0.42, 0.02])]
    table_top = table_z + table_size[2]

    scene = mujoco.MjSpec()
    scene.modelname = str(cfg.get("model_name") or "logistics_dual_artixon_6a")
    scene.option.timestep = float(cfg.get("timestep", 0.002))
    scene.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    wb = scene.worldbody
    wb.add_light(pos=[0.2, 0.0, 2.4], dir=[0, 0, -1], diffuse=[0.9, 0.9, 0.9])
    wb.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[3, 3, 0.1],
        rgba=[0.85, 0.85, 0.86, 1],
        contype=1,
        conaffinity=3,
    )

    # 木质作业台（参考视频）
    table = wb.add_body(name="work_table", pos=[0.42, 0.0, table_z])
    table.add_geom(
        name="table_top",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=table_size,
        rgba=[0.82, 0.70, 0.52, 1.0],
        contype=1,
        conaffinity=3,
    )

    in_c = dict(cfg.get("in_conveyor") or {})
    out_c = dict(cfg.get("out_conveyor") or {})
    belt_hz = 0.012
    # 默认与桌面齐平：center_z = table_top - half_h
    flush_z = table_top - belt_hz
    in_pos = [float(x) for x in (in_c.get("pos") or [0.38, 0.38, flush_z])]
    in_size = [float(x) for x in (in_c.get("size") or [0.10, 0.18, belt_hz])]
    out_pos = [float(x) for x in (out_c.get("pos") or [0.38, -0.38, flush_z])]
    out_size = [float(x) for x in (out_c.get("size") or [0.10, 0.18, belt_hz])]
    # 强制带顶 = 桌面顶（若 yaml 写错高度则纠正）
    in_pos[2] = table_top - float(in_size[2])
    out_pos[2] = table_top - float(out_size[2])

    # 工坊空场景：不放传送带 / 预置包裹，只留桌+臂，由用户自行添加物品
    include_conveyors = bool(cfg.get("include_conveyors", True))
    if include_conveyors:
        _add_conveyor(wb, name="in_conveyor", pos=in_pos, size=in_size, rgba=[0.12, 0.12, 0.14, 1])
        _add_conveyor(wb, name="out_conveyor", pos=out_pos, size=out_size, rgba=[0.12, 0.12, 0.14, 1])

    # packages 键存在时尊重列表（含空列表）；键缺失才用演示默认包裹
    if "packages" in cfg:
        packages = list(cfg.get("packages") or [])
    else:
        packages = [
            {
                "name": "pkg_0",
                "size": [0.05, 0.028, 0.022],
                "rgba": [0.92, 0.72, 0.35, 1.0],
                "label_rgba": [0.95, 0.95, 0.95, 1.0],
                "pos": [in_pos[0], in_pos[1], in_pos[2] + in_size[2] + 0.022],
                "label_up": False,
                "mass": 0.1,
            },
            {
                "name": "pkg_1",
                "size": [0.05, 0.028, 0.022],
                "rgba": [0.86, 0.58, 0.28, 1.0],
                "label_rgba": [0.95, 0.95, 0.95, 1.0],
                "pos": [0.55, -0.18, table_top + 0.022],
                "label_up": True,
                "mass": 0.1,
            },
        ]

    prop_names: list[str] = []
    label_up: dict[str, bool] = {}
    for p in packages:
        name = str(p["name"])
        prop_names.append(name)
        label_up[name] = bool(p.get("label_up", False))
        size = [float(x) for x in p["size"]]
        pos = [float(x) for x in p["pos"]]
        rgba = [float(x) for x in (p.get("rgba") or [0.8, 0.5, 0.2, 1])]
        label_rgba = [float(x) for x in (p.get("label_rgba") or [0.95, 0.95, 0.95, 1])]
        mass = float(p.get("mass", 0.1))
        body = wb.add_body(name=name, pos=pos)
        body.add_freejoint(name=f"{name}_free")
        body.add_geom(
            name=f"{name}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=size,
            rgba=rgba,
            mass=mass,
            friction=[1.5, 0.005, 0.0001],
            margin=0.001,
            contype=2,
            conaffinity=1,
            solref=[0.02, 1],
            solimp=[0.9, 0.95, 0.001, 0.5, 2],
        )
        body.add_geom(
            name=f"{name}_label",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[size[0] * 0.7, size[1] * 0.5, 0.001],
            pos=[0, 0, size[2] + 0.001],
            rgba=label_rgba,
            contype=0,
            conaffinity=0,
        )

    left_xy = [float(x) for x in (cfg.get("left_arm_xy") or [0.05, 0.16])]
    right_xy = [float(x) for x in (cfg.get("right_arm_xy") or [0.05, -0.16])]
    home_l = [float(x) for x in (cfg.get("home_q_left") or [-0.21, 1.76, -1.15, 0.97, 0.0, -0.21])]
    home_r = [float(x) for x in (cfg.get("home_q_right") or [0.20, 1.76, -1.15, 0.97, 0.0, 0.20])]

    # 共享底座：双臂并排在操作员侧（世界 -X），共同面向 +X 作业桌
    base = wb.add_body(name="dual_base", pos=[0.05, 0.0, table_z])
    base.add_geom(
        name="dual_base_plate",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[0.12, 0.26, 0.025],
        pos=[0, 0, 0.025],
        rgba=[0.12, 0.12, 0.12, 1],
    )

    for prefix, xy in (("L_", left_xy), ("R_", right_xy)):
        # 桌面 base 插件：URDF 原点落在桌面（+ pedestal），换臂时桌面高度统一
        from teleop_birth.teleop.frame.desktop_base import get_desktop_base, mount_origin_z

        db = dict(cfg.get("desktop_base") or {})
        arm_id = str(db.get("arm_id") or "artixon_6a")
        try:
            spec = get_desktop_base(arm_id)
            if "pedestal_height_m" in db or "origin_above_bottom_m" in db:
                from teleop_birth.teleop.frame.desktop_base import DesktopBaseSpec

                spec = DesktopBaseSpec(
                    id=spec.id,
                    title=spec.title,
                    origin_above_bottom_m=float(
                        db.get("origin_above_bottom_m", spec.origin_above_bottom_m)
                    ),
                    pedestal_height_m=float(db.get("pedestal_height_m", spec.pedestal_height_m)),
                    base_body=spec.base_body,
                    notes=spec.notes,
                )
            mount_z = mount_origin_z(table_top, spec)
            ped_h = float(spec.pedestal_height_m)
        except KeyError:
            mount_z = table_z + 0.04
            ped_h = 0.04

        ped = wb.add_body(name=f"{prefix}pedestal", pos=[xy[0], xy[1], table_z])
        ped.add_geom(
            name=f"{prefix}pedestal_cyl",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            size=[0.045, max(0.01, ped_h * 0.5)],
            pos=[0, 0, max(0.01, ped_h * 0.5)],
            rgba=[0.95, 0.95, 0.95, 1],
        )
        # 基座不旋转：Arm-6A 零位朝 +X，正对操作面
        frame = ped.add_frame(name=f"{prefix}mount", pos=[0, 0, mount_z - table_z])
        arm = _prepare_arm_spec(urdf, tcp_pos=tcp_pos)
        scene.attach(arm, prefix=prefix, frame=frame)

    work_xy = [float(x) for x in (cfg.get("work_xy") or [0.38, 0.0])]
    in_xy = [float(x) for x in (cfg.get("in_xy") or [0.38, 0.38])]
    out_xy = [float(x) for x in (cfg.get("out_xy") or [0.38, -0.38])]

    # 工坊草稿 / 采集额外程序化物品
    raw_props = list(cfg.get("props") or []) + list(cfg.get("extra_props") or [])
    if raw_props:
        from teleop_birth.scene.props.catalog import PropSpec
        from teleop_birth.scene.props.spawner import PropSpawner

        project_root = Path(cfg.get("project_root") or Path.cwd())
        specs = [
            p if isinstance(p, PropSpec) else PropSpec.from_dict(dict(p))
            for p in raw_props
        ]
        # 避免与 packages / 草稿内重名
        existing = set(prop_names)
        unique_specs: list = []
        for s in specs:
            if s.name in existing:
                continue
            existing.add(s.name)
            unique_specs.append(s)
        if unique_specs:
            extra = PropSpawner(project_root=project_root).attach_all(scene, unique_specs)
            prop_names.extend(extra)

    if bool(cfg.get("show_base_frames", True)):
        try:
            from teleop_birth.ui.session.base_frame_viz import add_base_frame_geoms_to_spec

            n_viz = add_base_frame_geoms_to_spec(scene, cfg)
            if n_viz:
                print(f"  [base] 已加入 {n_viz} 个 base 坐标系标记（青=接触 / 品红=URDF）")
        except Exception as exc:  # noqa: BLE001
            print(f"  [base] 标记跳过: {exc}")

    model = scene.compile()
    # 编译后再确保指网无碰撞
    for gid in range(model.ngeom):
        bid = int(model.geom_bodyid[gid])
        bn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid) or ""
        if "arm_gripper_left_link" in bn or "arm_gripper_right_link" in bn:
            gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or ""
            if "finger_pad" not in gname:
                model.geom_contype[gid] = 0
                model.geom_conaffinity[gid] = 0
                model.geom_rgba[gid, 3] = 0.0

    name_map = {
        "tcp_sites": {"left": f"L_{TCP_SITE}", "right": f"R_{TCP_SITE}"},
        "arm_joints": {
            "left": [f"L_{j}" for j in ARM_JOINTS],
            "right": [f"R_{j}" for j in ARM_JOINTS],
        },
        "arm_actuators": {
            "left": [f"L_act_{j}" for j in ARM_JOINTS],
            "right": [f"R_act_{j}" for j in ARM_JOINTS],
        },
        "grip_actuators": {
            "left": [f"L_act_{GRIP_LEFT}", f"L_act_{GRIP_RIGHT}"],
            "right": [f"R_act_{GRIP_LEFT}", f"R_act_{GRIP_RIGHT}"],
        },
        "prop_names": prop_names,
        "label_up": label_up,
        "table_z": table_z,
        "work_xy": work_xy,
        "in_conveyor_xy": in_xy,
        "out_conveyor_xy": out_xy,
        "home_q": {"left": home_l, "right": home_r},
        "prefixes": ["L_", "R_"],
    }
    return model, name_map
