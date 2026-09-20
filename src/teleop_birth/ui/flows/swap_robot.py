"""场景更换机器人：在已搭建场景内无损换臂（桌面接触点固定）。"""

from __future__ import annotations

from pathlib import Path

from teleop_birth.robot import list_robots
from teleop_birth.scene.ops.swap_arm import (
    extract_contact,
    infer_robot_id,
    swap_scene_robot,
    supported_robot_ids,
    write_scene_yaml,
)
from teleop_birth.scene.plugins.scene import load_scene_yaml
from teleop_birth.ui import tui
from teleop_birth.ui.flows.session_common import pick_built_scene
from teleop_birth.ui.scene_builder.draft import user_scenes_dir


def action_swap_scene_robot(root: Path) -> None:
    tui.section("场景更换机器人")
    print(
        "  无损换臂：桌面接触点 (table_top_z / xy / yaw) 不变；\n"
        "  仅更换机械臂型号、夹爪与腕部相机挂点（base 仍在底座↔桌面交汇处）。"
    )

    scene_path = pick_built_scene(root)
    if scene_path is None:
        return
    try:
        scene = load_scene_yaml(scene_path)
    except Exception as exc:  # noqa: BLE001
        print(f"  读取场景失败: {exc}")
        return

    contact = extract_contact(scene)
    cur_robot = infer_robot_id(scene)
    cur_arm = str((scene.get("desktop_base") or {}).get("arm_id") or "")
    print(f"  当前场景: {scene.get('title') or scene_path.name}")
    print(f"  当前机器人: {cur_robot}  (desktop_base.arm_id={cur_arm or '—'})")
    print(
        f"  接触点(固定): table_top_z={contact.table_top_z:.4f}  "
        f"xy=[{contact.xy[0]:.3f}, {contact.xy[1]:.3f}]  yaw={contact.yaw_rad:.4f}"
    )

    robots = [
        r
        for r in list_robots(category="arm", ready_only=True)
        if r.id in supported_robot_ids()
    ]
    if not robots:
        print("  没有可更换的 ready 机械臂。")
        return
    opts = []
    for r in robots:
        mark = "（当前）" if r.id == cur_robot else ""
        opts.append((r.id, f"{r.title}  [{r.id}]{mark}"))
    opts.append(("cancel", "取消"))
    choice = tui.select("选择目标机械臂", opts, allow_cancel=True)
    if choice in (None, "cancel"):
        return
    if choice == cur_robot:
        print("  目标与当前相同，无需更换。")
        return

    try:
        result = swap_scene_robot(scene, new_robot_id=str(choice))
    except Exception as exc:  # noqa: BLE001
        print(f"  换臂失败: {exc}")
        return

    print()
    print(f"  {result.old_robot_id} → {result.new_robot_id}")
    print(f"  desktop_base.arm_id: {result.old_arm_id} → {result.new_arm_id}")
    print(
        f"  URDF mount_z（接触点上方）: {result.mount_z_old:.4f} → {result.mount_z_new:.4f}"
    )
    print(
        f"  接触点保持: table_top_z={result.contact.table_top_z:.4f}  "
        f"xy={list(result.contact.xy)}"
    )
    wrist = [
        f"{c.get('name')}→{c.get('site')}"
        for c in result.scene.get("cameras") or []
        if c.get("role") == "wrist"
    ]
    if wrist:
        print(f"  腕部相机: {', '.join(wrist)}")

    # 写出策略：库存场景 → 复制到 user/；用户场景可选覆盖或另存
    is_user = "scenes/user" in str(scene_path).replace("\\", "/")
    default_id = str(result.scene.get("id") or scene_path.stem)
    if is_user:
        mode = tui.select(
            "保存方式",
            [
                ("overwrite", f"覆盖当前文件  [{scene_path.name}]"),
                ("copy", "另存为新场景（推荐保留原场景）"),
                ("cancel", "取消"),
            ],
            allow_cancel=True,
        )
        if mode in (None, "cancel"):
            return
        if mode == "overwrite":
            out_path = scene_path
            result.scene["id"] = default_id
        else:
            new_id = tui.text(
                "新场景 id",
                default=f"{default_id}__{result.new_robot_id}",
            )
            if not new_id:
                return
            sid = "".join(c if c.isalnum() or c in "-_" else "_" for c in new_id.strip())
            result.scene["id"] = sid
            out_path = user_scenes_dir(root) / f"{sid}.yaml"
    else:
        print("  库存场景不会被覆盖，将保存到 configs/scenes/user/")
        new_id = tui.text(
            "新场景 id",
            default=f"{default_id}__{result.new_robot_id}",
        )
        if not new_id:
            return
        sid = "".join(c if c.isalnum() or c in "-_" else "_" for c in new_id.strip())
        result.scene["id"] = sid
        out_path = user_scenes_dir(root) / f"{sid}.yaml"

    if not tui.confirm(f"写入 {out_path.relative_to(root) if out_path.is_relative_to(root) else out_path}？", default=True):
        print("已取消。")
        return

    write_scene_yaml(out_path, result.scene)
    print(f"  已保存: {out_path}")

    if tui.confirm("是否在 MuJoCo 中预览换臂后场景？", default=True):
        _preview(root, out_path)


def _preview(root: Path, path: Path) -> None:
    try:
        from teleop_birth.ui.flows.browse_scenes import _build_scene_model
        import mujoco
        import mujoco.viewer

        model = _build_scene_model(root, path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print("  MuJoCo 预览已开；关闭窗口后返回。")
        mujoco.viewer.launch(model, data)
    except Exception as exc:  # noqa: BLE001
        print(f"  预览失败: {exc}")
        print("  场景 YAML 已保存，可稍后在「插件已部署场景」中预览。")
