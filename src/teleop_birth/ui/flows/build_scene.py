"""搭建场景：语义搭建 / 调用搭建 → 选机器人 → 选兼容夹爪 → MuJoCo + PyQt。"""

from __future__ import annotations

from pathlib import Path

from teleop_birth.robot import get_robot, list_robots
from teleop_birth.robot.registry import grippers_compatible_with
from teleop_birth.scene.plugins.gripper import GripperPlugin
from teleop_birth.ui import tui
from teleop_birth.ui.scene_builder.draft import SceneDraft, save_scene_draft
from teleop_birth.ui.scene_builder.gripper_util import register_gripper_in_catalog


GRIPPER_ADD_RULES = """
添加夹爪/灵巧手规则
─────────────────
1. 登记到 src/teleop_birth/scene/assets/catalog.yaml → grippers:
2. 本地模型：source: local，key 指向 assets 下相对路径（需 MJCF .xml 才能挂 UR）
3. Menagerie：source: menagerie，key 如 robotiq_2f85/2f85.xml
4. 可选字段：title, attach_site, builtin: true（内置于臂）
5. 在对应机器人的 RobotSpec.compatible_grippers 中登记兼容 id
6. 保存后重新进入「搭建场景」即可在列表中看到新夹爪
"""


def _desktop_arm_id(robot_id: str) -> str:
    mapping = {
        "ur5e": "ur5e",
        "artixon_arm_6a": "artixon_6a",
        "artixon_6a": "artixon_6a",
        "franka_panda": "franka_panda",
    }
    return mapping.get(robot_id, "ur5e")


def action_build_scene(root: Path) -> None:
    tui.section("搭建场景")
    mode = tui.select(
        "搭建方式",
        [
            ("semantic", "语义搭建场景（文本语义生成物品 + 编辑）"),
            ("catalog", "调用搭建场景（从三维模型库拉取物品）"),
            ("cancel", "取消"),
        ],
        allow_cancel=True,
    )
    if mode in (None, "cancel"):
        return
    _build_pipeline(root, mode=str(mode))  # type: ignore[arg-type]


def _build_pipeline(root: Path, *, mode: str) -> None:
    robot = _pick_robot(root)
    if robot is None:
        return
    robot_id, title, category = robot
    spec = get_robot(robot_id)

    if not spec.scene_preview or spec.status != "ready":
        print(
            f"  ✗ 「{title}」尚未接入场景搭建（status={spec.status}，"
            f"scene_preview={spec.scene_preview}）。\n"
            f"    请选择已就绪机型：UR5e / ArtiXon Arm-6A。"
        )
        tui.press_enter("按回车返回…")
        return

    mount = "empty" if category in ("biped", "wheeled_humanoid") else "table"
    print(f"  → 机器人: {title} ({robot_id})")
    print(
        f"  → 放置: {'空白场景（人形/轮式）' if mount == 'empty' else '桌面 + 机器人（单臂/双臂）'}"
    )
    allowed = grippers_compatible_with(robot_id)
    print(f"  → 兼容末端: {', '.join(allowed) if allowed else '（无）'}")

    grip = _pick_gripper(root, robot_id=robot_id)
    if grip is None:
        return
    if grip == "__add__":
        _wizard_add_gripper(root)
        return
    gid, gtitle = grip

    draft = SceneDraft(
        mode="semantic" if mode == "semantic" else "catalog",  # type: ignore[arg-type]
        robot_id=robot_id,
        robot_title=title,
        robot_category=category,
        mount_kind=mount,  # type: ignore[arg-type]
        gripper_id=gid,
        gripper_title=gtitle,
        desktop_base_arm=_desktop_arm_id(robot_id),
    )
    if draft.mode == "catalog":
        draft.props = _seed_props_from_catalog(root)

    preview = None
    if tui.confirm("是否打开 MuJoCo 预览窗口（与 PyQt 工坊并用）？", default=True):
        from teleop_birth.ui.scene_builder.preview import PreviewSession

        preview = PreviewSession(root, draft)
        preview.start()

    from teleop_birth.ui.scene_builder.workshop_qt import run_scene_workshop

    try:
        saved = run_scene_workshop(root, draft, preview=preview)
    except ImportError as exc:
        print(f"\nPyQt 工坊不可用: {exc}")
        print("可安装: uv sync --extra viz  或重新 bash start.sh")
        if tui.confirm("是否将当前草稿保存为场景 YAML？", default=True):
            sid = tui.text("场景 id", default=f"{robot_id}_{gid}")
            if sid:
                path = save_scene_draft(root, draft, scene_id=sid.strip())
                print(f"  已保存: {path}")
        if preview is not None:
            preview.stop()
        return

    if preview is not None:
        preview.stop()
    if saved:
        print(f"\n场景已保存: {saved}")


def _pick_robot(root: Path) -> tuple[str, str, str] | None:
    """搭建场景只列出可预览的 ready 机型；stub 单独提示不可选。"""
    ready = list_robots(preview_only=True)
    stubs = [r for r in list_robots() if not r.scene_preview]
    if not ready:
        print("没有可搭建预览的机器人。")
        return None
    opts = [
        (
            r.id,
            f"{r.title}  [ready]  末端={','.join(r.compatible_grippers) or '—'}"
            + (f" — {r.notes}" if r.notes else ""),
        )
        for r in ready
    ]
    if stubs:
        opts.append(
            (
                "__stubs__",
                f"（另有 {len(stubs)} 个占位机型不可搭建："
                + "、".join(s.title for s in stubs)
                + "）",
            )
        )
    rid = tui.select("选择机器人", opts, allow_cancel=True)
    if not rid:
        return None
    if rid == "__stubs__":
        print("  占位机型尚未接入 MuJoCo 场景搭建，请选上方 ready 机型。")
        tui.press_enter("按回车返回…")
        return None
    r = get_robot(rid)
    return r.id, r.title, r.category


def _pick_gripper(root: Path, *, robot_id: str) -> tuple[str, str] | None | str:
    """只列出与当前机器人兼容的夹爪；无兼容项则提示并返回。"""
    allowed = set(grippers_compatible_with(robot_id))
    items = GripperPlugin().list_available(root, robot_id=robot_id)
    if not items:
        print(
            f"  ✗ 机器人「{robot_id}」当前没有兼容夹爪。\n"
            f"    允许的 id: {sorted(allowed) or '（空）'}；请在 RobotSpec / catalog 中登记。"
        )
        tui.press_enter("按回车返回…")
        return None
    opts = [(g["id"], f"{g['title']}  [{g['id']}]") for g in items]
    opts.append(("__add__", "＋ 添加夹爪/灵巧手（向导写入 catalog，仍需登记兼容）"))
    choice = tui.select("选择夹爪/灵巧手（已按机器人过滤）", opts, allow_cancel=True)
    if choice is None:
        return None
    if choice == "__add__":
        return "__add__"
    g = next(x for x in items if x["id"] == choice)
    return str(g["id"]), str(g["title"])


def _wizard_add_gripper(root: Path) -> None:
    tui.section("添加夹爪/灵巧手")
    print(GRIPPER_ADD_RULES)
    if not tui.confirm("按向导写入 catalog.yaml？", default=True):
        return
    gid = tui.text("夹爪 id（字母数字_-）", default="my_gripper")
    if not gid:
        return
    title = tui.text("显示名称", default=gid) or gid
    source = tui.select(
        "资产来源",
        [
            ("menagerie", "menagerie（third_party/mujoco_menagerie 相对 key）"),
            ("local", "local（scene/assets 下相对 key）"),
            ("local_artixon", "local_artixon"),
        ],
    )
    if not source:
        return
    key = tui.text(
        "相对路径 key（如 robotiq_2f85/2f85.xml）",
        default="my_gripper/model.xml",
    )
    if not key:
        return
    attach = tui.text("attach_site", default="attachment_site") or "attachment_site"
    builtin = tui.confirm("是否 builtin（内置于臂，不单独挂 MJCF）？", default=False)
    try:
        path = register_gripper_in_catalog(
            root,
            gripper_id=gid.strip(),
            source=str(source),
            key=key.strip(),
            title=str(title).strip(),
            attach_site=attach.strip(),
            builtin=bool(builtin),
        )
        print(f"  已写入: {path}")
        print("  请在对应机器人的 compatible_grippers 中登记后，重新进入「搭建场景」。")
    except Exception as exc:  # noqa: BLE001
        print(f"  写入失败: {exc}")
    tui.press_enter("按回车返回…")


def _seed_props_from_catalog(root: Path) -> list[dict]:
    """调用模式：默认空列表，由工坊「从模型库拉取」逐个加入（避免一进来就塞满且重名失败）。"""
    return []
