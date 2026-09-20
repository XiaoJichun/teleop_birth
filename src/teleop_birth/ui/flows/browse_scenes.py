"""插件已部署场景：快速查看 configs/scenes 与 user/ 下已搭建场景。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from teleop_birth.scene.plugins.scene import load_scene_yaml
from teleop_birth.ui import tui
from teleop_birth.ui.scene_builder.draft import list_built_scenes


def action_browse_deployed_scenes(root: Path) -> None:
    tui.section("插件已部署场景")
    scenes = list_built_scenes(root)
    if not scenes:
        print("  （暂无已部署场景）")
        print("  请先「搭建场景」并保存到 configs/scenes/user/")
        return

    print(f"  共 {len(scenes)} 个场景（configs/scenes/*.yaml + user/）\n")
    for i, p in enumerate(scenes, 1):
        info = _scene_summary(root, p)
        print(f"  {i}. {info['title']}")
        print(f"     id={info['id']}  type={info['type']}  物品={info['n_props']}")
        print(f"     {info['rel']}")
        if info.get("robot"):
            print(f"     机器人={info['robot']}  夹爪={info.get('gripper') or '-'}")
        print()

    while True:
        choice = tui.select(
            "操作",
            [
                ("detail", "查看某个场景详情"),
                ("preview", "打开 MuJoCo 预览（只读查看）"),
                ("refresh", "刷新列表"),
                ("back", "返回主菜单"),
            ],
            allow_cancel=True,
        )
        if choice in (None, "back"):
            return
        if choice == "refresh":
            action_browse_deployed_scenes(root)
            return
        path = _pick_scene(root, scenes)
        if path is None:
            continue
        if choice == "detail":
            _print_detail(root, path)
        elif choice == "preview":
            _preview_scene(root, path)


def _pick_scene(root: Path, scenes: list[Path] | None = None) -> Path | None:
    scenes = scenes or list_built_scenes(root)
    if not scenes:
        print("没有场景可选。")
        return None
    labels = []
    for p in scenes:
        info = _scene_summary(root, p)
        labels.append((str(p), f"{info['title']}  [{info['rel']}]"))
    choice = tui.select("选择场景", labels, allow_cancel=True)
    return Path(choice) if choice else None


def _scene_summary(root: Path, path: Path) -> dict[str, Any]:
    rel = str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
    try:
        cfg = load_scene_yaml(path)
    except Exception as exc:  # noqa: BLE001
        return {
            "id": path.stem,
            "title": f"{path.stem} (读取失败)",
            "type": "?",
            "n_props": 0,
            "rel": rel,
            "error": str(exc),
        }
    props = cfg.get("props") or []
    prop_specs = cfg.get("prop_specs") or []
    n_props = len(prop_specs) if prop_specs else (len(props) if isinstance(props, list) else 0)
    backend = cfg.get("backend") or {}
    robot = backend.get("robot_id") or cfg.get("robot_id") or ""
    grippers = cfg.get("grippers") or []
    gid = ""
    if grippers and isinstance(grippers[0], dict):
        gid = str(grippers[0].get("id") or "")
    elif isinstance(grippers, list) and grippers:
        gid = str(grippers[0])
    return {
        "id": str(cfg.get("id") or path.stem),
        "title": str(cfg.get("title") or cfg.get("id") or path.stem),
        "type": str(cfg.get("type") or backend.get("type") or "-"),
        "n_props": n_props,
        "rel": rel,
        "robot": str(robot),
        "gripper": gid,
        "cfg": cfg,
    }


def _print_detail(root: Path, path: Path) -> None:
    info = _scene_summary(root, path)
    cfg = info.get("cfg") or {}
    tui.section(f"场景详情 — {info['title']}")
    print(f"  文件: {info['rel']}")
    print(f"  id: {info['id']}")
    print(f"  type: {info['type']}")
    print(f"  build_mode: {cfg.get('build_mode') or '-'}")
    print(f"  机器人: {info.get('robot') or '-'}")
    print(f"  夹爪: {info.get('gripper') or '-'}")
    mount = cfg.get("mount") or {}
    if mount:
        print(f"  放置: kind={mount.get('kind')}  table_top_z={mount.get('table_top_z')}")
    props = cfg.get("prop_specs") or cfg.get("props") or []
    print(f"  物品数: {len(props) if isinstance(props, list) else 0}")
    for i, p in enumerate((props or [])[:20]):
        if isinstance(p, dict):
            zh = (p.get("labels") or {}).get("zh") or ""
            print(f"    - {p.get('name')}  {zh}  kind={p.get('kind')}  size={p.get('size')}")
        else:
            print(f"    - {p}")
    if isinstance(props, list) and len(props) > 20:
        print(f"    … 另有 {len(props) - 20} 个")
    cams = (cfg.get("camera") or {}).get("extra") or cfg.get("cameras") or []
    if isinstance(cams, list) and cams:
        print(f"  相机: {len(cams)}")
        for c in cams[:8]:
            if isinstance(c, dict):
                print(f"    - {c.get('name')}  role={c.get('role')}")
    notes = cfg.get("notes")
    if notes:
        print(f"  备注: {notes}")
    tui.press_enter("按回车继续…")


def _preview_scene(root: Path, path: Path) -> None:
    """用 MuJoCo viewer 只读打开已部署场景。"""
    print(f"  正在打开预览: {path.name} …")
    try:
        model = _build_scene_model(root, path)
    except Exception as exc:  # noqa: BLE001
        print(f"  预览加载失败: {exc}")
        print("  可改用「Pico仿真遥操 / 自动数据采集」选择该场景启动。")
        tui.press_enter("按回车继续…")
        return

    try:
        import mujoco
        import mujoco.viewer

        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        print("  MuJoCo 窗口已打开；关闭窗口后返回。")
        mujoco.viewer.launch(model, data)
    except Exception as exc:  # noqa: BLE001
        print(f"  viewer 打开失败: {exc}")
        tui.press_enter("按回车继续…")


def _build_scene_model(root: Path, path: Path):
    from omegaconf import OmegaConf

    from teleop_birth.scene.plugins.scene.adapters import resolve_adapter
    from teleop_birth.scene.props.catalog import PropCatalog, PropSpec

    info = resolve_adapter(root, path)
    adapter = str(info.get("adapter") or "")
    cfg = dict(info["ctx"].scene_cfg)
    prop_dicts = _resolve_scene_props(root, cfg)

    if adapter == "logistics_x2arm":
        from teleop_birth.robot.arm.artixon.builder import build_logistics_scene

        module = info.get("module_config") or "configs/robots/artixon_logistics_scene.yaml"
        base = OmegaConf.to_container(OmegaConf.load(root / str(module)), resolve=True)
        assert isinstance(base, dict)
        # 已部署场景预览：保留 yaml 内 packages；若用户场景显式给了 packages 则覆盖
        if "packages" in cfg:
            base["packages"] = list(cfg.get("packages") or [])
        if cfg.get("desktop_base"):
            base["desktop_base"] = dict(cfg["desktop_base"])
        base["props"] = prop_dicts
        base["project_root"] = str(root)
        model, _ = build_logistics_scene(base)
        return model

    if adapter == "dual_ur5e":
        from teleop_birth.robot.arm.ur5e.builder import build_dual_ur5e_2f85

        backend = OmegaConf.to_container(
            OmegaConf.load(root / "configs/device/backend/mujoco_dual_ur.yaml"),
            resolve=True,
        )
        paths = OmegaConf.to_container(
            OmegaConf.load(root / "configs/device/paths/default.yaml"),
            resolve=True,
        )
        assert isinstance(backend, dict) and isinstance(paths, dict)
        scene = dict(backend["scene"])
        scene["props"] = prop_dicts
        scene["project_root"] = str(root)
        # 场景 yaml 的桌面接触点 / arm_id 覆盖 backend 默认（换臂预览关键）
        if cfg.get("desktop_base"):
            scene["desktop_base"] = dict(cfg["desktop_base"])
        menagerie = root / str(paths.get("menagerie", "third_party/mujoco_menagerie"))
        model, _ = build_dual_ur5e_2f85(menagerie, scene)
        return model

    raise ValueError(f"暂不支持预览的场景类型: {adapter or cfg.get('type')}")


def _resolve_scene_props(root: Path, cfg: dict) -> list[dict]:
    """场景 props 可能是名字列表或 dict 列表；优先 prop_specs。"""
    from teleop_birth.scene.props.catalog import PropCatalog

    specs = cfg.get("prop_specs")
    if isinstance(specs, list) and specs and isinstance(specs[0], dict):
        return [dict(p) for p in specs]

    raw = cfg.get("props") or []
    if not raw:
        return []
    if isinstance(raw[0], dict):
        return [dict(p) for p in raw]

    cat = PropCatalog(root / "configs" / "scene" / "props")
    out: list[dict] = []
    for name in raw:
        n = str(name)
        try:
            out.append(cat.get(n).to_dict())
        except KeyError:
            print(f"  警告: 物品「{n}」不在模型库，已跳过")
    return out
