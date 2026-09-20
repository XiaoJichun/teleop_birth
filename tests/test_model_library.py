"""模型库：默认库 + 可配置路径 + 拉取进草稿。"""

from __future__ import annotations

from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_default_model_library_has_builtins_and_extras():
    from teleop_birth.scene.props.catalog import PropCatalog, resolve_library_dirs

    dirs = resolve_library_dirs(ROOT)
    assert dirs
    assert dirs[0].name == "props" or "props" in str(dirs[0])
    cat = PropCatalog.from_project(ROOT)
    names = set(cat.list_names())
    for need in (
        "red_cube",
        "blue_plate",
        "green_cube",
        "yellow_cylinder",
        "gray_box",
        "white_sphere",
    ):
        assert need in names, need
    entries = cat.list_entries()
    ids = [n for n, _ in entries]
    assert "red_cube" in ids
    assert "_template" not in ids
    assert not any(n.startswith("sem_") for n in ids)
    # 中文显示
    by = dict(entries)
    assert "红色" in by["red_cube"] or "red_cube" in by["red_cube"]


def test_library_override_roundtrip(tmp_path: Path):
    from teleop_birth.scene.props.catalog import (
        PropCatalog,
        resolve_library_dirs,
        save_library_override,
    )

    # 用临时项目根：复制 config + 自建库
    proj = tmp_path / "proj"
    cfg_dir = proj / "configs" / "scene"
    cfg_dir.mkdir(parents=True)
    (cfg_dir / "model_library.yaml").write_text(
        "default_root: configs/scene/props\noverride_root: ''\nextra_roots: []\n",
        encoding="utf-8",
    )
    props = proj / "configs" / "scene" / "props"
    props.mkdir(parents=True)
    (props / "red_cube.yaml").write_text(
        "name: red_cube\nkind: box\nsize: [0.02,0.02,0.02]\nlabels: {zh: 红}\n",
        encoding="utf-8",
    )
    custom = tmp_path / "my_lib"
    custom.mkdir()
    (custom / "orange_box.yaml").write_text(
        "name: orange_box\nkind: box\nsize: [0.03,0.03,0.03]\nlabels: {zh: 橙色盒}\n",
        encoding="utf-8",
    )

    save_library_override(proj, custom)
    dirs = resolve_library_dirs(proj)
    assert dirs[0].resolve() == custom.resolve()
    cat = PropCatalog.from_project(proj)
    assert "orange_box" in cat.list_names()
    assert "橙色" in dict(cat.list_entries())["orange_box"]

    save_library_override(proj, None)
    dirs2 = resolve_library_dirs(proj)
    assert "props" in str(dirs2[0])


def test_catalog_pull_into_draft_compiles():
    from teleop_birth.scene.props.catalog import PropCatalog
    from teleop_birth.ui.scene_builder.draft import SceneDraft
    from teleop_birth.ui.scene_builder.preview import PreviewSession

    cat = PropCatalog.from_project(ROOT)
    draft = SceneDraft(
        mode="catalog",
        robot_id="ur5e",
        robot_title="UR5e",
        robot_category="arm",
        mount_kind="table",
        gripper_id="robotiq_2f85",
        desktop_base_arm="ur5e",
    )
    for name in ("red_cube", "blue_plate", "green_cube"):
        prop = cat.get(name).to_dict()
        draft.props.append(prop)
    ps = PreviewSession(ROOT, draft)
    assert ps.sync_draft(draft) is True
    assert ps._model is not None
    # 二次拉取同名 → 重命名后仍可编译
    prop = cat.get("red_cube").to_dict()
    prop["name"] = "red_cube_1"
    draft.props.append(prop)
    assert ps.sync_draft(draft) is True
