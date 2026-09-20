"""导出格式与遥操轮次计划自测。"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from teleop_birth.scene.recording.export_formats import export_episode, export_lerobot
from teleop_birth.teleop.plan_rounds import inject_plan_props_into_scene
from teleop_birth.ui.session.randomize import region_from_arrays
from teleop_birth.ui.session.round_layouts import pregenerate_episode_layouts


ROOT = Path(__file__).resolve().parents[1]


def test_pregenerate_layouts_non_colliding():
    region = region_from_arrays([0.15, 0.40], [0.30, 0.25], min_separation_m=0.08)
    layouts = pregenerate_episode_layouts(
        ROOT,
        prop_names=["red_cube", "blue_plate"],
        region=region,
        episodes=3,
        seed=0,
    )
    assert len(layouts) == 3
    for lay in layouts:
        assert "red_cube" in lay and "blue_plate" in lay
        a = lay["red_cube"]["pos"][:2]
        b = lay["blue_plate"]["pos"][:2]
        dist = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        assert dist >= 0.08 - 1e-5


def test_inject_plan_props():
    plan = {"props": ["red_cube"]}
    scene = inject_plan_props_into_scene({"base_height": 0.42}, plan, ROOT)
    assert isinstance(scene["props"], list)
    assert scene["props"][0]["name"] == "red_cube"


def test_export_lerobot(tmp_path: Path):
    mcap = tmp_path / "ep.mcap"
    mcap.write_bytes(b"fake-mcap-bytes")
    out = export_lerobot(mcap, version="2.1", meta={"task": "demo", "steps": 10})
    assert (out / "meta" / "info.json").is_file()
    assert (out / "data" / "ep.mcap").is_file()
    info = json.loads((out / "meta" / "info.json").read_text(encoding="utf-8"))
    assert info["codebase_version"] == "2.1"


def test_export_hdf5_optional(tmp_path: Path):
    pytest.importorskip("h5py")
    mcap = tmp_path / "ep.mcap"
    mcap.write_bytes(b"abc123")
    out = export_episode(mcap, dataset_format="hdf5", meta={"episode": 0})
    assert out is not None and out.suffix == ".hdf5"
    assert out.is_file()
