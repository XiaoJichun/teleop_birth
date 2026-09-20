"""随机化区域解析与防碰撞采样。"""

from __future__ import annotations

import numpy as np
import pytest

from teleop_birth.scene.placement.randomizer import LayoutRandomizer
from teleop_birth.ui.session.randomize import (
    parse_region_text,
    region_from_arrays,
    sample_non_colliding,
)
from teleop_birth.ui.session.round_layouts import pregenerate_episode_layouts


def test_parse_region_meters_and_mm():
    off, size = parse_region_text("[[0.15, 0.40], [0.25, 0.20]]")
    assert off == pytest.approx([0.15, 0.40])
    assert size == pytest.approx([0.25, 0.20])
    off2, size2 = parse_region_text("[[150,400],[250,200]]")
    assert off2 == pytest.approx([0.15, 0.40])
    assert size2 == pytest.approx([0.25, 0.20])


def test_region_world_bounds_relative_to_base():
    r = region_from_arrays(
        [0.15, 0.40],
        [0.20, 0.10],
        base_xy=(0.0, 0.0),
        base_z=0.42,
        min_separation_m=0.08,
    )
    b = r.world_bounds()
    assert b["x_min"] == pytest.approx(0.05)
    assert b["x_max"] == pytest.approx(0.25)
    assert b["y_min"] == pytest.approx(0.35)
    assert b["y_max"] == pytest.approx(0.45)


def test_sample_non_colliding_two_objects():
    r = region_from_arrays([0.2, 0.4], [0.4, 0.3], min_separation_m=0.1)
    pts = sample_non_colliding(r, 2, seed=1)
    assert len(pts) == 2
    d = float(np.linalg.norm(pts[0][:2] - pts[1][:2]))
    assert d >= 0.1 - 1e-6


def test_layout_randomizer_from_region_dict():
    r = region_from_arrays([0.15, 0.40], [0.25, 0.20], min_separation_m=0.08)
    rnd = LayoutRandomizer.from_region_dict(r.to_dict(), seed=0)
    assert rnd.workspace.min_separation_m == pytest.approx(0.08)
    assert rnd.workspace.x_max > rnd.workspace.x_min


def test_pregenerate_episode_layouts():
    from pathlib import Path as P

    root = P(__file__).resolve().parents[1]
    r = region_from_arrays([0.15, 0.40], [0.30, 0.25], min_separation_m=0.08)
    layouts = pregenerate_episode_layouts(
        root,
        prop_names=["red_cube", "blue_plate"],
        region=r,
        episodes=3,
        seed=0,
    )
    assert len(layouts) == 3
    assert "red_cube" in layouts[0] and "blue_plate" in layouts[0]
