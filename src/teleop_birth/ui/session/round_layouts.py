"""遥操多轮随机化：预采样布局 + 按轮次应用。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

import yaml

from teleop_birth.scene.placement.randomizer import LayoutRandomizer
from teleop_birth.scene.props.catalog import PropCatalog, PropSpec
from teleop_birth.ui.session.randomize import RandomizeRegion, region_from_arrays


def region_from_plan_dict(data: dict[str, Any] | None) -> RandomizeRegion | None:
    if not data:
        return None
    if "world_bounds" in data and "offset" in data:
        return RandomizeRegion(
            offset=tuple(data["offset"]),
            size=tuple(data["size"]),
            dim=int(data.get("dim", 2)),
            min_separation_m=float(data.get("min_separation_m", 0.08)),
            base_xy=tuple(data.get("base_xy") or (0.0, 0.0)),  # type: ignore[arg-type]
            base_z=float(data.get("base_z", 0.42)),
        )
    # 采集会话传入的世界包围盒：{x_min,x_max,y_min,y_max,z,...}
    if "x_min" in data and "x_max" in data and "y_min" in data and "y_max" in data:
        x0, x1 = float(data["x_min"]), float(data["x_max"])
        y0, y1 = float(data["y_min"]), float(data["y_max"])
        z = float(data.get("z", data.get("base_z", 0.42)))
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        sx, sy = abs(x1 - x0), abs(y1 - y0)
        return RandomizeRegion(
            offset=(cx, cy),
            size=(sx, sy),
            dim=2,
            min_separation_m=float(data.get("min_separation_m", 0.08)),
            base_xy=(0.0, 0.0),
            base_z=float(z - 0.025),
        )
    off = list(data.get("offset") or [])
    size = list(data.get("size") or [])
    if len(off) >= 2 and len(size) >= 2:
        return region_from_arrays(
            off,
            size,
            base_xy=data.get("base_xy") or (0.0, 0.0),
            base_z=float(data.get("base_z", 0.42)),
            min_separation_m=float(data.get("min_separation_m", 0.08)),
        )
    return None


def resolve_prop_specs(
    project_root: Path,
    names: Sequence[str],
    *,
    prop_specs_inline: Sequence[dict[str, Any]] | None = None,
) -> list[PropSpec]:
    cat = PropCatalog(project_root / "configs" / "scene" / "props")
    if prop_specs_inline:
        for raw in prop_specs_inline:
            if isinstance(raw, dict) and raw.get("name"):
                cat.register(PropSpec.from_dict(raw))
    return cat.resolve_many(list(names))


def pregenerate_episode_layouts(
    project_root: Path,
    *,
    prop_names: Sequence[str],
    region: RandomizeRegion,
    episodes: int,
    seed: int = 0,
    prop_specs_inline: Sequence[dict[str, Any]] | None = None,
) -> list[dict[str, dict]]:
    """为每一轮预采样无碰撞布局（pos + quat_xyzw）。"""
    specs = resolve_prop_specs(project_root, prop_names, prop_specs_inline=prop_specs_inline)
    rnd = LayoutRandomizer.from_region_dict(region.to_dict(), seed=seed)
    layouts: list[dict[str, dict]] = []
    for ep in range(int(episodes)):
        rnd.rng = __import__("numpy").random.default_rng(seed + ep * 997)
        layouts.append(rnd.sample_layout(specs))
    return layouts


def save_plan_with_layouts(plan_path: Path, payload: dict[str, Any]) -> Path:
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        __import__("json").dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return plan_path


def load_plan(plan_path: Path) -> dict[str, Any]:
    import json

    return json.loads(Path(plan_path).read_text(encoding="utf-8"))
