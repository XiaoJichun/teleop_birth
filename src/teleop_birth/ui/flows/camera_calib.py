"""相机标定菜单：选场景 → 选相机 → 自动标定并打印型号/内参/外参。"""

from __future__ import annotations

import json
from pathlib import Path

from teleop_birth.scene.plugins.camera import CameraPlugin
from teleop_birth.scene.plugins.scene import compose_scene, load_scene_yaml
from teleop_birth.scene.recording.calibration import (
    build_calibration,
    save_calibration,
    validate_calibration,
)
from teleop_birth.scene.recording.camera import CameraViewSpec, MultiCameraConfig
from teleop_birth.ui import tui
from teleop_birth.ui.scene_builder.draft import list_built_scenes


def action_camera_calib(root: Path) -> None:
    tui.section("相机标定")
    scenes = list_built_scenes(root)
    if not scenes:
        print("没有可用场景。请先「搭建场景」或使用 configs/scenes/。")
        return
    labels = []
    for p in scenes:
        try:
            title = load_scene_yaml(p).get("title") or p.stem
        except Exception:
            title = p.stem
        try:
            rel = p.relative_to(root)
        except ValueError:
            rel = p
        labels.append((str(p), f"{title}  [{rel}]"))
    choice = tui.select("选择场景", labels, allow_cancel=True)
    if not choice:
        return
    scene_path = Path(choice)
    cfg = load_scene_yaml(scene_path)
    ctx = compose_scene(root, scene_path)
    cams = list(ctx.extras.get("cameras") or cfg.get("cameras") or [])
    if not cams:
        print("该场景没有相机条目，将使用默认 D455 + 双腕 D405。")
        cams = CameraPlugin().resolve_views(root, None)

    opts = [(str(i), f"{c.get('name')}  ({c.get('camera_asset') or c.get('role')})") for i, c in enumerate(cams)]
    opts.insert(0, ("all", "全部相机"))
    pick = tui.select("选择需要标定的相机", opts, allow_cancel=True)
    if pick is None:
        return
    if pick == "all":
        selected = cams
    else:
        selected = [cams[int(pick)]]

    # 统一经 CameraPlugin 解析后再标定
    resolved = CameraPlugin().resolve_views(root, selected)
    views = [CameraViewSpec.from_dict(r) for r in resolved]
    payload = build_calibration(
        views=views,
        width=int(resolved[0].get("width", 640)),
        height=int(resolved[0].get("height", 480)),
        source=f"scene:{scene_path.name}",
    )

    errs = validate_calibration(payload)
    if errs:
        print("标定校验失败:")
        for e in errs:
            print(f"  - {e}")
        return

    out_dir = root / "logs" / "calibration" / scene_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    path = save_calibration(payload, out_dir / "calibration.json")

    tui.section("标定结果")
    print(f"  场景: {scene_path.name}")
    print(f"  落盘: {path}")
    for name, c in (payload.get("cameras") or {}).items():
        K = c.get("intrinsics") or {}
        ex = c.get("extrinsics") or {}
        print(f"\n  [{name}]")
        print(f"    型号/资产: {c.get('camera_asset')}")
        print(f"    角色: {c.get('role')}  type={c.get('type')}")
        print(
            f"    内参: fx={K.get('fx'):.2f} fy={K.get('fy'):.2f} "
            f"cx={K.get('cx'):.2f} cy={K.get('cy'):.2f}"
        )
        T = ex.get("T")
        if T:
            print(
                f"    外参 T_parent_cam 平移: "
                f"[{T[0][3]:.4f}, {T[1][3]:.4f}, {T[2][3]:.4f}]  "
                f"parent={ex.get('parent')}"
            )
        print(f"    fovy_deg: {c.get('fovy_deg')}")

    (out_dir / "summary.txt").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"\n  完整 JSON: {path}")
