"""相机视野调参：MuJoCo 被动窗 + 终端步进，实时看视野并写回 YAML。"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Optional

import numpy as np
import yaml

SelectFn = Callable[..., Optional[str]]


def _load_scene(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8")) or {}


def _save_scene(path: Path, data: dict[str, Any]) -> None:
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")


def run_camera_tuner(
    project_root: Path,
    scene_yaml: Path | None = None,
    *,
    select: SelectFn | None = None,
) -> None:
    """调参入口。select 不传则用编号输入，避免 plugins → cli。"""
    root = Path(project_root)

    def _select(prompt: str, options: list[tuple[str, str]], *, allow_cancel: bool = True) -> str | None:
        if select is not None:
            return select(prompt, options, allow_cancel=allow_cancel)
        print(f"\n{prompt}")
        for i, (_value, label) in enumerate(options, 1):
            print(f"  {i}. {label}")
        if allow_cancel:
            print("  0. 取消")
        raw = input("> ").strip()
        if allow_cancel and raw in ("", "0", "q"):
            return None
        try:
            idx = int(raw) - 1
        except ValueError:
            return None
        if idx < 0 or idx >= len(options):
            return None
        return options[idx][0]

    scenes_dir = root / "configs" / "scenes"
    if scene_yaml is None:
        cands = sorted(scenes_dir.glob("*.yaml")) if scenes_dir.is_dir() else []
        if not cands:
            fallback = root / "configs" / "scene" / "recording.yaml"
            if not fallback.is_file():
                print("未找到 configs/scenes/*.yaml，请先创建场景清单。")
                return
            scene_yaml = fallback
            data = _load_scene(fallback)
            cams = list((data.get("cameras") or {}).get("views") or [])
        else:
            labels = [(str(p), p.name) for p in cands]
            choice = _select("选择场景 YAML", labels, allow_cancel=True)
            if not choice:
                return
            scene_yaml = Path(choice)
            data = _load_scene(scene_yaml)
            cams = list(data.get("cameras") or [])
    else:
        scene_yaml = Path(scene_yaml)
        data = _load_scene(scene_yaml)
        cams = list(data.get("cameras") or data.get("views") or [])
        if not cams and isinstance(data.get("cameras"), dict):
            cams = list((data["cameras"] or {}).get("views") or [])

    if not cams:
        print("场景中无 cameras 配置。")
        return

    options = []
    for i, c in enumerate(cams):
        name = str(c.get("name") or f"cam_{i}")
        options.append((str(i), f"{name} ({c.get('type', 'world')})"))
    idx_s = _select("选择要调节的相机", options, allow_cancel=True)
    if idx_s is None:
        return
    idx = int(idx_s)
    cam = dict(cams[idx])
    if str(cam.get("type") or "world").lower() != "world":
        print("当前 Tuner MVP 仅支持 type=world 相机；腕部相机请改 offset 后重开场景验证。")

    pos = np.array(cam.get("pos") or [1.0, 0.0, 1.0], dtype=float)
    lookat = np.array(cam.get("lookat") or [0.3, 0.0, 0.5], dtype=float)
    fovy = float(cam.get("fovy_deg") or 55.0)
    step = 0.02

    print("\n相机调参（终端步进）")
    print("  w/s: X±  a/d: Y±  r/f: Z±  |  i/k lookat X  j/l lookat Y  u/o lookat Z")
    print("  [/]: fovy±  +/-: 步长  p: 预览一帧  Enter: 保存并退出  q: 放弃\n")

    viewer = None
    model = data_mj = None
    try:
        import mujoco
        from mujoco import viewer as mj_viewer

        spec = mujoco.MjSpec()
        spec.modelname = "camera_tuner"
        wb = spec.worldbody
        wb.add_light(pos=[0, 0, 2], dir=[0, 0, -1])
        wb.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.1], rgba=[0.9, 0.9, 0.9, 1])
        box = wb.add_body(name="work", pos=[0.4, 0.0, 0.45])
        box.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2, 0.3, 0.02], rgba=[0.8, 0.7, 0.5, 1])
        cam_body = wb.add_body(name="tune_cam", pos=pos.tolist())
        cam_body.add_camera(name="tune_cam", pos=[0, 0, 0], fovy=fovy)
        model = spec.compile()
        data_mj = mujoco.MjData(model)
        viewer = mj_viewer.launch_passive(model, data_mj)
    except Exception as exc:
        print(f"[tuner] MuJoCo 预览窗未打开（仍可用终端调参）: {exc}")

    def _sync_viewer() -> None:
        if viewer is not None:
            try:
                viewer.cam.lookat[:] = lookat
                viewer.cam.distance = float(np.linalg.norm(pos - lookat))
                viewer.sync()
            except Exception:
                pass

    def _preview_frame() -> None:
        if model is None or data_mj is None:
            print(f"  pos={pos.tolist()} lookat={lookat.tolist()} fovy={fovy:.1f}")
            return
        try:
            import mujoco

            mujoco.mj_forward(model, data_mj)
            renderer_cls = getattr(mujoco, "Renderer", None)
            if renderer_cls is None:
                print(f"  pos={pos.tolist()} lookat={lookat.tolist()} fovy={fovy:.1f}")
                return
            renderer = renderer_cls(model, height=240, width=320)
            renderer.update_scene(data_mj, camera=-1)
            print(f"  预览就绪 pos={np.round(pos,3)} lookat={np.round(lookat,3)} fovy={fovy:.1f}")
            out = Path(project_root) / "logs" / "camera_tuner_preview.png"
            out.parent.mkdir(parents=True, exist_ok=True)
            try:
                from PIL import Image

                rgb = renderer.render()
                Image.fromarray(rgb).save(out)
                print(f"  已写预览: {out}")
            except Exception:
                pass
            renderer.close()
        except Exception as exc:
            print(f"  预览失败: {exc}")

    keymap = {
        "w": ("pos", 0, +1),
        "s": ("pos", 0, -1),
        "a": ("pos", 1, -1),
        "d": ("pos", 1, +1),
        "r": ("pos", 2, +1),
        "f": ("pos", 2, -1),
        "i": ("lookat", 0, +1),
        "k": ("lookat", 0, -1),
        "j": ("lookat", 1, -1),
        "l": ("lookat", 1, +1),
        "u": ("lookat", 2, +1),
        "o": ("lookat", 2, -1),
    }

    while True:
        print(f"pos={np.round(pos, 3)} lookat={np.round(lookat, 3)} fovy={fovy:.1f} step={step:.3f}")
        cmd = input("> ").strip().lower()
        if cmd in ("", "enter", "save", "y"):
            cam["pos"] = [float(x) for x in pos]
            cam["lookat"] = [float(x) for x in lookat]
            cam["fovy_deg"] = float(fovy)
            cams[idx] = cam
            if "cameras" in data and isinstance(data["cameras"], list):
                data["cameras"] = cams
            elif "cameras" in data and isinstance(data["cameras"], dict):
                data["cameras"]["views"] = cams
            else:
                data["cameras"] = cams
            _save_scene(scene_yaml, data)
            print(f"已保存 → {scene_yaml}")
            break
        if cmd in ("q", "quit", "n"):
            print("已放弃修改。")
            break
        if cmd == "p":
            _preview_frame()
            continue
        if cmd == "+":
            step *= 1.5
            continue
        if cmd == "-":
            step /= 1.5
            continue
        if cmd == "[":
            fovy = max(20.0, fovy - 2.0)
            _sync_viewer()
            continue
        if cmd == "]":
            fovy = min(120.0, fovy + 2.0)
            _sync_viewer()
            continue
        if cmd in keymap:
            target, axis, sign = keymap[cmd]
            if target == "pos":
                pos[axis] += sign * step
            else:
                lookat[axis] += sign * step
            _sync_viewer()
            continue
        print("未知命令。")

    if viewer is not None:
        try:
            viewer.close()
        except Exception:
            pass
