"""采集数据多格式导出：MCAP → HDF5 / LeRobot v2.1 / LeRobot v3.0（含轨迹张量）。"""

from __future__ import annotations

import json
import shutil
from pathlib import Path
from typing import Any

import numpy as np

from teleop_birth.scene.recording.mcap_episode import (
    action_14,
    observation_state,
    parse_mcap_episode,
)


def export_episode(
    mcap_path: Path,
    *,
    dataset_format: str,
    meta: dict[str, Any] | None = None,
) -> Path | None:
    """根据格式导出；mcap 原样返回；未知格式返回 None。"""
    fmt = str(dataset_format or "mcap").lower().replace("-", "_").replace(".", "_")
    src = Path(mcap_path)
    if not src.is_file():
        return None
    if fmt in ("mcap", ""):
        return src
    if fmt in ("hdf5", "h5"):
        return export_hdf5(src, meta=meta)
    if fmt in ("lerobot_v2_1", "lerobotv2_1", "lerobot_v2"):
        return export_lerobot(src, version="2.1", meta=meta)
    if fmt in ("lerobot_v3_0", "lerobotv3_0", "lerobot_v3"):
        return export_lerobot(src, version="3.0", meta=meta)
    print(f"[export] 未知格式 {dataset_format}，保留 MCAP")
    return src


def export_hdf5(mcap_path: Path, *, meta: dict[str, Any] | None = None) -> Path:
    """解析 MCAP 轨迹写入 HDF5；解析失败时仍保留 mcap_bytes。"""
    try:
        import h5py
    except ImportError as exc:
        raise ImportError("HDF5 导出需要 h5py：uv add h5py 或 pip install h5py") from exc

    ep = parse_mcap_episode(mcap_path)
    out = mcap_path.with_suffix(".hdf5")
    merged_meta = {**(meta or {}), **(ep.meta or {})}

    with h5py.File(out, "w") as f:
        f.attrs["source_mcap"] = str(mcap_path.name)
        f.attrs["format"] = "teleop_birth_hdf5_v2"
        f.create_dataset(
            "meta_json",
            data=np.frombuffer(
                json.dumps(merged_meta, ensure_ascii=False).encode("utf-8"), dtype="S1"
            ),
        )
        if len(ep.timestamps_ns):
            f.create_dataset("timestamps_ns", data=ep.timestamps_ns)
        if len(ep.ee_left):
            f.create_dataset("observation/ee_left", data=ep.ee_left, compression="gzip")
        if len(ep.ee_right):
            f.create_dataset("observation/ee_right", data=ep.ee_right, compression="gzip")
        state = observation_state(ep)
        if len(state):
            f.create_dataset("observation/state", data=state, compression="gzip")
        act = action_14(ep)
        if len(act):
            f.create_dataset("action", data=act, compression="gzip")
        if len(ep.gripper):
            f.create_dataset("observation/gripper", data=ep.gripper, compression="gzip")
        for cam, frames in (ep.images or {}).items():
            if not frames:
                continue
            # 统一裁到最小公共尺寸
            h = min(im.shape[0] for im in frames)
            w = min(im.shape[1] for im in frames)
            stack = np.stack([im[:h, :w] for im in frames], axis=0)
            f.create_dataset(
                f"observation/images/{cam}",
                data=stack,
                compression="gzip",
            )
        # 始终附带原始 mcap，便于溯源
        raw = np.frombuffer(mcap_path.read_bytes(), dtype=np.uint8)
        f.create_dataset("mcap_bytes", data=raw, compression="gzip")
    return out


def export_lerobot(
    mcap_path: Path,
    *,
    version: str = "2.1",
    meta: dict[str, Any] | None = None,
) -> Path:
    """写 LeRobot 风格目录：meta + 轨迹 npy + 原始 mcap。"""
    ep = parse_mcap_episode(mcap_path)
    stem = mcap_path.stem
    root = mcap_path.parent / f"lerobot_v{version.replace('.', '_')}" / stem
    root.mkdir(parents=True, exist_ok=True)
    data_dir = root / "data"
    data_dir.mkdir(exist_ok=True)
    shutil.copy2(mcap_path, data_dir / mcap_path.name)

    state = observation_state(ep)
    act = action_14(ep)
    n = int(max(len(state), len(act), (meta or {}).get("steps") or 0, ep.n_steps))

    # 对齐长度
    if len(state) and len(act) and len(state) != len(act):
        n_use = min(len(state), len(act))
        state, act = state[:n_use], act[:n_use]
        n = n_use
    elif len(state):
        n = len(state)
    elif len(act):
        n = len(act)

    if len(state):
        np.save(data_dir / "observation_state.npy", state)
    if len(act):
        np.save(data_dir / "action.npy", act)
    if len(ep.ee_left):
        np.save(data_dir / "ee_left.npy", ep.ee_left)
    if len(ep.ee_right):
        np.save(data_dir / "ee_right.npy", ep.ee_right)
    for cam, frames in (ep.images or {}).items():
        if frames:
            h = min(im.shape[0] for im in frames)
            w = min(im.shape[1] for im in frames)
            stack = np.stack([im[:h, :w] for im in frames], axis=0)
            np.save(data_dir / f"images_{cam}.npy", stack)

    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32",
            "shape": [14],
            "names": None,
        },
        "action": {"dtype": "float32", "shape": [14], "names": None},
    }
    for cam, frames in (ep.images or {}).items():
        if frames:
            h, w = frames[0].shape[:2]
            features[f"observation.images.{cam}"] = {
                "dtype": "video",
                "shape": [h, w, 3],
            }

    info = {
        "codebase_version": version,
        "robot_type": (meta or {}).get("robot_type", "dual_arm"),
        "fps": (meta or {}).get("fps", 50),
        "features": features,
        "total_episodes": 1,
        "total_frames": n,
        "source_mcap": str(mcap_path.name),
        "teleop_birth_meta": {**(meta or {}), **(ep.meta or {})},
        "tensors": {
            "observation_state": bool(len(state)),
            "action": bool(len(act)),
            "images": list((ep.images or {}).keys()),
        },
    }
    (root / "meta").mkdir(exist_ok=True)
    (root / "meta" / "info.json").write_text(
        json.dumps(info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    episodes = [
        {
            "episode_index": 0,
            "tasks": [(meta or {}).get("task", "")],
            "length": n,
        }
    ]
    (root / "meta" / "episodes.jsonl").write_text(
        json.dumps(episodes[0], ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return root


def export_session_results(
    results: list[Any],
    *,
    dataset_format: str,
    extra_meta: dict[str, Any] | None = None,
) -> list[Path]:
    """对 AutoCollector EpisodeResult 列表批量导出。"""
    out_paths: list[Path] = []
    for r in results:
        path = Path(getattr(r, "path", r))
        meta = {
            "episode": getattr(r, "episode", 0),
            "success": getattr(r, "success", None),
            "steps": getattr(r, "steps", 0),
            "layout": getattr(r, "layout", None),
            **(extra_meta or {}),
        }
        exported = export_episode(path, dataset_format=dataset_format, meta=meta)
        if exported is not None:
            out_paths.append(Path(exported))
            if Path(exported) != path:
                print(f"[export] {dataset_format} → {exported}")
    return out_paths
