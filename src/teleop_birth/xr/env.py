"""Ensure LD_LIBRARY_PATH for local XRobo SDK before import."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml


def apply_xrobo_library_path(project_root: Path, setup_config: str | Path = "configs/setup/xrobo.yaml") -> None:
    path = Path(setup_config)
    if not path.is_absolute():
        path = project_root / path
    if not path.is_file():
        return
    cfg: dict[str, Any] = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    pc = cfg.get("pc_service") or {}
    py = cfg.get("pybind") or {}
    extract = project_root / str(pc.get("extract_dir") or "third_party/xrobo_runtime")
    svc = extract / str(pc.get("service_rel_dir") or "opt/apps/roboticsservice")
    venv_lib = project_root / ".venv" / str(py.get("venv_lib_subdir") or "lib/xrobo")
    parts = [
        str(venv_lib),
        str(svc),
        str(svc / "lib"),
        str(svc / "SDK" / "x64"),
    ]
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    merged = ":".join([*parts, existing] if existing else parts)
    os.environ["LD_LIBRARY_PATH"] = merged
    os.environ.setdefault("LC_ALL", "C.UTF-8")
    os.environ.setdefault("LANG", "C.UTF-8")
