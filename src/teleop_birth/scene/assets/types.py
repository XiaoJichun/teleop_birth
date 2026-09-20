"""资产类型：与插件解耦，供 assets / plugins 共用。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional


@dataclass
class AssetRef:
    """开源库或本地资产引用。"""

    source: str  # menagerie | tacto | local
    key: str
    path: Optional[Path] = None
    meta: dict[str, Any] = field(default_factory=dict)


@dataclass
class MountSpec:
    """挂载点：父 body/site + 相对位姿。"""

    parent: str  # body 或 site 名
    parent_kind: str = "site"  # site | body
    pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quat_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    prefix: str = ""
    extra: dict[str, Any] = field(default_factory=dict)
