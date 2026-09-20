"""资产层：catalog 索引 + 本地 URDF/相机描述（不依赖 plugins）。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from teleop_birth.scene.assets.types import AssetRef, MountSpec

__all__ = ["AssetRef", "AssetRegistry", "MountSpec"]


class AssetRegistry:
    def __init__(self, project_root: Path, catalog: dict[str, Any] | None = None) -> None:
        self.root = Path(project_root)
        if catalog is None:
            cat_path = Path(__file__).resolve().parent / "catalog.yaml"
            catalog = yaml.safe_load(cat_path.read_text(encoding="utf-8")) or {}
        self.catalog = catalog
        self.sources = dict(catalog.get("sources") or {})

    def _source_root(self, source: str) -> Path:
        info = self.sources.get(source) or {}
        rel = str(info.get("root") or "")
        if source == "local":
            return Path(__file__).resolve().parent
        return (self.root / rel).resolve() if rel else self.root

    def resolve(self, kind: str, key: str) -> AssetRef:
        block = dict((self.catalog.get(kind) or {}).get(key) or {})
        if not block:
            raise KeyError(f"catalog 无此项: {kind}/{key}")
        source = str(block.get("source") or "local")
        rel_key = str(block.get("key") or "")
        base = self._source_root(source)
        path = (base / rel_key).resolve() if rel_key else None
        return AssetRef(source=source, key=key, path=path, meta=block)

    def list_keys(self, kind: str) -> list[str]:
        return sorted((self.catalog.get(kind) or {}).keys())

    def bootstrap_hint(self, source: str) -> str:
        info = self.sources.get(source) or {}
        return str(info.get("bootstrap") or "scripts/bootstrap_third_party.sh")
