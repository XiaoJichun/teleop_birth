"""将 catalog 夹爪 id 映射到 UR MuJoCo scene_cfg 字段。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from teleop_birth.scene.assets import AssetRegistry

# 已知 MJCF 夹爪的执行器 / TCP 命名（相对 gripper_prefix）
_KNOWN_SUFFIXES: dict[str, dict[str, str]] = {
    "robotiq_2f85": {
        "gripper_actuator_suffix": "g_fingers_actuator",
        "tcp_site_suffix": "g_pinch",
        "gripper_prefix": "g_",
        "gripper_attach_site": "attachment_site",
    },
}


def apply_gripper_to_scene_cfg(
    project_root: Path,
    scene_cfg: dict[str, Any],
    gripper_id: str | None,
    *,
    menagerie: Path | None = None,
) -> dict[str, Any]:
    """就地写入 gripper_xml / attach_site；不可用则保留原配置并返回提示键。"""
    out = scene_cfg
    gid = (gripper_id or "").strip()
    if not gid:
        return out

    reg = AssetRegistry(project_root)
    try:
        ref = reg.resolve("grippers", gid)
    except KeyError:
        out["_gripper_warn"] = f"未知夹爪 id: {gid}，沿用 scene 默认夹爪"
        return out

    if ref.meta.get("builtin"):
        out["_gripper_warn"] = (
            f"夹爪「{ref.meta.get('title') or gid}」标记为 builtin（内置于臂），"
            f"UR 双臂预览仍使用 scene 默认 MJCF 夹爪"
        )
        return out

    path = ref.path
    if path is None or not path.is_file():
        out["_gripper_warn"] = f"夹爪文件不存在: {path}"
        return out

    suffix = path.suffix.lower()
    if suffix not in (".xml", ".mjcf"):
        out["_gripper_warn"] = (
            f"夹爪「{gid}」为 {suffix or '未知'}，MuJoCo attach 需要 MJCF(.xml)；"
            f"请改用 menagerie MJCF 或保持 robotiq_2f85"
        )
        return out

    # menagerie 相对路径优先；否则写绝对路径（builder 已支持）
    menagerie = Path(menagerie) if menagerie else project_root / "third_party" / "mujoco_menagerie"
    try:
        rel = path.resolve().relative_to(menagerie.resolve())
        out["gripper_xml"] = str(rel).replace("\\", "/")
    except ValueError:
        out["gripper_xml"] = str(path.resolve())

    attach = str(ref.meta.get("attach_site") or "attachment_site")
    out["gripper_attach_site"] = attach

    known = _KNOWN_SUFFIXES.get(gid) or {}
    for k, v in known.items():
        out[k] = v
    # catalog 可覆盖
    if ref.meta.get("gripper_actuator_suffix"):
        out["gripper_actuator_suffix"] = str(ref.meta["gripper_actuator_suffix"])
    if ref.meta.get("tcp_site_suffix"):
        out["tcp_site_suffix"] = str(ref.meta["tcp_site_suffix"])
    if ref.meta.get("gripper_prefix"):
        out["gripper_prefix"] = str(ref.meta["gripper_prefix"])

    out["_gripper_applied"] = gid
    return out


def register_gripper_in_catalog(
    project_root: Path,
    *,
    gripper_id: str,
    source: str,
    key: str,
    title: str,
    attach_site: str = "attachment_site",
    builtin: bool = False,
    actuator_suffix: str | None = None,
    tcp_site_suffix: str | None = None,
) -> Path:
    """向 catalog.yaml 追加 grippers 项；返回 catalog 路径。"""
    import yaml

    cat_path = Path(project_root) / "src" / "teleop_birth" / "scene" / "assets" / "catalog.yaml"
    # 包内路径更稳妥
    pkg_cat = Path(__file__).resolve().parents[2] / "scene" / "assets" / "catalog.yaml"
    if pkg_cat.is_file():
        cat_path = pkg_cat

    data = yaml.safe_load(cat_path.read_text(encoding="utf-8")) or {}
    grippers = dict(data.get("grippers") or {})
    gid = "".join(c if c.isalnum() or c in "-_" else "_" for c in gripper_id.strip())
    if not gid:
        raise ValueError("夹爪 id 为空")
    if gid in grippers:
        raise ValueError(f"夹爪 id 已存在: {gid}")

    entry: dict[str, Any] = {
        "source": source,
        "key": key,
        "title": title or gid,
        "attach_site": attach_site or "attachment_site",
    }
    if builtin:
        entry["builtin"] = True
    if actuator_suffix:
        entry["gripper_actuator_suffix"] = actuator_suffix
    if tcp_site_suffix:
        entry["tcp_site_suffix"] = tcp_site_suffix

    grippers[gid] = entry
    data["grippers"] = grippers
    cat_path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return cat_path
