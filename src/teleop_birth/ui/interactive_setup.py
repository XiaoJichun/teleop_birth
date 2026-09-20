"""交互式启动：设备参数与算法参数分组选择。"""

from __future__ import annotations

from pathlib import Path

from teleop_birth.ui import tui
from teleop_birth.ui.util import hydra_default_choice, list_config_group, load_yaml


def _pick_default(options: list[tuple[str, str]], preferred: str | None) -> str | None:
    if not options:
        return None
    if preferred and any(value == preferred for value, _ in options):
        return preferred
    return options[0][0]


def _labeled_options(group_dir: Path, labels: dict[str, str]) -> list[tuple[str, str]]:
    options: list[tuple[str, str]] = []
    for value, _ in list_config_group(group_dir):
        label = labels.get(value) or value
        options.append((value, f"{label}  [{value}]"))
    return options


def build_interactive_overrides(project_root: Path) -> list[str] | None:
    config_root = project_root / "configs"
    runtime = load_yaml(config_root / "algorithm" / "runtime" / "default.yaml")
    labels_all = load_yaml(config_root / "ui" / "labels.yaml")

    tui.banner("遥操交互式配置")

    # —— 换设备时常改 ——
    tui.section("设备参数（换仿真/真机/头显时改这里）")

    source = tui.select(
        "【设备】XR 数据源",
        _labeled_options(config_root / "device" / "source", labels_all.get("source") or {}),
        default=_pick_default(
            list_config_group(config_root / "device" / "source"),
            hydra_default_choice(config_root, "source"),
        ),
    )
    if source is None:
        return None

    backend = tui.select(
        "【设备】机器人后端（sim / real）",
        _labeled_options(config_root / "device" / "backend", labels_all.get("backend") or {}),
        default=_pick_default(
            list_config_group(config_root / "device" / "backend"),
            hydra_default_choice(config_root, "backend"),
        ),
    )
    if backend is None:
        return None

    ee = tui.select(
        "【设备】末端执行器",
        _labeled_options(config_root / "device" / "end_effector", labels_all.get("end_effector") or {}),
        default=_pick_default(
            list_config_group(config_root / "device" / "end_effector"),
            hydra_default_choice(config_root, "end_effector"),
        ),
    )
    if ee is None:
        return None

    # —— 算法常驻（一般不换） ——
    tui.section("算法参数（常驻，换设备通常不用改）")

    ik = tui.select(
        "【算法】IK 求解器",
        _labeled_options(config_root / "algorithm" / "ik", labels_all.get("ik") or {}),
        default=_pick_default(
            list_config_group(config_root / "algorithm" / "ik"),
            hydra_default_choice(config_root, "ik"),
        ),
    )
    if ik is None:
        return None

    mapper = tui.select(
        "【算法】映射器",
        _labeled_options(config_root / "algorithm" / "mapper", labels_all.get("mapper") or {}),
        default=_pick_default(
            list_config_group(config_root / "algorithm" / "mapper"),
            hydra_default_choice(config_root, "mapper"),
        ),
    )
    if mapper is None:
        return None

    # 臂坐标剖面（仿真/真机共用 R_a）
    arm_profile_override: str | None = None
    try:
        from teleop_birth.teleop.frame.profiles import list_arm_profiles

        profiles = list_arm_profiles()
        profile_opts: list[tuple[str, str]] = [
            ("", "默认（用 mapper YAML 的 xr_align）"),
        ]
        profile_opts.extend((p.id, f"{p.id} — {p.title}") for p in profiles)
        picked = tui.select(
            "【算法】臂坐标剖面 arm_profile（换臂/换轴向）",
            profile_opts,
            default="",
            allow_cancel=True,
        )
        if picked is None:
            return None
        if picked:
            arm_profile_override = str(picked)
    except Exception:  # noqa: BLE001
        pass

    hz = tui.text(
        "【算法】控制频率 hz",
        default=str(runtime.get("hz", 50.0)),
        validate=lambda s: True if _is_pos_float(s) else "请输入正数",
    )
    if hz is None:
        return None

    duration = tui.text(
        "运行时长 duration_s（<=0 直到 Ctrl+C）",
        default=str(runtime.get("duration_s", 8.0)),
        validate=lambda s: True if _is_float(s) else "请输入数字",
    )
    if duration is None:
        return None

    scale = tui.text(
        "【算法】XR 位移缩放 scale",
        default=str(runtime.get("scale", 1.0)),
        validate=lambda s: True if _is_pos_float(s) else "请输入正数",
    )
    if scale is None:
        return None

    render = tui.confirm(
        "开启 MuJoCo 可视化窗口？",
        default=bool(runtime.get("render", False)),
    )
    if render is None:
        return None

    log_enabled = tui.confirm(
        "记录 JSONL 会话日志？",
        default=bool(runtime.get("log_enabled", False)),
    )
    if log_enabled is None:
        return None

    overrides = [
        f"source={source}",
        f"backend={backend}",
        f"end_effector={ee}",
        f"ik={ik}",
        f"mapper={mapper}",
        f"runtime.hz={hz.strip()}",
        f"runtime.duration_s={duration.strip()}",
        f"runtime.scale={scale.strip()}",
        f"runtime.render={str(render).lower()}",
        f"runtime.log_enabled={str(log_enabled).lower()}",
        "runtime.interactive_confirm=true",
    ]
    if arm_profile_override:
        overrides.append(f"mapper.arm_profile={arm_profile_override}")

    tui.section("配置摘要")
    print("  —— 设备 ——")
    for item in overrides[:3]:
        print(f"  {item}")
    print("  —— 算法 ——")
    for item in overrides[3:]:
        print(f"  {item}")
    ok = tui.confirm("确认并启动遥操？", default=True)
    if not ok:
        return None
    return overrides


def _is_float(raw: str) -> bool:
    try:
        float((raw or "").strip())
        return True
    except ValueError:
        return False


def _is_pos_float(raw: str) -> bool:
    try:
        return float((raw or "").strip()) > 0
    except ValueError:
        return False
