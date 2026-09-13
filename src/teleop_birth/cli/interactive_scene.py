"""场景交互配置：从 configs/scene/cli.yaml 读默认，终端提问覆盖。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from teleop_birth.cli import tui
from teleop_birth.cli.util import list_config_group, load_yaml
from teleop_birth.scene_ops.props.catalog import PropCatalog


def load_scene_cli(project_root: Path) -> dict[str, Any]:
    return load_yaml(project_root / "configs" / "scene" / "cli.yaml")


def _labels(project_root: Path) -> dict[str, Any]:
    return load_yaml(project_root / "configs" / "ui" / "labels.yaml")


def _prop_options(project_root: Path) -> list[tuple[str, str]]:
    catalog = PropCatalog(project_root / "configs" / "scene" / "props")
    labels = (_labels(project_root).get("props") or {})
    opts: list[tuple[str, str]] = []
    for name in catalog.list_names():
        label = labels.get(name) or name
        opts.append((name, f"{label}  [{name}]"))
    return opts


def _task_options(project_root: Path) -> list[tuple[str, str]]:
    labels = (_labels(project_root).get("tasks") or {})
    opts: list[tuple[str, str]] = []
    for value, _ in list_config_group(project_root / "configs" / "scene" / "tasks"):
        label = labels.get(value) or value
        opts.append((value, f"{label}  [{value}]"))
    return opts


def _is_int(raw: str) -> bool:
    try:
        int((raw or "").strip())
        return True
    except ValueError:
        return False


def _is_nonneg_int(raw: str) -> bool:
    try:
        return int((raw or "").strip()) >= 0
    except ValueError:
        return False


@dataclass
class GenerateConfig:
    props: list[str] = field(default_factory=lambda: ["red_cube", "blue_plate"])
    render: bool = False
    settle_steps: int = 100

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "GenerateConfig":
        return cls(
            props=list(data.get("props") or ["red_cube", "blue_plate"]),
            render=bool(data.get("render", False)),
            settle_steps=int(data.get("settle_steps", 100)),
        )


@dataclass
class PlaceConfig:
    props: list[str] = field(default_factory=lambda: ["red_cube", "blue_plate"])
    seed: int = 0
    repeats: int = 3
    render: bool = False

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PlaceConfig":
        return cls(
            props=list(data.get("props") or ["red_cube", "blue_plate"]),
            seed=int(data.get("seed", 0)),
            repeats=int(data.get("repeats", 3)),
            render=bool(data.get("render", False)),
        )


@dataclass
class CollectConfig:
    task: str = "right_pick_red_cube_to_blue_plate"
    episodes: int = 3
    seed: int = 0
    arm: str = "right"
    render: bool = True
    pace_s: float = 0.02
    output_dir: str = "logs/mcap"
    steps_per_segment: int = 35
    hold_steps: int = 0
    sticky_grasp: bool = True
    camera_enabled: bool = True
    # 分辨率 / 采样间隔以 configs/scene/recording.yaml 为准；此处仅兼容旧字段
    camera_width: int = 640
    camera_height: int = 480
    camera_every_n_steps: int = 2
    recording: str = "scene/recording.yaml"
    backend: str = "device/backend/mujoco_dual_ur.yaml"
    ik: str = "algorithm/ik/dls.yaml"
    end_effector: str = "device/end_effector/gripper.yaml"
    paths: str = "device/paths/default.yaml"

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CollectConfig":
        cam = data.get("camera") if isinstance(data.get("camera"), dict) else {}
        return cls(
            task=str(data.get("task") or "right_pick_red_cube_to_blue_plate"),
            episodes=int(data.get("episodes", 3)),
            seed=int(data.get("seed", 0)),
            arm=str(data.get("arm") or "right"),
            render=bool(data.get("render", True)),
            pace_s=float(data.get("pace_s", 0.02)),
            output_dir=str(data.get("output_dir") or "logs/mcap"),
            steps_per_segment=int(data.get("steps_per_segment", 35)),
            hold_steps=int(data.get("hold_steps", 0)),
            sticky_grasp=bool(data.get("sticky_grasp", True)),
            camera_enabled=bool(cam.get("enabled", data.get("camera_enabled", True))),
            camera_width=int(cam.get("width", data.get("camera_width", 640))),
            camera_height=int(cam.get("height", data.get("camera_height", 480))),
            camera_every_n_steps=int(cam.get("every_n_steps", data.get("camera_every_n_steps", 2))),
            recording=str(data.get("recording") or "scene/recording.yaml"),
            backend=str(data.get("backend") or "device/backend/mujoco_dual_ur.yaml"),
            ik=str(data.get("ik") or "algorithm/ik/dls.yaml"),
            end_effector=str(data.get("end_effector") or "device/end_effector/gripper.yaml"),
            paths=str(data.get("paths") or "device/paths/default.yaml"),
        )


def build_generate_config(project_root: Path) -> GenerateConfig | None:
    cli = load_scene_cli(project_root)
    base = GenerateConfig.from_dict(dict(cli.get("generate") or {}))
    tui.banner("场景：生成物品")
    tui.section("默认来自 configs/scene/cli.yaml → generate")

    props = tui.checkbox(
        "选择要生成的物品",
        _prop_options(project_root),
        defaults=base.props,
    )
    if props is None:
        return None
    if not props:
        print("未选择任何物品。")
        return None

    settle = tui.text(
        "settle_steps（物理沉降步数）",
        default=str(base.settle_steps),
        validate=lambda s: True if _is_nonneg_int(s) else "请输入非负整数",
    )
    if settle is None:
        return None

    render = tui.confirm("打开 MuJoCo 可视化？", default=base.render)
    if render is None:
        return None

    cfg = GenerateConfig(props=props, render=bool(render), settle_steps=int(settle.strip()))
    tui.section("配置摘要")
    print(f"  props: {cfg.props}")
    print(f"  settle_steps: {cfg.settle_steps}")
    print(f"  render: {cfg.render}")
    ok = tui.confirm("确认并生成？", default=True)
    return cfg if ok else None


def build_place_config(project_root: Path) -> PlaceConfig | None:
    cli = load_scene_cli(project_root)
    base = PlaceConfig.from_dict(dict(cli.get("place") or {}))
    tui.banner("场景：摆放物品")
    tui.section("默认来自 configs/scene/cli.yaml → place")

    props = tui.checkbox(
        "选择要摆放的物品",
        _prop_options(project_root),
        defaults=base.props,
    )
    if props is None:
        return None
    if not props:
        print("未选择任何物品。")
        return None

    seed = tui.text(
        "随机种子 seed",
        default=str(base.seed),
        validate=lambda s: True if _is_int(s) else "请输入整数",
    )
    if seed is None:
        return None
    repeats = tui.text(
        "随机布局次数 repeats",
        default=str(base.repeats),
        validate=lambda s: True if _is_nonneg_int(s) and int(s.strip()) >= 1 else "请输入正整数",
    )
    if repeats is None:
        return None
    render = tui.confirm("结束后打开 MuJoCo 可视化？", default=base.render)
    if render is None:
        return None

    cfg = PlaceConfig(
        props=props,
        seed=int(seed.strip()),
        repeats=int(repeats.strip()),
        render=bool(render),
    )
    tui.section("配置摘要")
    print(f"  props: {cfg.props}")
    print(f"  seed: {cfg.seed}  repeats: {cfg.repeats}  render: {cfg.render}")
    ok = tui.confirm("确认并摆放？", default=True)
    return cfg if ok else None


def build_collect_config(project_root: Path) -> CollectConfig | None:
    cli = load_scene_cli(project_root)
    base = CollectConfig.from_dict(dict(cli.get("collect") or {}))
    tui.banner("场景：语义自动化采集（MCAP）")
    tui.section("默认来自 configs/scene/cli.yaml → collect")

    tasks = _task_options(project_root)
    if not tasks:
        print("未找到 configs/scene/tasks/*.yaml")
        return None
    task = tui.select(
        "选择语义任务",
        tasks,
        default=base.task if any(v == base.task for v, _ in tasks) else tasks[0][0],
    )
    if task is None:
        return None

    arm = tui.select(
        "操作臂",
        [("right", "右臂"), ("left", "左臂")],
        default=base.arm if base.arm in ("left", "right") else "right",
    )
    if arm is None:
        return None

    episodes = tui.text(
        "采集 episode 数",
        default=str(base.episodes),
        validate=lambda s: True if _is_nonneg_int(s) and int(s.strip()) >= 1 else "请输入正整数",
    )
    if episodes is None:
        return None
    seed = tui.text(
        "随机种子 seed（每集物品位置变化）",
        default=str(base.seed),
        validate=lambda s: True if _is_int(s) else "请输入整数",
    )
    if seed is None:
        return None
    steps = tui.text(
        "每段 IK 插值步数 steps_per_segment",
        default=str(base.steps_per_segment),
        validate=lambda s: True if _is_nonneg_int(s) and int(s.strip()) >= 2 else "请输入 ≥2 的整数",
    )
    if steps is None:
        return None
    out = tui.text("MCAP 输出目录（相对项目根）", default=str(base.output_dir))
    if out is None:
        return None
    sticky = tui.confirm("启用粘性抓取（合成数据更稳）？", default=base.sticky_grasp)
    if sticky is None:
        return None
    render = tui.confirm("打开 MuJoCo 可视化窗口（方便盯着看）？", default=base.render)
    if render is None:
        return None
    pace = tui.text(
        "观摩节拍 pace_s（秒/步，0=尽快）",
        default=str(base.pace_s),
        validate=lambda s: True if _is_nonneg_float(s) else "请输入非负数",
    )
    if pace is None:
        return None

    cfg = CollectConfig(
        task=str(task),
        episodes=int(episodes.strip()),
        seed=int(seed.strip()),
        arm=str(arm),
        render=bool(render),
        pace_s=float(pace.strip()),
        output_dir=out.strip(),
        steps_per_segment=int(steps.strip()),
        hold_steps=base.hold_steps,
        sticky_grasp=bool(sticky),
        camera_enabled=base.camera_enabled,
        camera_width=base.camera_width,
        camera_height=base.camera_height,
        camera_every_n_steps=base.camera_every_n_steps,
        recording=base.recording,
        backend=base.backend,
        ik=base.ik,
        end_effector=base.end_effector,
        paths=base.paths,
    )
    tui.section("配置摘要")
    print(f"  task: {cfg.task}")
    print(f"  arm: {cfg.arm}  episodes: {cfg.episodes}  seed: {cfg.seed}")
    print(f"  render: {cfg.render}  pace_s: {cfg.pace_s}  hold_steps: {cfg.hold_steps}")
    print(f"  output_dir: {cfg.output_dir}")
    print(f"  recording: {cfg.recording}")
    print(f"  sticky_grasp: {cfg.sticky_grasp}")
    print(f"  backend: {cfg.backend}")
    print(f"  ik: {cfg.ik}")
    ok = tui.confirm("确认并开始采集？", default=True)
    return cfg if ok else None


def _is_nonneg_float(raw: str) -> bool:
    try:
        return float((raw or "").strip()) >= 0.0
    except ValueError:
        return False
