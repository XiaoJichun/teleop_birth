"""推理场景：选场景 → 可视化/录像选项 → MuJoCo + ROS 通讯循环。"""

from __future__ import annotations

from pathlib import Path

from omegaconf import OmegaConf

from teleop_birth.scene.inference.runner import InferenceOptions, InferenceRunner
from teleop_birth.ui import tui
from teleop_birth.ui.flows.session_common import pick_built_scene


def action_inference_session(root: Path) -> None:
    tui.section("推理场景")
    print(
        "  通过 ROS 发布观测（相机 + 关节/末端），订阅动作驱动机器人。\n"
        "  无 ROS2 时自动使用 mock 传输（可本地联调）；配置见 configs/plugins/comms/ros_bridge.yaml"
    )

    scene_path = pick_built_scene(root)
    if scene_path is None:
        return

    visualize = tui.confirm("是否打开 MuJoCo 可视化窗口？", default=True)
    if visualize is None:
        return

    save_video = False
    if visualize:
        sv = tui.confirm("是否同时保存推理视频？", default=False)
        if sv is None:
            return
        save_video = bool(sv)
    else:
        # 无界面：默认保存视频（否则无反馈）；仍可关
        sv = tui.confirm("无可视化：是否保存推理视频？", default=True)
        if sv is None:
            return
        save_video = bool(sv)

    state_mode = tui.select(
        "发布机器人状态类型",
        [("ee", "末端位姿 PoseStamped（左右）"), ("joint", "关节角 JointState"), ("cancel", "取消")],
        allow_cancel=True,
    )
    if state_mode in (None, "cancel"):
        return
    action_mode = tui.select(
        "订阅动作指令类型",
        [("ee", "末端位姿（IK 伺服）"), ("joint", "关节角（直写 ctrl）"), ("cancel", "取消")],
        allow_cancel=True,
    )
    if action_mode in (None, "cancel"):
        return

    cam_names = _pick_cameras(root, scene_path)

    transport = tui.select(
        "数据传输",
        [
            ("ros", "ROS2 pub/sub（需已 source ROS2 + rclpy）"),
            ("mock", "Mock（进程内队列，无 ROS 可跑通场景）"),
            ("cancel", "取消"),
        ],
        allow_cancel=True,
    )
    if transport in (None, "cancel"):
        return

    dur_raw = tui.text("最长运行秒数（0=直到中断/关窗口）", default="0")
    if dur_raw is None:
        return
    try:
        duration_s = float(dur_raw.strip() or "0")
    except ValueError:
        duration_s = 0.0

    inf_cfg = _load_inference_cfg(root)
    overrides = {
        "transport": transport,
        "publish": {
            "state": {"mode": state_mode},
            "cameras": {"names": cam_names},
        },
        "subscribe": {"mode": action_mode},
    }

    tui.section("确认推理")
    print(f"  场景: {scene_path.name}")
    print(f"  可视化: {visualize}  录像: {save_video}")
    print(f"  状态发布: {state_mode}  动作订阅: {action_mode}  传输: {transport}")
    print(f"  相机: {cam_names or '（全部）'}")
    print(f"  时长: {'无限' if duration_s <= 0 else f'{duration_s:.1f}s'}")
    if not tui.confirm("开始推理？", default=True):
        return

    opts = InferenceOptions(
        visualize=bool(visualize),
        save_video=bool(save_video),
        video_camera=str((inf_cfg.get("video") or {}).get("camera") or "third_person"),
        video_fps=float((inf_cfg.get("video") or {}).get("fps") or 20),
        control_hz=float(inf_cfg.get("control_hz") or 20),
        duration_s=float(duration_s),
        comms_overrides=overrides,
    )
    runner = InferenceRunner(root, scene_path, options=opts, inference_cfg=inf_cfg)
    result = runner.run()
    print(f"[inference] 结束 steps={result.steps} transport={result.transport}")
    if result.video_path:
        print(f"  视频: {result.video_path}")


def _load_inference_cfg(root: Path) -> dict:
    path = root / "configs" / "scene" / "inference.yaml"
    if not path.is_file():
        return {}
    data = OmegaConf.to_container(OmegaConf.load(path), resolve=True)
    return dict(data) if isinstance(data, dict) else {}


def _pick_cameras(root: Path, scene_path: Path) -> list[str]:
    from teleop_birth.scene.plugins.scene import load_scene_yaml

    cfg = load_scene_yaml(scene_path)
    cams = [str(c.get("name")) for c in (cfg.get("cameras") or []) if isinstance(c, dict) and c.get("name")]
    if not cams:
        return []
    if hasattr(tui, "checkbox"):
        picked = tui.checkbox(
            "选择要发布的相机（空=全部）",
            [(n, n) for n in cams],
            defaults=cams,
        )
        if picked is None:
            return cams
        return list(picked)
    print("  相机:", ", ".join(cams))
    raw = tui.text("发布相机名（逗号分隔，空=全部）", default=",".join(cams))
    if raw is None or not raw.strip():
        return cams
    return [x.strip() for x in raw.replace("，", ",").split(",") if x.strip()]
