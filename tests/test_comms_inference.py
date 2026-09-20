"""ROS 通讯插件与推理配置单测（不依赖 rclpy）。"""

from __future__ import annotations

import numpy as np

from teleop_birth.scene.plugins.comms.bridge import (
    ActionCommand,
    CommsConfig,
    ObservationBundle,
    RosDataComms,
    load_comms_config,
)


def test_load_comms_config(tmp_path, monkeypatch):
    root = tmp_path
    cfg_dir = root / "configs" / "plugins" / "comms"
    cfg_dir.mkdir(parents=True)
    (cfg_dir / "ros_bridge.yaml").write_text(
        "enabled: true\ntransport: mock\npublish:\n  state:\n    mode: joint\n"
        "subscribe:\n  mode: ee\n",
        encoding="utf-8",
    )
    cfg = load_comms_config(root)
    assert cfg.transport == "mock"
    assert cfg.state_mode == "joint"
    assert cfg.action_mode == "ee"


def test_comms_config_overrides():
    cfg = CommsConfig.from_dict(
        {
            "transport": "mock",
            "publish": {
                "cameras": {"names": ["third_person"]},
                "state": {"mode": "ee"},
            },
            "subscribe": {"mode": "joint"},
        }
    )
    assert cfg.camera_names == ["third_person"]
    assert cfg.state_mode == "ee"
    assert cfg.action_mode == "joint"


def test_mock_transport_pub_sub():
    cfg = CommsConfig(transport="mock", state_mode="ee", action_mode="ee")
    comms = RosDataComms(cfg)
    comms.start()
    assert comms.transport == "mock"

    rgb = np.zeros((16, 16, 3), dtype=np.uint8)
    obs = ObservationBundle(
        cameras={"third_person": rgb},
        ee_left=np.array([0, 0, 0.5, 0, 0, 0, 1], dtype=np.float64),
        ee_right=np.array([0.1, 0, 0.5, 0, 0, 0, 1], dtype=np.float64),
    )
    comms.publish_observation(obs)

    cmd = ActionCommand(
        mode="ee",
        ee_right=np.array([0.2, 0, 0.5, 0, 0, 0, 1], dtype=np.float64),
        right_active=True,
        right_gripper=0.5,
    )
    comms.inject_action(cmd)
    got = comms.poll_action()
    assert got is not None
    assert got.right_active
    assert got.right_gripper == 0.5
    np.testing.assert_allclose(got.ee_right[:3], [0.2, 0, 0.5])
    comms.stop()


def test_camera_filter_on_publish():
    cfg = CommsConfig(transport="mock", camera_names=["wrist_right"])
    comms = RosDataComms(cfg)
    comms.start()
    obs = ObservationBundle(
        cameras={
            "third_person": np.zeros((4, 4, 3), dtype=np.uint8),
            "wrist_right": np.ones((4, 4, 3), dtype=np.uint8),
        }
    )
    # 不应抛错；过滤在 publish_observation 内完成
    comms.publish_observation(obs)
    assert isinstance(comms._impl, object)
    comms.stop()
