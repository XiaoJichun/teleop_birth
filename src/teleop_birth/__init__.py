"""Progressive Extremity Teleop (PET) — dual-arm first, humanoid-ready.

顶层只分四类（依赖自上而下）:
  ui/      菜单与场景工坊
  scene/   物品、采集、插件、资产
  robot/   机型 registry + RobotBackend + 末端/IO
  teleop/  协议、主循环、映射、XR、IK
"""

__version__ = "0.1.0"
