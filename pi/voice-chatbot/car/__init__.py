"""小车控制。见 docs/plans/2026-09-17-car-tools-design.md。

模块边界：
  types.py       —— 纯数据类型与配置解析，无依赖
  kinematics.py  —— 纯函数（方向合成、限幅、云台换算），无依赖
  ros_bridge.py  —— **唯一** import rclpy 的地方
  controller.py  —— 编排：闭环运动、急停、状态播报
"""
