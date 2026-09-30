"""小车领域类型。**不 import ROS** —— 这样纯逻辑可以在不接硬件的机器上测。"""

from __future__ import annotations

from dataclasses import dataclass

# 与 l150pro_driver/protocol.py 的 FAULT_* 逐位一致。
# 刻意复制而不是 import：驱动是 ROS 包，import 它会把语音助手
# 绑死在 workspace 的环境上，而这里只需要 6 个整数。
FAULT_NAMES: dict[int, str] = {
    1 << 0: "欠压",
    1 << 1: "过压",
    1 << 2: "急停",
    1 << 3: "IMU故障",
    1 << 4: "下行超时",
    1 << 5: "编码器异常",
}


def describe_faults(fault: int | None) -> str:
    """把 fault 位掩码翻译成能念出来的中文。"""
    if fault is None:
        return "读不到故障码"
    if fault == 0:
        return "正常"
    names = [name for bit, name in FAULT_NAMES.items() if fault & bit]
    return "、".join(names) if names else f"未知({fault:#x})"


@dataclass(frozen=True)
class CarLimits:
    """限幅与默认值。全部来自 config.yaml 的 car: 段，不写死在代码里 ——
    将来标定完成或供电改善，只需要改 yaml。"""

    default_speed: float = 0.3          # m/s，car_move 未指定 speed 时用
    max_speed: float = 0.5              # m/s，与驱动 max_vx 一致
    default_turn_speed_dps: float = 60  # 度/秒，car_turn 未指定 speed 时用
    max_turn_speed_dps: float = 120     # 度/秒
    camera_step_deg: float = 15         # car_camera 未指定 degrees 时的步进
    camera_min_us: int = 800            # 云台行程下限（teleop 的保守值，防堵转扫齿）
    camera_max_us: int = 2200           # 云台行程上限
    camera_center_us: int = 1500        # 中位
    cmd_rate_hz: float = 25.0           # 发帧频率（驱动 cmd_rate_hz=20，留余量）

    @classmethod
    def from_config(cls, config) -> "CarLimits":
        get = config.get
        return cls(
            default_speed=float(get("car.default_speed", 0.3)),
            max_speed=float(get("car.max_speed", 0.5)),
            default_turn_speed_dps=float(get("car.default_turn_speed_dps", 60)),
            max_turn_speed_dps=float(get("car.max_turn_speed_dps", 120)),
            camera_step_deg=float(get("car.camera_step_deg", 15)),
            camera_min_us=int(get("car.camera_min_us", 800)),
            camera_max_us=int(get("car.camera_max_us", 2200)),
            camera_center_us=int(get("car.camera_center_us", 1500)),
            cmd_rate_hz=float(get("car.cmd_rate_hz", 25.0)),
        )


@dataclass(frozen=True)
class RobotState:
    """一帧机器人状态快照。RosBridge 与测试里的 FakeBridge 都产出它。"""

    voltage: float | None                              # V；驱动未发 battery_voltage 时为 None
    fault: int | None                                  # fault 位，见 FAULT_NAMES
    wheel_speeds: tuple[float, float, float, float] | None   # A/B/C/D，m/s
    yaw_rate: float | None                             # rad/s，IMU gz（不积分就不会漂）
    traveled: float                                    # 累计行进距离，m
    online: bool                                       # 是否收到过任何上行数据
