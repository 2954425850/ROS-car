"""小车控制工具。

## 为什么是原生工具而不是 MCP

`cmd_vel` 要 25Hz 持续发（底盘有 200ms 丢帧看门狗），而 MCP server 天生是
请求-响应；急停更不该经过一次 CallTool 往返。小车是**本机常驻硬件能力**，
不是远程服务 —— 与 tools/music.py 里「服务器给链接、宿主负责放」同一个道理。

## 描述怎么写

「何时该调用」全部由 description 承担，**不要往 system_prompt 里加针对小车
的提示行** —— 在 persona 里点名一批能力，会给那批 token 加权，小模型于是主动
推销（2026-09-13 踩过两次，见记忆 prompt-no-capability-emphasis）。
"""

from __future__ import annotations

from loguru import logger

from car.kinematics import camera_direction_names, direction_names
from tools.registry import Tool, ToolRegistry

_UNAVAILABLE = (
    "小车当前不可用 —— ROS 2 环境没准备好，或者底盘驱动没在跑。"
    "请检查服务日志。"
)


def _guard(controller):
    """controller 为 None 时统一回一句解释，而不是让模型看到「没有这个工具」。"""

    def _wrap(method_name: str):
        def _handler(**kwargs):
            if controller is None:
                return _UNAVAILABLE
            return getattr(controller, method_name)(**kwargs)

        return _handler

    return _wrap


def register_car_tools(registry: ToolRegistry, controller) -> None:
    """把小车能力登记成原生工具。controller 可以是 None（ROS 起不来时）。"""
    if controller is None:
        logger.warning("小车：控制器不可用，仍注册工具（调用时会说明原因）")

    guarded = _guard(controller)

    registry.register(
        Tool(
            name="car_move",
            group="car",
            description=(
                "让小车朝一个方向走一段距离，走完自动停。"
                "用户说「往前走」「后退」「往左前方走」「退两米」这类"
                "**带方向、也带一段路**的话时调用。"
                "原地转向请用 car_turn，本工具不接受纯左右。"
                "distance 单位是米；用户没提速度就不要传 speed，"
                "只在说了「快点」「慢点」「用 0.3 米每秒」时才传。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": direction_names(),
                        "description": "行进方向，六选一。",
                    },
                    "distance": {
                        "type": "number",
                        "description": "走多远，单位米。用户说「一点」时按 0.5 估。",
                    },
                    "speed": {
                        "type": "number",
                        "description": "线速度，单位米每秒。留空用默认值。",
                    },
                },
                "required": ["direction", "distance"],
            },
            handler=guarded("move_distance"),
        )
    )

    registry.register(
        Tool(
            name="car_turn",
            group="car",
            description=(
                "让小车原地转向，转完自动停。"
                "用户说「左转」「右转」「转九十度」「掉个头」时调用。"
                "angle_deg 左转为正、右转为负（如左转 90 度传 90，右转传 -90）。"
                "用户没提转速就不要传 speed。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "angle_deg": {
                        "type": "number",
                        "description": "要转的角度，左正右负。掉头按 180 算。",
                    },
                    "speed": {
                        "type": "number",
                        "description": "转向角速度，单位度每秒。留空用默认值。",
                    },
                },
                "required": ["angle_deg"],
            },
            handler=guarded("turn_by"),
        )
    )

    registry.register(
        Tool(
            name="car_stop",
            group="car",
            description=(
                "让小车立刻停下。用户说「停」「别走了」「停下吧」时调用。"
                "底盘正在走或正在转、用户想中止时就用它。"
            ),
            parameters={"type": "object", "properties": {}, "required": []},
            handler=guarded("stop_and_describe"),
        )
    )

    registry.register(
        Tool(
            name="car_camera",
            group="car",
            description=(
                "转动小车上的云台摄像头。"
                "用户说「往左看看」「往右看看」「抬头」「低头」「云台回正」时调用。"
                "用户没给具体角度就不要传 degrees（会转一个固定小步）；"
                "只有明确说了「转 30 度」这类角度时才传。"
            ),
            parameters={
                "type": "object",
                "properties": {
                    "direction": {
                        "type": "string",
                        "enum": camera_direction_names(),
                        "description": (
                            "left=往左转，right=往右转，up=抬头，"
                            "down=低头，center=回中位。"
                        ),
                    },
                    "degrees": {
                        "type": "number",
                        "description": "转动角度。留空 = 固定小步。",
                    },
                },
                "required": ["direction"],
            },
            handler=guarded("camera_move"),
        )
    )

    registry.register(
        Tool(
            name="car_status",
            group="car",
            description=(
                "查询小车的状态：电池电压、故障码、四轮速度、在不在动。"
                "用户问「还有多少电」「车怎么了」「车什么情况」时调用。"
            ),
            parameters={"type": "object", "properties": {}, "required": []},
            handler=guarded("status_text"),
        )
    )
