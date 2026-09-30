"""工具注册测试 —— 只查 schema 形状与接线，不碰 ROS。"""

import pytest

from tools.car import register_car_tools
from tools.registry import Tool, ToolRegistry


class _StubController:
    def __init__(self):
        self.calls = []

    def move_distance(self, direction, distance, speed=None):
        self.calls.append(("move", direction, distance, speed))
        return "已前进约 2.00 米。"

    def turn_by(self, angle_deg, speed=None):
        self.calls.append(("turn", angle_deg, speed))
        return "已左转约 90 度。"

    def stop_and_describe(self):
        self.calls.append(("stop",))
        return "好，停了。"

    def camera_move(self, direction, degrees=None):
        self.calls.append(("camera", direction, degrees))
        return "云台往左了（水平轴 1333 微秒）。"

    def status_text(self):
        self.calls.append(("status",))
        return "电池 11.6 伏，故障码 正常，停着。"


EXPECTED_TOOLS = {"car_move", "car_turn", "car_stop", "car_camera", "car_status"}


def _registry():
    registry = ToolRegistry()
    register_car_tools(registry, _StubController())
    return registry


def test_registers_exactly_five_tools():
    assert set(_registry()._tools) == EXPECTED_TOOLS


def test_all_tools_are_in_the_car_group():
    for tool in _registry()._tools.values():
        assert tool.group == "car"


def test_schemas_are_openai_shaped():
    for schema in _registry().schemas():
        assert schema["type"] == "function"
        assert schema["function"]["name"]
        assert "description" in schema["function"]
        assert schema["function"]["parameters"]["type"] == "object"


def test_schemas_enum_is_not_a_hand_copy():
    """schema 的 enum 必须**就是** kinematics 的表，不能是抄的一份。

    抄一份会静默漂移：schema 发给模型，漂了就变成「模型传一个控制器不认的方向」，
    而且没有任何东西会报错。
    """
    from car.kinematics import camera_direction_names, direction_names

    schemas = {s["function"]["name"]: s for s in _registry().schemas()}
    move_enum = schemas["car_move"]["function"]["parameters"]["properties"]["direction"]["enum"]
    camera_enum = schemas["car_camera"]["function"]["parameters"]["properties"]["direction"]["enum"]
    assert move_enum == direction_names()
    assert camera_enum == camera_direction_names()


def test_car_move_direction_enum_has_six_values():
    schema = {s["function"]["name"]: s for s in _registry().schemas()}["car_move"]
    enum = schema["function"]["parameters"]["properties"]["direction"]["enum"]
    assert len(enum) == 6
    assert "left" not in enum          # 纯转向归 car_turn
    assert "forward_left" in enum


def test_car_move_distance_is_required():
    schema = {s["function"]["name"]: s for s in _registry().schemas()}["car_move"]
    assert "distance" in schema["function"]["parameters"]["required"]


def test_optional_params_are_not_required():
    schemas = {s["function"]["name"]: s for s in _registry().schemas()}
    assert "speed" not in schemas["car_move"]["function"]["parameters"]["required"]
    assert "speed" not in schemas["car_turn"]["function"]["parameters"]["required"]
    assert "degrees" not in schemas["car_camera"]["function"]["parameters"]["required"]


def test_car_stop_takes_no_parameters():
    schema = {s["function"]["name"]: s for s in _registry().schemas()}["car_stop"]
    assert schema["function"]["parameters"]["properties"] == {}


# ---------------- 分发 ----------------

def test_dispatch_calls_the_controller():
    controller = _StubController()
    registry = ToolRegistry()
    register_car_tools(registry, controller)
    outcome = registry.dispatch("car_move", {"direction": "forward", "distance": 2.0})
    assert controller.calls == [("move", "forward", 2.0, None)]
    assert "2.00" in outcome.result


def test_dispatch_passes_optional_speed():
    controller = _StubController()
    registry = ToolRegistry()
    register_car_tools(registry, controller)
    registry.dispatch("car_move", {"direction": "forward", "distance": 2.0,
                                   "speed": 0.4})
    assert controller.calls == [("move", "forward", 2.0, 0.4)]


def test_dispatch_turn_passes_angle():
    controller = _StubController()
    registry = ToolRegistry()
    register_car_tools(registry, controller)
    registry.dispatch("car_turn", {"angle_deg": -90})
    assert controller.calls == [("turn", -90, None)]


# ---------------- 控制器不可用时 ----------------

def test_registers_even_when_controller_is_none():
    """小车起不来也要注册 —— 让模型调了以后得到一句解释，而不是「没有这个工具」。"""
    registry = ToolRegistry()
    register_car_tools(registry, None)
    assert set(registry._tools) == EXPECTED_TOOLS


def test_tools_say_so_when_controller_is_none():
    registry = ToolRegistry()
    register_car_tools(registry, None)
    outcome = registry.dispatch("car_move", {"direction": "forward", "distance": 1.0})
    assert "不可用" in outcome.result


# ---------------- 分组过滤（为将来工具过 20 个预留） ----------------

def test_schemas_can_filter_by_group():
    registry = ToolRegistry()
    registry.register(Tool(name="get_current_time", description="d",
                           parameters={"type": "object", "properties": {}},
                           handler=lambda: "x"))
    register_car_tools(registry, _StubController())
    names = {s["function"]["name"] for s in registry.schemas(groups=["car"])}
    assert names == EXPECTED_TOOLS


def test_schemas_without_groups_returns_everything():
    registry = ToolRegistry()
    register_car_tools(registry, _StubController())
    assert len(registry.schemas()) == 5


def test_duplicate_registration_still_raises():
    registry = _registry()
    with pytest.raises(ValueError):
        register_car_tools(registry, _StubController())
