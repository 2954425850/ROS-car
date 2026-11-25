import os

import pytest

yaml = pytest.importorskip("yaml")

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load(name):
    with open(os.path.join(HERE, "config", name), encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_nav2_fusion_global_plugins_have_no_obstacle_layer():
    d = load("nav2_params_fusion.yaml")
    p = d["global_costmap"]["global_costmap"]["ros__parameters"]
    assert p["plugins"] == ["static_layer", "inflation_layer"]


def test_nav2_fusion_keeps_unknown_blocked():
    d = load("nav2_params_fusion.yaml")
    p = d["planner_server"]["ros__parameters"]["GridBased"]
    assert p["allow_unknown"] is False


def test_nav2_fusion_local_costmap_is_4m():
    d = load("nav2_params_fusion.yaml")
    p = d["local_costmap"]["local_costmap"]["ros__parameters"]
    assert p["width"] == 4 and p["height"] == 4


def test_nav2_fusion_loosens_goal_checker():
    d = load("nav2_params_fusion.yaml")
    p = d["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert p["xy_goal_tolerance"] == 0.25
    assert p["yaw_goal_tolerance"] == 3.15


def test_slam_fusion_updates_faster_and_keeps_scan_filter():
    d = load("slam_params_fusion.yaml")
    p = d["slam_toolbox"]["ros__parameters"]
    assert p["map_update_interval"] == 1.0
    assert p["scan_topic"] == "/scan_filtered"
    assert p["max_laser_range"] == 11.5


def test_slam_base_clamps_max_range():
    d = load("slam_params.yaml")
    p = d["slam_toolbox"]["ros__parameters"]
    assert p["max_laser_range"] == 11.5
