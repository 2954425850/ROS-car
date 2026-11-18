import json

from raspot_teleop.driver_health import build_driver_health


def test_payload_shape_and_rounding():
    p = build_driver_health(
        arm_online=True, arm_estop=False, arm_status=1, ctrl_live=True,
        arm_cmd_age_s=0.2345, chassis_faults=0, battery_voltage=12.345,
        seen55=True, seen56=True, feedback_age_s=0.04, telemetry_age_s=0.02,
        cmd_age_s=None, cmd_vx=0.1234, cmd_wz=-0.5,
        odom_vx=0.1, odom_wz=0.0, wall=1000.0)
    assert p["component"] == "l150pro_driver"
    assert p["arm_online"] is True
    assert p["arm_estop"] is False
    assert p["arm_cmd_age_s"] == 0.234
    assert p["battery_voltage"] == 12.35
    assert p["feedback_age_s"] == 0.04
    assert p["cmd_age_s"] is None
    assert p["cmd_vx"] == 0.123
    assert isinstance(p["pid"], int)
    json.dumps(p)


def test_missing_ages_stay_none():
    p = build_driver_health(
        arm_online=False, arm_estop=False, arm_status=0, ctrl_live=False,
        arm_cmd_age_s=None, chassis_faults=0, battery_voltage=0.0,
        seen55=False, seen56=False, feedback_age_s=None, telemetry_age_s=None,
        cmd_age_s=None, cmd_vx=0.0, cmd_wz=0.0, odom_vx=0.0, odom_wz=0.0,
        wall=0.0)
    assert p["feedback_age_s"] is None
    assert p["telemetry_age_s"] is None
    assert p["arm_cmd_age_s"] is None
    assert p["seen55"] is False
