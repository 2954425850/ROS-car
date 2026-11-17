import json

from arm_grasp.health import health_payload


def test_payload_shape():
    p = health_payload(busy=True, uptime_s=12.34, wall=1000.0,
                       feedback_age_s=0.2, last_goal_wall=999.0)
    assert p["component"] == "arm_grasp"
    assert p["busy"] is True
    assert p["uptime_s"] == 12.3
    assert p["wall"] == 1000.0
    assert p["feedback_age_s"] == 0.2
    assert p["last_goal_wall"] == 999.0
    assert isinstance(p["pid"], int)
    json.dumps(p)          # 必须可序列化


def test_busy_is_bool_and_missing_feedback_is_none():
    p = health_payload(busy=1, uptime_s=0, wall=0.0)
    assert p["busy"] is True
    assert p["feedback_age_s"] is None
