from carhealth.probes.ros_topic import (
    RosTopicStore, TopicProbe, arm_health_evidence, driver_health_evidence,
    tof_status_evidence,
)


def test_snapshot_age_and_payload():
    now = [100.0]
    store = RosTopicStore(clock=lambda: now[0])
    store.record("/tof/status", {"connected": True, "fps": 12.0})
    now[0] = 101.5
    r = TopicProbe("topic:/tof/status", "/tof/status", store).poll()
    assert r.evidence["age_s"] == 1.5
    assert r.evidence["count"] == 1
    assert r.evidence["connected"] is True


def test_never_seen_topic_has_none_age():
    store = RosTopicStore(clock=lambda: 1.0)
    r = TopicProbe("topic:/x", "/x", store).poll()
    assert r.evidence["age_s"] is None
    assert r.evidence["count"] == 0


def test_blind_store_returns_unknown_not_stale():
    store = RosTopicStore(clock=lambda: 1.0)
    store.set_blind("rclpy 不可用")
    r = TopicProbe("topic:/x", "/x", store).poll()
    assert r.ok is None
    assert "rclpy" in r.error


def test_publisher_count_is_included():
    store = RosTopicStore(clock=lambda: 1.0)
    store.set_publisher_counter(lambda topic: 0 if topic == "/x" else 3)
    store.record("/x", {})
    assert store.snapshot("/x").evidence["publishers"] == 0
    assert store.snapshot("/never").evidence["publishers"] == 3


def test_publisher_counter_failure_is_none_not_zero():
    def boom(topic):
        raise RuntimeError("graph api down")

    store = RosTopicStore(clock=lambda: 1.0)
    store.set_publisher_counter(boom)
    assert store.snapshot("/x").evidence["publishers"] is None


def test_store_is_thread_safe_enough_for_counter():
    store = RosTopicStore(clock=lambda: 1.0)
    for _ in range(3):
        store.record("/x", {})
    assert store.snapshot("/x").evidence["count"] == 3


def test_negative_age_is_clamped():
    now = [100.0]
    store = RosTopicStore(clock=lambda: now[0])
    store.record("/x", {})
    now[0] = 99.0
    assert store.snapshot("/x").evidence["age_s"] == 0.0


def test_tof_status_keeps_only_contract_fields():
    ev = tof_status_evidence({"connected": True, "fps": 9.0, "reconnects": 2,
                              "age_s": 0.2, "last_error": None, "huge": "x" * 1000})
    assert ev["fps"] == 9.0
    assert "huge" not in ev


def test_arm_and_driver_health_converters():
    assert arm_health_evidence({"busy": True, "pid": 1, "feedback_age_s": 0.2})["busy"] is True
    ev = driver_health_evidence({
        "arm_online": True, "battery_voltage": 12.1, "cmd_vx": 0.2, "junk": 1})
    assert ev["arm_online"] is True
    assert ev["battery_voltage"] == 12.1
    assert ev["cmd_vx"] == 0.2
    assert "junk" not in ev
