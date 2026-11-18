from carhealth.busy import BusyState, compute
from carhealth.types import ProbeResult


def h(**ev):
    return ProbeResult(ok=True, evidence=ev)


def test_idle_everything():
    assert compute({}, {}).any is False


def test_arm_command_makes_arm_busy():
    st = compute({"topic:/health/driver": h(age_s=0.1, arm_cmd_age_s=0.2)}, {})
    assert st.arm is True


def test_stale_arm_command_does_not_count():
    st = compute({"topic:/health/driver": h(age_s=0.1, arm_cmd_age_s=5.0)}, {})
    assert st.arm is False


def test_chassis_from_cmd_or_odom():
    assert compute({"topic:/health/driver": h(
        age_s=0.1, cmd_age_s=0.1, cmd_vx=0.2, cmd_wz=0.0)}, {}).chassis
    assert compute({"topic:/health/driver": h(
        age_s=0.1, telemetry_age_s=0.1, odom_vx=0.0, odom_wz=0.6)}, {}).chassis
    # 车被推着直线走：只看角速度会判成空闲，二期这就是自动重启的开关。
    assert compute({"topic:/health/driver": h(
        age_s=0.1, telemetry_age_s=0.1, odom_vx=0.3, odom_wz=0.0)}, {}).chassis
    assert not compute({"topic:/health/driver": h(
        age_s=0.1, cmd_age_s=0.1, cmd_vx=0.0, cmd_wz=0.0,
        telemetry_age_s=0.1, odom_vx=0.0, odom_wz=0.0)}, {}).chassis


def test_stale_health_topic_means_idle():
    st = compute({"topic:/health/driver": h(
        age_s=5.0, cmd_age_s=0.1, cmd_vx=1.0)}, {})
    assert st.chassis is False


def test_grasp_busy_from_health_topic():
    st = compute({"topic:/health/arm_grasp": h(age_s=0.5, busy=True)}, {})
    assert st.grasp is True
    assert st.arm is True          # 抓取也是臂在动


def test_voice_busy_from_file_evidence():
    assert compute({}, {"busy": True}).voice is True
    assert compute({}, {"busy": False}).voice is False


def test_any_combines():
    st = BusyState(arm=False, chassis=False, grasp=False, voice=True)
    assert st.any is True
