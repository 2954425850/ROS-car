from carhealth.busy import BusyState, compute
from carhealth.types import ProbeResult


def r(**ev):
    return ProbeResult(ok=True, evidence=ev)


def test_idle_everything():
    assert compute({}, {}).any is False


def test_arm_command_makes_arm_busy():
    st = compute({"topic:/arm/command": r(age_s=0.2)}, {})
    assert st.arm is True


def test_stale_arm_command_does_not_count():
    st = compute({"topic:/arm/command": r(age_s=5.0)}, {})
    assert st.arm is False


def test_chassis_from_cmd_vel_or_odom():
    assert compute({"topic:/cmd_vel_mux": r(age_s=0.1, linear_x=0.2, angular_z=0.0)}, {}).chassis
    assert compute({"topic:/wheel_odom": r(age_s=0.1, linear_x=0.0, angular_z=0.6)}, {}).chassis
    # 车被推着直线走：只看角速度会判成空闲，二期这就是自动重启的开关。
    assert compute({"topic:/wheel_odom": r(age_s=0.1, linear_x=0.3, angular_z=0.0)}, {}).chassis
    assert not compute({"topic:/cmd_vel_mux": r(age_s=0.1, linear_x=0.0, angular_z=0.0)}, {}).chassis


def test_grasp_busy_from_health_topic():
    st = compute({"topic:/health/arm_grasp": r(age_s=0.5, busy=True)}, {})
    assert st.grasp is True
    assert st.arm is True          # 抓取也是臂在动


def test_voice_busy_from_file_evidence():
    assert compute({}, {"busy": True}).voice is True
    assert compute({}, {"busy": False}).voice is False


def test_any_combines():
    st = BusyState(arm=False, chassis=False, grasp=False, voice=True)
    assert st.any is True
