import subprocess
from pathlib import Path

from carhealth.busy import BusyState
from carhealth.recovery import (ACTION_ARGS, RECOVER_CMD, Recovery,
                                RecoveryConfig, RecoveryRequest)
from carhealth.types import ComponentState, Diagnosis, Verdict

IDLE = BusyState(False, False, False, False)


def st(name, verdict, root=None, since=0.0):
    return ComponentState(name, name, verdict, "r", {}, root, (), (), since, since)


def diag(component, rule_id):
    return {component: [Diagnosis(rule_id, rule_id, "x", "h")]}


def cfg(**kw):
    base = dict(enabled=True, startup_grace_s=0.0, settle_s=10.0,
                budget_max=10, cooldowns={"wifi_powersave_off": 0.0,
                                         "ros_daemon_stop": 0.0,
                                         "restart_stack": 0.0,
                                         "restart_voice": 0.0})
    base.update(kw)
    return RecoveryConfig(**base)


class Runner:
    def __init__(self, rc=0):
        self.cmds = []
        self.rc = rc

    def __call__(self, cmd, **kw):
        self.cmds.append(cmd)
        return subprocess.CompletedProcess(cmd, self.rc, "ok", "")


def test_wifi_powersave_triggers_even_when_busy():
    r = Recovery(cfg(), runner=Runner())
    req = r.decide(states={"host": st("host", Verdict.OK)},
                   diag=diag("host", "host.wifi_powersave"),
                   busy=BusyState(True, True, True, True),
                   network_changed=False, now=100.0)
    assert req is not None and req.action == "wifi_powersave_off"
    assert req.deferred is False


def test_restart_stack_on_network_change_with_blocked_ros():
    r = Recovery(cfg(), runner=Runner())
    req = r.decide(states={"chassis": st("chassis", Verdict.BLOCKED, "ros_discovery")},
                   diag={}, busy=IDLE, network_changed=True, now=100.0)
    assert req is not None and req.action == "restart_stack"
    assert req.targets == ("chassis",)


def test_restart_stack_deferred_while_busy():
    r = Recovery(cfg(), runner=Runner())
    req = r.decide(states={"chassis": st("chassis", Verdict.BLOCKED, "ros_discovery")},
                   diag={}, busy=BusyState(True, False, False, False),
                   network_changed=True, now=100.0)
    assert req.deferred is True and req.detail == "interlock"


def test_one_blocked_without_network_change_does_nothing():
    r = Recovery(cfg(), runner=Runner())
    assert r.decide(states={"chassis": st("chassis", Verdict.BLOCKED, "ros_discovery")},
                    diag={}, busy=IDLE, network_changed=False, now=100.0) is None


def test_ros_daemon_stop_for_stale_ip_only():
    r = Recovery(cfg(), runner=Runner())
    req = r.decide(states={"chassis": st("chassis", Verdict.OK)},
                   diag=diag("chassis", "dds.stale_ip"),
                   busy=IDLE, network_changed=False, now=100.0)
    assert req is not None and req.action == "ros_daemon_stop"


def test_voice_restart_after_persistent_wakeword_loss():
    r = Recovery(cfg(), runner=Runner())
    states = {"voice": st("voice", Verdict.STALE, since=0.0)}
    req = r.decide(states=states, diag=diag("voice", "voice.wakeword_lost"),
                   busy=IDLE, network_changed=False, now=100.0)
    assert req is not None and req.action == "restart_voice"


def test_voice_restart_deferred_when_voice_busy():
    r = Recovery(cfg(), runner=Runner())
    states = {"voice": st("voice", Verdict.STALE, since=0.0)}
    req = r.decide(states=states, diag=diag("voice", "voice.wakeword_lost"),
                   busy=BusyState(False, False, False, True),
                   network_changed=False, now=100.0)
    assert req.deferred is True


def test_execute_runs_privileged_script_and_obeys_cooldown():
    runner = Runner()
    r = Recovery(cfg(cooldowns={"wifi_powersave_off": 300.0}), runner=runner)
    req = r.decide(states={"host": st("host", Verdict.OK)},
                   diag=diag("host", "host.wifi_powersave"),
                   busy=IDLE, network_changed=False, now=100.0)
    result = r.execute(req, 100.0)
    assert runner.cmds == [["sudo", "-n", "/usr/local/sbin/carhealth-recover",
                            "wifi-powersave-off"]]
    assert result["rc"] == 0
    # 验证窗口内不重复决策
    assert r.decide(states={}, diag=diag("host", "host.wifi_powersave"),
                    busy=IDLE, network_changed=False, now=105.0) is None
    # 过了验证窗口，冷却仍未过 -> deferred
    again = r.decide(states={}, diag=diag("host", "host.wifi_powersave"),
                     busy=IDLE, network_changed=False, now=120.0)
    assert again.deferred is True


def test_restart_stack_runs_restart_ros_stack_subcommand():
    """11-18 现场：restart_stack 被 `replace("_","-")` 拼成 restart-stack，
    特权脚本回 "unknown action"（rc=2），整栈重启其实一次都没跑过。"""
    runner = Runner()
    r = Recovery(cfg(), runner=runner)
    req = r.decide(states={"chassis": st("chassis", Verdict.BLOCKED, "ros_discovery")},
                   diag={}, busy=IDLE, network_changed=True, now=100.0)
    assert req.action == "restart_stack"
    res = r.execute(req, 100.0)
    assert runner.cmds == [["sudo", "-n", RECOVER_CMD, "restart-ros-stack"]]
    assert res["rc"] == 0


def test_every_action_maps_to_a_subcommand_the_privileged_script_accepts():
    """ACTION_ARGS 与 tools/carhealth-recover 的 case 分支不许漂移。"""
    script = (Path(__file__).resolve().parents[1] / "tools" /
              "carhealth-recover").read_text(encoding="utf-8")
    for action, arg in ACTION_ARGS.items():
        assert f"    {arg})" in script, f"{action} -> {arg} 不在特权脚本里"


def test_unknown_action_does_not_invoke_privileged_script():
    """没登记的动作必须失败关闭，不能把拼出来的字符串交给 sudo 脚本。"""
    runner = Runner()
    r = Recovery(cfg(), runner=runner)
    res = r.execute(RecoveryRequest("mystery_action", "x"), 100.0)
    assert runner.cmds == []
    assert res["rc"] != 0
    assert "unknown action" in res["output"]


def test_observe_success_and_failure_escalation():
    r = Recovery(cfg(fail_escalate=2), runner=Runner(rc=1))
    req = r.decide(states={"host": st("host", Verdict.OK)},
                   diag=diag("host", "host.wifi_powersave"),
                   busy=IDLE, network_changed=False, now=100.0)
    r.execute(req, 100.0)
    ok = r.observe(states={"host": st("host", Verdict.OK)}, diag={}, now=111.0)
    assert ok["ok"] is True

    r2 = Recovery(cfg(fail_escalate=2), runner=Runner(rc=1))
    req = r2.decide(states={"host": st("host", Verdict.OK)},
                    diag=diag("host", "host.wifi_powersave"),
                    busy=IDLE, network_changed=False, now=100.0)
    r2.execute(req, 100.0)
    fail = r2.observe(states={"host": st("host", Verdict.OK)},
                      diag=diag("host", "host.wifi_powersave"), now=111.0)
    assert fail["ok"] is False and fail["failures"] == 1


def test_kill_switch_and_startup_grace(tmp_path):
    flag = tmp_path / "recovery-off"
    r = Recovery(cfg(kill_switch=flag), runner=Runner())
    assert r.decide(states={}, diag=diag("host", "host.wifi_powersave"),
                    busy=IDLE, network_changed=False, now=100.0) is not None
    flag.write_text("paused")
    assert r.decide(states={}, diag=diag("host", "host.wifi_powersave"),
                    busy=IDLE, network_changed=False, now=200.0) is None

    r2 = Recovery(cfg(startup_grace_s=60.0), runner=Runner())
    assert r2.decide(states={}, diag=diag("host", "host.wifi_powersave"),
                     busy=IDLE, network_changed=False, now=10.0) is None
