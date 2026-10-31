# -*- coding: utf-8 -*-
"""grasp_server 里 /arm/motion 的胶水逻辑（用桩模块替掉 rclpy 等，Windows 也能跑）。

不起 ROS：对象用 object.__new__ 造，只填动作路径用到的属性；驱动参数服务换成记录调用的假函数；
/arm/command 的发布器换成记录列表，回读由测试直接塞进 `_mfb`（或用一个"跟随目标"的假臂刷新）。
"""
import importlib
import json
import sys
import threading
import time
import types

import pytest


class _Msg:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=None)
        self.name, self.position = [], []


class _Action:
    class Goal:
        pass

    class Result:
        def __init__(self):
            self.ok, self.code, self.reason, self.state_json = False, '', '', ''

    class Feedback:
        def __init__(self):
            self.step = self.total = 0
            self.detail = ''


def _stubs():
    m = {}

    def mod(name, **attrs):
        x = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(x, k, v)
        m[name] = x
        return x

    class Node:
        def __init__(self, *a, **k):
            pass
    mod('rclpy', init=None, ok=lambda: False, shutdown=None)
    mod('rclpy.action', ActionServer=object, CancelResponse=types.SimpleNamespace(ACCEPT=1),
        GoalResponse=types.SimpleNamespace(ACCEPT=1, REJECT=0))
    mod('rclpy.callback_groups', ReentrantCallbackGroup=object)
    mod('rclpy.executors', MultiThreadedExecutor=object)
    mod('rclpy.node', Node=Node)
    mod('rclpy.parameter', Parameter=object)
    mod('sensor_msgs')
    mod('sensor_msgs.msg', JointState=_Msg)
    mod('std_msgs')
    mod('std_msgs.msg', Bool=object, String=object)
    mod('std_srvs')
    mod('std_srvs.srv', Trigger=object)
    mod('raspot_interfaces')
    mod('raspot_interfaces.action', ArmMotion=_Action, Grasp=_Action)
    return m


@pytest.fixture
def gs(monkeypatch):
    for k, v in _stubs().items():
        monkeypatch.setitem(sys.modules, k, v)
    monkeypatch.delitem(sys.modules, 'arm_grasp.grasp_server', raising=False)
    mod = importlib.import_module('arm_grasp.grasp_server')
    yield mod
    sys.modules.pop('arm_grasp.grasp_server', None)


HOME = [240.0, 496.0, 180.0, 129.0, 415.0, 0.0]


class GH:
    def __init__(self, spec, dry_run=False):
        self.request = types.SimpleNamespace(
            spec_json=json.dumps(spec, ensure_ascii=False), dry_run=dry_run)
        self.status, self.fbs = None, []

    def succeed(self):
        self.status = 'succeeded'

    def abort(self):
        self.status = 'aborted'

    def canceled(self):
        self.status = 'canceled'

    def publish_feedback(self, fb):
        self.fbs.append((fb.step, fb.total, fb.detail))


def make(gs, fb=HOME, follow=True):
    """造一个只够跑动作路径的 GraspServer。follow=True：每次发布后回读立即跳到目标（理想臂）。"""
    from arm_grasp import motion_exec, motion_spec
    s = object.__new__(gs.GraspServer)
    s._lock = threading.Lock()
    s._motion_lock = threading.Lock()
    s._busy, s._abort = False, None
    s._grip_cmd, s._estop_on = None, False
    s._gate = motion_exec.CmdGate()
    s._history = motion_exec.PoseHistory(maxlen=5)
    s._motion_err = None
    s._mcfg = motion_spec.load_config()
    s._ecfg, s._scfg = motion_exec.load_runtime_config()
    s._mfb = (list(fb), time.monotonic()) if fb is not None else None
    s.pubs, s.t_ms = [], []
    log = types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None, error=lambda *a: None)
    s.get_logger = lambda: log
    s.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    s.get_parameter = lambda name: types.SimpleNamespace(value={'normal_arm_t_ms': 40}[name])

    def set_t(v):
        s.t_ms.append(int(v))
        return True
    s._set_driver_t_ms = set_t

    def pub(m):
        s.pubs.append((m.header.frame_id, list(m.position)))
        s._grip_cmd = m.position[0]
        if follow:
            s._mfb = (list(m.position), time.monotonic())
    s.pub_arm = types.SimpleNamespace(publish=pub)

    # 理想臂到位后回读要保持新鲜
    orig = s.motion_fb

    def fresh():
        if s._mfb is not None:
            s._mfb = (s._mfb[0], time.monotonic())
        return orig()
    s.motion_fb = fresh
    return s


def test_dry_run_plans_without_publishing(gs):
    s = make(gs)
    gh = GH({'steps': [{'turn': [{'dir': '下', 'amount': '一点'}]}]}, dry_run=True)
    res = s._execute_motion(gh)
    assert res.ok and res.code == 'OK' and gh.status == 'succeeded'
    assert res.reason.startswith('dry-run（没动）：已完成：往下转一点')
    assert s.pubs == [] and s.t_ms == []
    assert json.loads(res.state_json)['gripper']['state'] == '张开'
    assert s._busy is False and s._history.items == []


def test_bad_goal(gs):
    s = make(gs)
    res = s._execute_motion(GH({'steps': [{'turn': [{'dir': '斜'}]}]}))
    assert res.code == 'BAD_GOAL' and not res.ok and s.pubs == []
    assert s._busy is False


def test_no_feedback(gs):
    s = make(gs, fb=None)
    res = s._execute_motion(GH({'steps': [{'gripper': '合上'}]}))
    assert res.code == 'NO_FEEDBACK' and s.pubs == []


def test_gate_dead_is_busy(gs):
    s = make(gs)
    now = time.monotonic()
    s._gate.on_msg(True, now)
    s._gate.on_msg(False, now)
    res = s._execute_motion(GH({'steps': [{'gripper': '合上'}]}))
    assert res.code == 'BUSY' and '复位' in res.reason and s.pubs == []


def test_busy_rejected_in_execute(gs):
    s = make(gs)
    s._busy = True
    res = s._execute_motion(GH({'steps': [{'gripper': '合上'}]}))
    assert res.code == 'BUSY' and s.pubs == []
    assert s._busy is True               # 别人的忙标志不许被清掉


def test_real_run_publishes_motion_frames_and_pushes_history(gs):
    s = make(gs)
    gh = GH({'steps': [{'turn': [{'dir': '下', 'amount': '一点'}]}]})
    res = s._execute_motion(gh)
    assert res.ok and res.code == 'OK', res.reason
    assert [p[0] for p in s.pubs] == ['motion']
    assert s.t_ms[-1] == 40                # 还原 normal_arm_t_ms
    assert s._history.items == [HOME]      # 动作前姿态
    assert gh.fbs and gh.fbs[0][:2] == (1, 1)
    # 「上一个姿态」弹出栈顶并回去
    res = s._execute_motion(GH({'steps': [{'toward': '上一个姿态'}]}))
    assert res.ok, res.reason
    assert s.pubs[-1][1][2] == pytest.approx(HOME[2], abs=0.01)


def test_holding_gripper_not_loosened_by_motion(gs):
    s = make(gs, fb=[520.0] + HOME[1:])
    s._grip_cmd = 578.0                    # 合爪被物体挡在 520
    res = s._execute_motion(GH({'steps': [{'turn': [{'dir': '左', 'amount': '一点'}]}]}))
    assert res.ok, res.reason
    assert s.pubs[0][1][0] == 578.0


def test_teleop_abort_zero_publish(gs):
    s = make(gs, follow=False)             # 臂不动 ⇒ 帧会一直等
    gh = GH({'steps': [{'turn': [{'dir': '下', 'amount': '一些'}]}, {'gripper': '合上'}]})
    out = {}
    th = threading.Thread(target=lambda: out.setdefault('res', s._execute_motion(gh)))
    th.start()
    end = time.monotonic() + 2.0
    while not s.pubs and time.monotonic() < end:
        time.sleep(0.01)
    s._busy and s._on_cmd(types.SimpleNamespace(header=types.SimpleNamespace(frame_id='teleop')))
    th.join(5.0)
    res = out['res']
    assert res.code == 'ABORTED_BY_OPERATOR' and '动作让出控制权' in res.reason
    assert len(s.pubs) == 1 and gh.status == 'aborted'
    assert s.t_ms[-1] == 40 and s._history.items == []


def test_cancel_sends_freeze(gs):
    s = make(gs, follow=False)
    gh = GH({'steps': [{'turn': [{'dir': '下', 'amount': '一些'}]}]})
    out = {}
    th = threading.Thread(target=lambda: out.setdefault('res', s._execute_motion(gh)))
    th.start()
    end = time.monotonic() + 2.0
    while not s.pubs and time.monotonic() < end:
        time.sleep(0.01)
    s._on_cancel(None)
    th.join(5.0)
    res = out['res']
    assert res.code == 'CANCELED' and gh.status == 'canceled'
    assert len(s.pubs) == 2 and s.pubs[1][1] == HOME     # 冻结 = 当前回读
    assert s.t_ms[-2:] == [s._ecfg.freeze_t_ms, 40]


def test_gate_dead_mid_motion_zero_publish(gs):
    s = make(gs, follow=False)
    gh = GH({'steps': [{'turn': [{'dir': '下', 'amount': '一些'}]}, {'gripper': '合上'}]})
    out = {}
    th = threading.Thread(target=lambda: out.setdefault('res', s._execute_motion(gh)))
    th.start()
    end = time.monotonic() + 2.0
    while not s.pubs and time.monotonic() < end:
        time.sleep(0.01)
    s._on_cmd_gate(types.SimpleNamespace(data=True))
    s._on_cmd_gate(types.SimpleNamespace(data=False))
    th.join(5.0)
    assert out['res'].code == 'ABORTED_BY_OPERATOR' and len(s.pubs) == 1


def test_describe(gs):
    s = make(gs)
    resp = s._describe_srv(None, types.SimpleNamespace(success=None, message=None))
    assert resp.success and json.loads(resp.message)['text'].startswith('视线正前方')
    s = make(gs, fb=None)
    resp = s._describe_srv(None, types.SimpleNamespace(success=None, message=None))
    assert resp.success is False


@pytest.mark.parametrize('code', ['ABORTED_BY_OPERATOR', 'ESTOP'])
def test_publish_motion_refuses_after_operator_or_estop(gs, code):
    from arm_grasp.motion_exec import Abort
    s = make(gs)
    s._abort = (code, 'x')
    with pytest.raises(Abort):
        s.publish_motion(HOME)
    assert s.pubs == []


def test_publish_motion_allows_freeze_after_cancel(gs):
    s = make(gs)
    s._abort = ('CANCELED', '取消了')
    s.publish_motion(HOME)
    assert s.pubs == [('motion', HOME)]
