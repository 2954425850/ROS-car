# -*- coding: utf-8 -*-
"""motion_exec：逐帧执行（假 Link）+ 状态描述 + 门/夹爪/文案小件。"""
import json

import pytest

from arm_grasp.arm_kin import to_fields
from arm_grasp.motion_exec import (
    Abort, CmdGate, ExecConfig, PoseHistory, StateConfig, compose_reason, describe_state,
    final_code, grip_hold_value, gripper_state, load_runtime_config, run_motion, spec_summary)
from arm_grasp.motion_plan import Keyframe, Plan, plan
from arm_grasp.motion_spec import load_config, parse_spec

ECFG, SCFG = load_runtime_config()
MCFG = load_config()
HOME = [240.0, 496.0, 180.0, 129.0, 415.0, 0.0]


class FakeArm:
    """假 Link：时钟是假的；回读按"发布时刻驱动现读的 t_ms"线性逼近目标。

    注入：drop={发布序号}（这帧被串口丢了）、stuck={下标: 卡住的值}（被挡住，过不去）、
    events=[(时刻, code, reason)]（到时刻 spin 抛打断）、fb_none_after=时刻（之后回读 None）。
    """

    def __init__(self, start=HOME, drop=(), stuck=None, events=(), fb_none_after=None,
                 set_ok=True, fb_none=False):
        self.t = 0.0
        self.pos = [float(v) for v in start]
        self.motion = None            # (t0, from, to, t_ms)
        self.drv_t_ms = 40
        self.drop = set(drop)
        self.stuck = dict(stuck or {})
        self.events = list(events)
        self.fb_none_after = fb_none_after
        self.fb_none = fb_none
        self.set_ok = set_ok
        self.abort = None
        self.pubs = []                # (t, fields, t_ms)
        self.t_ms_log = []

    # -- Link --
    def now(self):
        return self.t

    def fb(self):
        if self.fb_none or (self.fb_none_after is not None and self.t >= self.fb_none_after):
            return None
        self._advance()
        return list(self.pos)

    def set_t_ms(self, ms):
        self.t_ms_log.append(ms)
        if not self.set_ok:
            return False
        self.drv_t_ms = ms
        return True

    def publish(self, fields):
        if self.abort and self.abort[0] in ('ABORTED_BY_OPERATOR', 'ESTOP'):
            raise Abort(*self.abort)
        idx = len(self.pubs)
        self.pubs.append((self.t, list(fields), self.drv_t_ms))
        if idx in self.drop:
            return
        self._advance()
        self.motion = (self.t, list(self.pos), list(fields), max(1, self.drv_t_ms))

    def spin(self, dt):
        end = self.t + dt
        while True:
            for ev in list(self.events):
                if ev[0] <= self.t:
                    self.events.remove(ev)
                    self.abort = self.abort or (ev[1], ev[2])
            if self.abort:
                raise Abort(*self.abort)
            if self.t >= end - 1e-12:
                return
            self.t = min(end, self.t + 0.01)
            self._advance()

    # -- 物理 --
    def _advance(self):
        if self.motion is None:
            return
        t0, a, b, tm = self.motion
        r = min(1.0, (self.t - t0) * 1000.0 / tm)
        for i in range(6):
            v = a[i] + (b[i] - a[i]) * r
            if i in self.stuck:
                lim = self.stuck[i]
                v = min(v, lim) if b[i] >= a[i] else max(v, lim)
            self.pos[i] = v


def KP(*frames):
    return Plan([Keyframe(list(f), t, 'f%d' % i) for i, (f, t) in enumerate(frames)], [], False)


def moved(base, **kw):
    f = list(base)
    for k, v in kw.items():
        f[int(k[1:]) - 1] = v
    return f


F1 = moved(HOME, p3=260.0)                  # 腕俯仰 +80 field
F2 = moved(F1, p6=40.0)                     # 底座 +40 field
F3 = moved(F2, p1=578.0, p4=200.0)          # 合爪 + 肘


def run(arm, p, cfg=ECFG, **kw):
    st = {}
    code, reason = run_motion(p, arm, cfg, stats=st, **kw)
    return code, reason, st


# ---------------------------------------------------------------- 正常

def test_multi_frame_all_reached():
    arm = FakeArm()
    code, reason, st = run(arm, KP((F1, 400), (F2, 300), (F3, 500)))
    assert code == 'OK', reason
    assert st['published'] == 3 and st['resends'] == 0 and st['frames_done'] == 3
    assert [p[2] for p in arm.pubs] == [400, 300, 500]          # 每帧先设 t_ms 再发
    assert [p[1] for p in arm.pubs] == [F1, F2, F3]
    for i in range(1, 6):
        assert arm.pos[i] == pytest.approx(F3[i], abs=1e-6)
    assert arm.t_ms_log[-1] == ECFG.normal_t_ms                 # 还原


def test_each_frame_waits_at_least_t_ms():
    arm = FakeArm()
    run(arm, KP((F1, 400), (F2, 800)))
    t = [p[0] for p in arm.pubs]
    assert t[1] - t[0] >= 0.4 - 1e-9
    assert arm.t >= t[1] + 0.8 - 1e-9


def test_gripper_only_frame_waits_t_ms_without_checking_gripper():
    """夹爪被物体挡住（卡在 500 到不了 578）也不算没到位，等够 t_ms 就走。"""
    arm = FakeArm(stuck={0: 500.0})
    code, reason, st = run(arm, KP((moved(HOME, p1=578.0), 600)))
    assert code == 'OK', reason
    assert arm.t >= 0.6 - 1e-9
    assert arm.t < 0.6 + ECFG.timeout_extra_ms / 1000.0 - 0.1   # 没有等到超时
    assert st['published'] == 1


def test_real_plan_runs():
    p = plan({'steps': [{'oscillate': {'axis': '上下', 'amount': '一些', 'times': 2}}]}, HOME, [], MCFG)
    arm = FakeArm()
    code, reason, st = run(arm, p)
    assert code == 'OK', reason
    assert st['published'] == len(p.keyframes) == 5
    assert arm.pos[2] == pytest.approx(HOME[2], abs=1e-6)


# ---------------------------------------------------------------- 丢帧重发

def test_dropped_first_frame_resent_once_then_reached():
    arm = FakeArm(drop={0})
    code, reason, st = run(arm, KP((F1, 1000), (F2, 300)))
    assert code == 'OK', reason
    assert st['resends'] == 1 and st['published'] == 3
    t0, t1 = arm.pubs[0][0], arm.pubs[1][0]
    assert 0.25 - 1e-9 <= t1 - t0 <= 0.25 + 2 * ECFG.poll_s
    assert arm.pubs[1][1] == F1
    assert arm.pubs[1][2] < 1000 and arm.pubs[1][2] >= 700    # 以剩余时间重发


def test_resend_at_most_once_then_partial():
    arm = FakeArm(drop={0, 1})
    code, reason, st = run(arm, KP((F1, 400),))
    assert code == 'PARTIAL'
    assert st['resends'] == 1 and st['published'] == 2
    assert '腕俯仰' in reason


def test_no_resend_when_moving():
    arm = FakeArm()
    _c, _r, st = run(arm, KP((F1, 2000),))
    assert st['resends'] == 0


# ---------------------------------------------------------------- 到不了

def test_unreachable_reports_residual_without_pushing():
    """肘被挡在 160（目标 200）：超时记残差、PARTIAL；发布次数有上限，不加压。"""
    arm = FakeArm(stuck={3: 160.0})
    p = KP((moved(HOME, p4=200.0), 300), (moved(HOME, p4=200.0, p3=200.0), 300))
    code, reason, st = run(arm, p)
    assert code == 'PARTIAL'
    assert '肘差' in reason and '10°' in reason            # 40 field / (1000/240) ≈ 9.6°
    assert st['published'] <= 2 * len(p.keyframes)
    assert all(pub[1][3] == 200.0 for pub in arm.pubs)    # 目标从没被往外"加码"
    assert len(st['misses']) == 2


def test_timeout_is_t_ms_plus_extra():
    arm = FakeArm(stuck={3: 160.0})
    run(arm, KP((moved(HOME, p4=200.0), 300),))
    total = 0.3 + ECFG.timeout_extra_ms / 1000.0
    assert total - 1e-9 <= arm.t <= total + 2 * ECFG.poll_s


def test_base_tolerance_is_tighter():
    """底座差 7 field（> tol_base 6）不算到位；腕差 7（< tol_joint 8）算。"""
    arm = FakeArm(stuck={5: 33.0})
    code, _r, st = run(arm, KP((moved(HOME, p6=40.0), 300),))
    assert code == 'PARTIAL' and 5 in st['misses'][0][1]
    arm = FakeArm(stuck={2: 253.0})
    code, _r, _st = run(arm, KP((moved(HOME, p3=260.0), 300),))
    assert code == 'OK'


# ---------------------------------------------------------------- 打断

def test_cancel_freezes_at_current_readback():
    arm = FakeArm(events=[(0.3, 'CANCELED', '取消了')])
    code, reason, st = run(arm, KP((F1, 1000), (F2, 300)))
    assert code == 'CANCELED'
    assert len(arm.pubs) == 2
    t, frz, t_ms = arm.pubs[-1]
    assert t_ms == ECFG.freeze_t_ms
    assert 180.0 < frz[2] < 260.0                        # 中途的回读，不是目标
    assert frz[2] == pytest.approx(st['freeze'][2])
    assert arm.t_ms_log[-1] == ECFG.normal_t_ms


def test_cancel_freeze_keeps_gripper_command_when_blocked():
    arm = FakeArm(stuck={0: 520.0}, events=[(0.95, 'CANCELED', '取消了')])
    run(arm, KP((moved(F1, p1=578.0), 1000),))
    frz = arm.pubs[-1][1]
    assert frz[0] == 578.0                               # 被挡住 ⇒ 保持指令值，不松爪
    assert arm.pos[0] == 520.0


def test_cancel_freeze_mid_open_uses_readback():
    arm = FakeArm(events=[(0.5, 'CANCELED', '取消了')])
    run(arm, KP((moved(F1, p1=578.0), 1000),))
    frz = arm.pubs[-1][1]
    assert 300 < frz[0] < 500                            # 合到一半 ⇒ 停在回读


@pytest.mark.parametrize('code', ['ABORTED_BY_OPERATOR', 'ESTOP'])
def test_operator_or_estop_zero_publish_after(code):
    arm = FakeArm(events=[(0.3, code, 'x')])
    got, _r, st = run(arm, KP((F1, 1000), (F2, 300), (F3, 300)))
    assert got == code
    assert [p[0] for p in arm.pubs] == [0.0]             # 只有打断前那一帧
    assert 'freeze' not in st
    assert arm.t_ms_log[-1] == ECFG.normal_t_ms


def test_operator_between_spin_and_publish_still_zero_publish():
    """打断在"最后一次查"之后才到：publish 自己拒发。"""
    arm = FakeArm()
    orig = arm.spin

    def spin(dt):
        orig(dt)
        if dt == 0 and len(arm.pubs) == 1:     # 第二帧发之前的最后一次查已经过了
            arm.abort = ('ABORTED_BY_OPERATOR', '手柄')
    arm.spin = spin
    code, _r, _st = run(arm, KP((F1, 300), (F2, 300)))
    assert code == 'ABORTED_BY_OPERATOR'
    assert len(arm.pubs) == 1


# ---------------------------------------------------------------- 回读 / t_ms

def test_no_feedback_at_start_publishes_nothing():
    arm = FakeArm(fb_none=True)
    code, reason, st = run(arm, KP((F1, 300),))
    assert code == 'NO_FEEDBACK' and arm.pubs == [] and arm.t_ms_log == []
    assert '没动' in reason


def test_feedback_lost_mid_motion():
    arm = FakeArm(fb_none_after=0.2)
    code, reason, _st = run(arm, KP((F1, 3000),))
    assert code == 'NO_FEEDBACK' and '回读丢了' in reason
    assert arm.t_ms_log[-1] == ECFG.normal_t_ms


def test_set_t_ms_failure_does_not_publish():
    arm = FakeArm(set_ok=False)
    code, _r, _st = run(arm, KP((F1, 300),))
    assert code == 'ERROR' and arm.pubs == []


@pytest.mark.parametrize('arm', [
    FakeArm(), FakeArm(drop={0}), FakeArm(stuck={3: 150.0}),
    FakeArm(events=[(0.2, 'CANCELED', 'c')]), FakeArm(events=[(0.2, 'ESTOP', 'e')]),
    FakeArm(events=[(0.2, 'ABORTED_BY_OPERATOR', 't')]), FakeArm(fb_none_after=0.1),
])
def test_t_ms_restored_on_every_path(arm):
    run(arm, KP((moved(HOME, p4=200.0), 600), (F2, 300)), cfg=ExecConfig(normal_t_ms=37))
    assert arm.t_ms_log[-1] == 37


def test_exception_in_link_still_restores_t_ms():
    arm = FakeArm()

    def boom(_f):
        raise RuntimeError('串口炸了')
    arm.publish = boom
    with pytest.raises(RuntimeError):
        run(arm, KP((F1, 300),))
    assert arm.t_ms_log[-1] == ECFG.normal_t_ms


def test_same_t_ms_not_set_twice():
    arm = FakeArm()
    run(arm, KP((F1, 300), (F2, 300), (F3, 500)))
    assert arm.t_ms_log == [300, 500, ECFG.normal_t_ms]


# ---------------------------------------------------------------- 门 / 夹爪

def test_cmd_gate_three_states():
    g = CmdGate(stale_s=1.0)
    assert g.state(0.0) == 'unknown'
    g.on_msg(False, 0.0)
    assert g.state(0.1) == 'unknown'          # 从没见过 1（老驱动）⇒ 放行
    g.on_msg(True, 0.2)
    assert g.state(0.3) == 'live'
    g.on_msg(False, 0.4)
    assert g.state(0.5) == 'dead' and g.dead(0.5)
    assert g.state(1.5) == 'unknown'          # 过期


def test_grip_hold_value():
    assert grip_hold_value(520, 578, SCFG) == 578        # 夹着东西
    assert grip_hold_value(559, 578, SCFG) == 578        # 空爪合到底
    assert grip_hold_value(240, 578, SCFG) == 240        # 对不上（被手柄掰开过）⇒ 回读
    assert grip_hold_value(300, 240, SCFG) == 300        # 读回比指令大 ⇒ 回读
    assert grip_hold_value(400, None, SCFG) == 400


def test_gripper_state():
    assert gripper_state(245, None, SCFG) == '张开'
    assert gripper_state(559, 578, SCFG) == '合上'
    assert gripper_state(500, 578, SCFG) == '夹着东西'
    assert gripper_state(500, None, SCFG) == '夹着东西'
    assert gripper_state(402, 400, SCFG) == '半开'
    assert gripper_state(380, 578, SCFG) == '半开'       # 没到 holding_min


# ---------------------------------------------------------------- 状态描述

def test_describe_home():
    s = describe_state(HOME, SCFG)
    assert s['gaze_deg']['az'] == 0.0
    assert s['gripper']['state'] == '张开'
    assert '正前方' in s['text'] and '夹爪张开' in s['text']
    assert '向下' in s['text']
    json.dumps(s, ensure_ascii=False)


def test_describe_left_down_holding():
    f = to_fields({'base': 20.0, 'shoulder': 80.0, 'elbow': -60.0, 'wrist_pitch': -55.0}, 500.0, 496.0)
    s = describe_state(f, SCFG, grip_cmd=578)
    assert s['gaze_deg'] == {'az': 20.0, 'el': -35.0}
    assert s['text'].startswith('视线朝左 20°、向下 35°；爪尖在车前')
    assert '偏左' in s['text'] and s['text'].endswith('夹爪夹着东西')
    assert s['tip_cm']['y'] > 0


def test_describe_right_up_behind():
    f = to_fields({'base': -120.0, 'shoulder': 90.0, 'elbow': 0.0, 'wrist_pitch': -70.0}, 578.0, 496.0)
    s = describe_state(f, SCFG)
    assert '朝右 120°（在车后方那一侧）' in s['text'] and '向上 20°' in s['text']
    assert '车后' in s['text'] and '夹爪合上' in s['text']


# ---------------------------------------------------------------- 文案 / 栈

def test_spec_summary_and_reason():
    spec = parse_spec({'steps': [{'turn': [{'dir': '下', 'amount': '一点'}], 'gripper': '张开'},
                                 {'oscillate': {'axis': '上下', 'times': 3}}], 'repeat': 2})
    s = spec_summary(spec)
    assert s == '往下转一点+夹爪张开，然后点头×3（重复 2 遍）'
    r = compose_reason('PARTIAL', s, ['已经低到最低了'], '没完全到位：肘差 5°')
    assert r.startswith('做了：') and '已经低到最低了' in r and '肘差 5°' in r
    assert compose_reason('OK', '看向左边', []) == '已完成：看向左边'


def test_final_code():
    assert final_code('OK', True) == 'PARTIAL'
    assert final_code('OK', False) == 'OK'
    assert final_code('CANCELED', True) == 'CANCELED'


def test_pose_history():
    h = PoseHistory(maxlen=5)
    for i in range(7):
        h.commit(0, [i] * 6)
    assert [x[0] for x in h.items] == [2, 3, 4, 5, 6]
    h.commit(2, [9] * 6)                       # 用掉 2 个（6、5），再压动作前姿态
    assert [x[0] for x in h.items] == [2, 3, 4, 9]
    snap = h.snapshot()
    snap[0][0] = 100
    assert h.items[0][0] == 2


def test_cancel_before_first_frame_publishes_nothing():
    arm = FakeArm(events=[(0.0, 'CANCELED', '取消了')])
    code, _r, st = run(arm, KP((F1, 300),))
    assert code == 'CANCELED' and arm.pubs == [] and 'freeze' not in st
