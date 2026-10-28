# -*- coding: utf-8 -*-
"""arm_gate.CmdGate 测试（不需要 ROS）：
    cd pi/raspot_ws/src && python -m pytest raspot_teleop/test/test_arm_gate.py -q
"""
from raspot_teleop.arm_gate import CmdGate


def test_no_topic_yet_allows_without_resume():
    """驱动还没发过（旧驱动）：放行，且不要在启动时误触发重设。"""
    g = CmdGate()
    assert g.step(1.0) == (True, False)


def test_first_live_counts_as_resume():
    """启动就 READY：第一拍 verified 要重设目标（可能刚在假回读上初始化过）。"""
    g = CmdGate()
    g.on_msg(True, 1.0)
    assert g.step(1.0) == (True, True)
    g.on_msg(True, 1.04)
    assert g.step(1.04) == (True, False)     # 只来一次


def test_rehome_blocks_then_resumes_once():
    g = CmdGate()
    g.on_msg(True, 1.0)
    g.step(1.0)
    g.on_msg(False, 1.1)                     # 回中开始
    assert g.step(1.1) == (False, False)
    g.on_msg(False, 1.2)
    assert g.step(1.2) == (False, False)
    g.on_msg(True, 5.0)                      # 复位结束
    assert g.step(5.0) == (True, True)
    g.on_msg(True, 5.1)
    assert g.step(5.1) == (True, False)


def test_never_seen_live_bit_does_not_block():
    """老固件不报 byte19：一直 0 也不能门死，也不能触发重设。"""
    g = CmdGate()
    for i in range(5):
        now = 1.0 + i * 0.04
        g.on_msg(False, now)
        assert g.step(now) == (True, False)


def test_topic_going_stale_reopens_and_next_live_resumes():
    """驱动重启：话题停更新 ⇒ 放行（旧行为），再收到 1 时按回读重设。"""
    g = CmdGate(stale_s=1.0)
    g.on_msg(True, 1.0)
    g.step(1.0)
    g.on_msg(False, 1.1)
    assert g.step(1.1)[0] is False
    assert g.step(2.5)[0] is True            # 1.4s 没更新
    g.on_msg(True, 3.0)
    assert g.step(3.0) == (True, True)


def test_no_repeat_resume_while_blocked():
    """回中期间每拍都被门住，不会因为消息间隔抖动而反复触发 resumed。"""
    g = CmdGate()
    g.on_msg(True, 1.0)
    g.step(1.0)
    edges = 0
    for i in range(10):
        now = 1.1 + i * 0.04
        g.on_msg(False, now)
        edges += g.step(now)[1]
    assert edges == 0
