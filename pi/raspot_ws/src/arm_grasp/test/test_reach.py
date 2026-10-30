# test/test_reach.py
"""`reach.py` 的纯函数测试：可行集、路径余量、字典序选解、肘支。"""
import math

import pytest

from arm_grasp import grasp, reach
from arm_grasp.arm_kin import fk, ikine
from arm_grasp.grasp import Refused


def _cfg(**kw):
    d = dict(alpha_lo=-88.0, alpha_hi=-45.0, alpha_span=14.0, alpha_step=1.0,
             alpha0=-58.0, min_slack=40.0, warn_slack=10.0, slack_tie=5.0,
             gripper=240.0, wrist_roll=496.0,
             vis=grasp.Vis(0.08 * 320, 0.92 * 320, 0.12 * 180, 0.86 * 180))
    d.update(kw)
    return grasp.GraspConfig(**d)


def _cand(alpha, min_slack, slack=None, joints=None):
    j = joints or dict(base=0.0, shoulder=0.0, elbow=0.0, wrist_pitch=0.0)
    return reach.Candidate(alpha=alpha, branch=-1, joints=j,
                           fields=[240, 496, 500, 500, 500, 0],
                           slack=slack if slack is not None else min_slack,
                           min_slack=min_slack)


def test_ikine_both_branches_roundtrip():
    """两支都要能 IK→FK 回到原点，且肘角符号相反。"""
    for branch in (-1, 1):
        j = ikine(17.0, 0.0, -12.0, -80.0, branch=branch)
        tip, _axis = fk(j)
        assert math.dist(tip, (17.0, 0.0, -12.0)) < 1e-9
        assert j['elbow'] * branch > 0.0


def test_enumerate_feasible_rejects_endpoint_ok_but_path_bad():
    """★ 旧 pick_target 只看末态：α=−60° 在 s=8cm 处合法，但下扎到更近处
    field 越界。新枚举必须把它排除，留下 −65…−61。"""
    cfg = _cfg()
    O = (0.10, -0.175, 0.016)          # 与 test_grasp.CAP 同一个几何点
    cands = reach.enumerate_feasible(O, cfg, 0.08)
    alphas = {c.alpha for c in cands}
    assert -60.0 not in alphas
    assert {-65.0, -64.0, -63.0, -62.0, -61.0} <= alphas
    assert all(c.min_slack >= 0.0 for c in cands)


def test_enumerate_feasible_default_alpha_seed_does_not_limit():
    """★ 桌面目标：默认 alpha0=−58 也必须能在全包络找到 −84 附近的解。
    这是“可达域偏小”里**软件窗**那一块的回归。"""
    cfg = _cfg()
    O = (0.17, 0.0, -0.123)            # 桌面高度附近（米）
    cands = reach.enumerate_feasible(O, cfg, 0.0)
    assert cands
    best = max(cands, key=lambda c: c.min_slack)
    assert best.alpha <= -80.0
    # 按旧的种子窗 [−72,−45] 是找不到的
    assert not reach.enumerate_feasible(O, cfg, 0.0, alpha_lo=-72.0, alpha_hi=-45.0)


def test_select_prefers_margin_then_continuity():
    """整段余量优先；并列档内选离上一拍 α 最近的。"""
    cfg = _cfg(slack_tie=5.0)
    a = _cand(-80.0, 80.0)
    b = _cand(-60.0, 77.0)
    c = _cand(-58.0, 76.0)
    assert reach.select([a, b, c], cfg, alpha_prev=-58.0) is c
    # b/c 都掉出并列档（< 80−5）⇒ 回到余量最大的 a
    b2 = _cand(-60.0, 70.0)
    c2 = _cand(-58.0, 70.0)
    assert reach.select([a, b2, c2], cfg, alpha_prev=-58.0) is a


def test_select_travel_breaks_equal_continuity():
    """α 距离相同（都离上一拍 2°）时，关节行程少的赢。"""
    cfg = _cfg(slack_tie=5.0)
    near = _cand(-58.0, 50.0, joints=dict(base=0.0, shoulder=0.0,
                                          elbow=0.0, wrist_pitch=0.0))
    far = _cand(-62.0, 50.0, joints=dict(base=0.0, shoulder=10.0,
                                         elbow=10.0, wrist_pitch=10.0))
    j_prev = dict(base=0.0, shoulder=0.0, elbow=0.0, wrist_pitch=0.0)
    assert reach.select([far, near], cfg, alpha_prev=-60.0,
                        j_prev=j_prev) is near


def test_lift_path_does_not_require_full_descent():
    """★ 2026-10-30 真机：抬起相只需要 s_stop→s_stop+lift 可行。

    这里的目标/α 在下扎整段路径上不可行，但在抬起这一段可行；如果 pick_target
    仍拿整段路径检查，就会把抬起解误拒成 REFUSED。
    """
    cfg = _cfg()._replace(lift_m=0.055)
    O = (0.15, 0.0, -0.04)
    s_m = cfg.s_stop_m + cfg.lift_m
    assert not reach.enumerate_feasible(O, cfg, s_m, alpha_lo=-85.0,
                                        alpha_hi=-85.0)
    lift = reach.path_samples(cfg, s_lo=cfg.s_stop_m, s_hi=s_m)
    cands = reach.enumerate_feasible(O, cfg, s_m, alpha_lo=-85.0,
                                     alpha_hi=-85.0, s_samples=lift)
    assert cands and cands[0].min_slack > 0.0


def test_feasible_intervals_merges_contiguous_alphas():
    assert reach.feasible_alpha_intervals(
        [_cand(-88, 1), _cand(-87, 1), _cand(-85, 1)]) == [[-88.0, -87.0],
                                                          [-85.0, -85.0]]


def test_pick_target_reports_searched_window_when_empty():
    cfg = _cfg()
    with pytest.raises(Refused) as e:
        grasp.pick_target((0.0, 0.0, 3.0), cfg, 0.0, -58.0, cfg.vis)
    assert '全包络' in str(e.value)
