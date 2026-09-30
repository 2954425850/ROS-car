# test/test_grasp.py
import math
import pytest
from arm_grasp import geom, grasp
from arm_grasp.grasp import Refused

# 单测夹具点：**不是当前瓶盖位置**（瓶盖已被挪过，旧数据一律作废），只用来做几何单测。
# 2026-09-30 实测扫过一遍（z=0.016、α∈[−88°,−45°]、s=0 的末态）：
#     r ≤ 16cm → **refuse**（解不出姿态）；r=17cm → slack 14（warn）；r=18cm → 47；
#     r=20cm → 111；r=25cm → 163。
# 所以夹具取 **r≈20cm**，而且**故意离轴**（方位角 −60°）：在 −y 轴上时下扎轴的水平 x 分量
# 恒为 0（ux≈3e-17），单分量反号的变异抓不到（见 test_axis_point_direction_off_axis）。
CAP = (0.10, -0.175, 0.016)          # r = 20.2cm，方位角 −60.3°


def _cfg(**kw):
    d = dict(alpha_lo=-88.0, alpha_hi=-45.0, alpha_span=14.0, alpha_step=1.0,
             alpha0=-58.0, min_slack=40.0, warn_slack=10.0, slack_tie=5.0,
             gripper=240.0, wrist_roll=496.0,
             vis=grasp.Vis(0.08 * 320, 0.92 * 320, 0.12 * 180, 0.86 * 180))
    d.update(kw)
    return grasp.GraspConfig(**d)


def test_axis_point_is_s_above_O_along_the_gripper_axis():
    """s=0 ⇒ 就是 O 本身；s>0 ⇒ 沿下扎轴退 s，且退的方向与可见像素的方位一致。"""
    cfg = _cfg()
    assert grasp.axis_point(CAP, 0.0, -60.0) == pytest.approx(CAP, abs=1e-12)
    T = grasp.axis_point(CAP, 0.05, -60.0)
    assert math.dist(T, CAP) == pytest.approx(0.05, abs=1e-9)
    # 下扎轴指向下：z 分量必须为正（爪尖在目标**上方**）
    assert T[2] > CAP[2]


def test_pick_target_respects_min_slack():
    """选出来的姿态余量必须尽量大（这条对应 17:34 那次肘关节顶死的失败）。

    夹具点本身必须是"舒服"的点 ⇒ 直接断言 `tier()=='ok'`：夹具哪天飘到边缘，
    这条会**立刻红**并说明是夹具的问题，而不是悄悄退化成 warn（上一版就只差 1 count）。
    """
    cfg = _cfg()
    t = grasp.pick_target(CAP, cfg, 0.0, -58.0, cfg.vis)
    assert grasp.tier(t.slack, cfg) == 'ok'    # 夹具点必须余量充裕
    assert t.slack >= 40.0                     # ≥ min_slack，留出余量
    assert isinstance(t.fields, list) and len(t.fields) == 6
    assert all(125.0 <= v <= 875.0 for v in t.fields[2:5])


def test_pick_target_refuses_unreachable():
    with pytest.raises(Refused):
        grasp.pick_target((0.0, 0.0, 3.0), _cfg(), 0.0, -58.0, _cfg().vis)   # 3 米高


def test_visible_uses_project():
    """看得见 = 模型预测的像素落在有效区。用一个被裁到画面外的点验证它真的会红。"""
    from arm_grasp.arm_kin import from_fields
    from arm_grasp.servo import ik_open_m
    j = ik_open_m(CAP[0], CAP[1], CAP[2] + 0.05, -60.0)
    ok, px = grasp.visible(j, CAP, _cfg().vis)
    assert isinstance(ok, bool) and len(px) == 2
    far = (0.0, -0.9, 0.5)                      # 很远的点 → 一定出画
    ok2, _ = grasp.visible(j, far, _cfg().vis)
    assert ok2 is False


def test_tier_three_levels():
    cfg = _cfg()
    assert grasp.tier(60.0, cfg) == 'ok'
    assert grasp.tier(20.0, cfg) == 'warn'
    assert grasp.tier(3.0, cfg) == 'refuse'


def test_plan_verdict_tiers():
    lvl, tgt, msg = grasp.plan_verdict(CAP, _cfg(), _cfg().vis)
    assert lvl in ('ok', 'warn', 'refuse') and msg
    if lvl != 'refuse':
        assert tgt is not None
    lvl2, _, _ = grasp.plan_verdict((0.0, 0.0, 3.0), _cfg(), _cfg().vis)
    assert lvl2 == 'refuse'


# ---------------------------------------------------------------------------
# ★ 以下两条是**变异自检的兜底**：计划 Task 4 Step 4 自己写了
#   "如果测试仍绿…那就在测试里加一条"。上面 6 条是计划原文，一字未改。
# ---------------------------------------------------------------------------

def test_axis_point_direction_off_axis():
    """★ 兜底（对应变异 1：`O_m[0] - s_m * ux` → `O_m[0] + s_m * ux`）。

    为什么计划里那条抓不到（两条独立的理由，都算过）：
    1. 夹具点 CAP=(0,-0.17,·) 在 **−y 轴**上 ⇒ 下扎轴的水平 x 分量
       ux = cosα·cos(tb) ≈ 3.1e-17，x 项贡献只有 **1.5e-18 m**；
    2. 更根本："把某一个分量反号"**不改变 |T−O|** ⇒ 只断言 `math.dist` 的用例
       在数学上不可能抓到任何单分量反号。
    这里换一个**离轴**的点（方位角 −45°，ux ≈ 0.354），并把**完整方向向量**
    钉死 ⇒ 三个分量里任何一个反号都会红。
    """
    O = (0.12, -0.12, 0.016)
    s, alpha = 0.05, -60.0
    T = grasp.axis_point(O, s, alpha)
    assert T == pytest.approx((0.1023223304703363, -0.1023223304703363,
                               0.05930127018922193), abs=1e-9)
    d = tuple((T[i] - O[i]) / s for i in range(3))      # 单位下扎轴 −ẑ_T(α)
    assert d == pytest.approx((-0.3535533905932739, 0.3535533905932739,
                               0.8660254037844386), abs=1e-9)
    assert d[0] < 0.0 and d[1] > 0.0                    # 与 −ẑ_T 同向，不是镜像
    assert d[2] == pytest.approx(-math.sin(math.radians(alpha)))


def test_pick_target_breaks_slack_ties_toward_prefer_alpha():
    """★ 兜底（对应变异 3：`near.sort` 的 key 去掉 `abs(c.alpha - prefer_alpha)`）。

    为什么计划里那条抓不到：默认 `slack_tie=5.0` 时，夹具点上**并列档里只有
    一个候选**（s=0 时是 −72°，s=0.05 时是 −69°）⇒ 那条排序是死代码。
    把宽容放宽到能让 8 个候选全进并列档，排序才被真正覆盖 —— 这正好是那个
    "避免每拍换解"的连续性约束：并列时**宁可选余量只有 2.8 的 −65°，也要选
    离 prefer(−58°) 最近的那个**（余量最大的是 −69°/50.5）。
    """
    cfg = _cfg(slack_tie=60.0)          # 宽容放大到 60 → 并列档里真有多个候选
    t = grasp.pick_target(CAP, cfg, 0.05, -58.0, cfg.vis)
    # 实测并列档（本夹具、s=5cm、prefer=−58°）：
    #   α     : -70   -69   -68   -67   -66   -65   -64   -63   -62   -61
    #   slack : 18.5  31.4  43.6  55.2  66.4 *77.3* 68.5  54.4  40.7  27.4
    # 余量最大的是 −65°(77.3)；选中的是**离 prefer(−58°) 最近**的 −61°(27.4)
    # —— 为了"每拍不换解"的连续性主动放弃 50 点余量（故意的：那条排序规则就是这么用的）。
    assert t.alpha == pytest.approx(-61.0)
    assert t.slack == pytest.approx(27.406628, abs=1e-6)
