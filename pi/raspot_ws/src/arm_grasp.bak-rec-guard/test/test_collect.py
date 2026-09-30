import json
import math
import os
import time

import pytest

from arm_grasp.arm_kin import fk, to_fields
from arm_grasp.collect import (plan_poses, pose_is_safe, pick_track, box_to_px,
                               read_result, k230_host_from_result)


def test_plan_poses_is_deterministic():
    a = plan_poses(8, seed=1)
    b = plan_poses(8, seed=1)
    assert a == b
    assert len(a) == 8
    assert len({tuple(sorted(p.items())) for p in a}) == 8      # 互不相同


def test_plan_poses_covers_the_workspace_for_every_seed():
    """★ 这条是 2026-09-28 变异自检/实测后加强的。

    计划原文只断言「base 跨度 > 8、shoulder 跨度 > 15」，而且只测 seed=1。
    实测：**纯拒绝采样下 300 个种子里有 68 个 shoulder 跨度 ≤ 15**
    （安全集只占关节箱 11%，且 shoulder 上限 105.9° 封顶）——
    也就是说计划那条测试是**靠运气过的**。

    这里改成对 **40 个种子**都断言，并且直接断言**任务空间**的覆盖
    （半径 / 高度 / 下扎角）—— 那才是决定 5 个参数可不可辨识的东西。
    """
    for seed in range(40):
        ps = plan_poses(8, seed=seed)
        assert len(ps) == 8, seed
        assert all(pose_is_safe(p) for p in ps), seed

        bases = [p['base'] for p in ps]
        assert max(bases) - min(bases) > 8.0, (seed, bases)
        sh = [p['shoulder'] for p in ps]
        assert max(sh) - min(sh) > 15.0, (seed, sh)

        # 任务空间覆盖
        sig = []
        for p in ps:
            tip, ax = fk(p)
            sig.append((math.hypot(tip[0], tip[1]), tip[2],
                        math.degrees(math.asin(max(-1.0, min(1.0, ax[2]))))))
        r = [s[0] for s in sig]
        z = [s[1] for s in sig]
        al = [s[2] for s in sig]
        assert max(r) - min(r) > 5.0, (seed, r)
        assert max(z) - min(z) > 2.0, (seed, z)
        assert max(al) - min(al) > 15.0, (seed, al)


def test_pose_is_safe_rejects_out_of_field():
    # ⚠️ 计划原文的"安全"例子是 shoulder=90,elbow=-45,wrist_pitch=-60 ——
    # 实测**它并不安全**（tip z = 15.03，远高于窗口上限 2）。
    # 换成实际验证过的：tip z = -4.13，fields 187.5/187.5/562.5 全在 125..875 内。
    assert pose_is_safe(dict(base=0, shoulder=75, elbow=-75, wrist_pitch=-75))
    assert not pose_is_safe(dict(base=0, shoulder=0, elbow=0, wrist_pitch=0))
    # ★ 上面那条负例是"靠 tip 高度拒绝的"，**测不出 field 那半条检查的死活**
    #   （2026-09-28 变异自检：把 field 检查改成恒不触发，测试照样全绿）。
    #   这条负例 field 越界 (p5=895.8 > 875) 而 tip 高度是合法的(z=-0.34)，
    #   只有 field 检查能拒它。
    assert not pose_is_safe(dict(base=0, shoulder=-5, elbow=0, wrist_pitch=0))


def test_pose_is_safe_rejects_tip_above_the_window():
    """field 都在范围内、但夹爪尖太高（够不到桌面上的标记点）也要拒。

    ⚠️ 计划那条用例 `shoulder=0,elbow=0,wrist=0` 会同时被 field 和 tip
    两条条件拒掉，所以它**分不出**是哪条在起作用。这条把 tip 那条单独钉住。
    """
    j = dict(base=0.0, shoulder=100.0, elbow=-20.0, wrist_pitch=-10.0)
    f = to_fields(j, 240.0, 496.0)
    # 前提：field 三条都是合法的，否则这条测试没在测它想测的东西
    assert all(125.0 <= v <= 875.0 for v in (f[2], f[3], f[4])), f
    tip, _ = fk(j)
    assert tip[2] > 2.0, tip
    assert not pose_is_safe(j)


def test_pick_track_prefers_track_and_tolerates_junk():
    objs = [{'cls': 'person', 'src': 'det', 'box': [0, 0, 0.1, 0.1]},
            {'cls': 'person', 'src': 'track', 'box': [0.2, 0.3, 0.4, 0.5]}]
    assert pick_track(objs)['box'] == [0.2, 0.3, 0.4, 0.5]
    assert pick_track([]) is None
    assert pick_track(None) is None
    assert pick_track([None, 'x', 3]) is None


def test_box_to_px_uses_the_stream_size():
    px = box_to_px([0.25, 0.5, 0.75, 0.75], 1280, 720)
    assert abs(px['u'] - 640.0) < 1e-9
    assert abs(px['v'] - 450.0) < 1e-9
    assert abs(px['w'] - 640.0) < 1e-9
    assert abs(px['h'] - 180.0) < 1e-9


def test_read_result_reports_missing_file_in_plain_words(tmp_path):
    with pytest.raises(RuntimeError) as e:
        read_result(str(tmp_path / 'nope.json'))
    assert '没在推结果' in str(e.value)


def test_read_result_reports_stale_stream(tmp_path):
    p = tmp_path / 'r.json'
    p.write_text('{"w":1280,"h":720,"objs":[]}', encoding='utf-8')
    old = time.time() - 60
    os.utime(str(p), (old, old))
    with pytest.raises(RuntimeError) as e:
        read_result(str(p), max_age=1.5)
    assert '结果流断了' in str(e.value)


def test_read_result_accepts_a_fresh_file(tmp_path):
    p = tmp_path / 'r.json'
    p.write_text('{"w":1280,"h":720,"objs":[]}', encoding='utf-8')
    assert read_result(str(p))['w'] == 1280


def test_k230_host_from_result_strips_the_port(tmp_path):
    p = tmp_path / 'r.json'
    p.write_text(json.dumps({'_peer': '192.168.5.237:64919'}), encoding='utf-8')
    assert k230_host_from_result(str(p)) == '192.168.5.237'


# ---------------------------------------------------------------------------
# blue_blob_px：用**合成图**做正/负样本，这条必须能红
# ---------------------------------------------------------------------------

def _synth(tmp_path, name, rgb, radius=70, bg=(90, 80, 70)):
    """造一张 1280x720 的图：背景 + 一个圆。返回路径。"""
    from PIL import Image, ImageDraw
    im = Image.new('RGB', (1280, 720), bg)
    d = ImageDraw.Draw(im)
    cx, cy = 640, 360
    d.ellipse([cx - radius, cy - radius, cx + radius, cy + radius], fill=rgb)
    p = str(tmp_path / name)
    im.save(p, quality=90)
    return p


def test_blue_blob_finds_a_saturated_blue_disc(tmp_path):
    """正样本：饱和蓝的圆盘必须被找到，且质心对得上。"""
    from arm_grasp.collect import blue_blob_px
    p = _synth(tmp_path, 'cap.jpg', (37, 133, 255))
    b = blue_blob_px(p)
    assert b is not None, '饱和蓝的圆盘没被找到'
    assert abs(b[0] - 640) < 6 and abs(b[1] - 360) < 6, b
    assert b[2] > 10000, b


def test_blue_blob_rejects_pale_bluish_white(tmp_path):
    """★ 负样本：蓝白色的大块**不能**被当成瓶盖。

    这是 2026-09-28 抓到的真实假阳性：相机扫到床单/桌面（R=169 G=224 B=252），
    旧判据（只看 B 比 R/G 大）报了 26904 px "找到瓶盖"。
    实测瓶盖本体是 R=37 G=133 B=255 —— 靠**饱和度**（B-R 和 B-G 都要够大）分开。
    """
    from arm_grasp.collect import blue_blob_px
    p = _synth(tmp_path, 'bed.jpg', (169, 224, 252))
    assert blue_blob_px(p) is None, '蓝白色被误判成瓶盖'


def test_blue_blob_rejects_a_small_speck(tmp_path):
    """太小的蓝点不算（噪声/反光），要有面积下限。"""
    from arm_grasp.collect import blue_blob_px
    p = _synth(tmp_path, 'speck.jpg', (37, 133, 255), radius=5)
    assert blue_blob_px(p) is None, '小蓝点被当成了瓶盖'


def test_blue_blob_rejects_a_plain_background(tmp_path):
    """完全没有蓝的图必须是 None。"""
    from arm_grasp.collect import blue_blob_px
    assert blue_blob_px(_synth(tmp_path, 'plain.jpg', (37, 133, 255),
                               radius=0)) is None


# ---------------------------------------------------------------------------
# plan_poses_aimed：按"相机真能看到瓶盖"排姿态
# ---------------------------------------------------------------------------

CAP = (17.0, 0.0, -12.3)


def _alpha_of(j):
    _, ax = fk(j)
    return math.degrees(math.asin(max(-1.0, min(1.0, ax[2]))))


def test_plan_poses_aimed_keeps_alpha_in_the_verified_band():
    """★ α 必须落在"相机真的看得见桌面"的区间里。

    实测锚点：α=-79.2° 能看到瓶盖；α=-58° 看不到（画面是抽屉壁）。
    所以 α 的上限要卡在 -70°，不能是 IK 可达的 -20°。
    """
    from arm_grasp.collect import plan_poses_aimed
    for seed in range(6):
        for j in plan_poses_aimed(6, CAP, seed=seed):
            assert pose_is_safe(j)
            al = _alpha_of(j)
            assert -88.5 <= al <= -70.0, (seed, al)


def test_plan_poses_aimed_follows_the_measured_aim_relation():
    """瞄准关系：alpha ≈ 仰角(夹爪尖->瓶盖) - 18.4°（18.4 是实测锚点）。

    ⚠️ 这里的 18.4 **写死**，不从模块 import —— 变异自检抓过这个：
    原来 `from ... import AIM_OFFSET_DEG` 拿它当期望值，于是把常量改成 0
    时期望值跟着变成 0，测试**永远绿**。期望值必须是独立于实现的。
    """
    from arm_grasp.collect import plan_poses_aimed
    for j in plan_poses_aimed(6, CAP, seed=1):
        tip, _ = fk(j)
        d = math.dist(tip, CAP)
        elev = math.degrees(math.asin((CAP[2] - tip[2]) / d))
        assert abs(_alpha_of(j) - (elev - 18.4)) < 0.6, j


def _min_path_dist(ja, jb, cap, steps=24):
    """两个姿态之间（夹爪尖直线插值）离 cap 的最近距离。**测试自己算**，
    不调用被测的那个 helper —— 否则就是拿实现当判据（变异测不出来）。"""
    ta, _ = fk(ja)
    tb, _ = fk(jb)
    best = float('inf')
    for i in range(steps + 1):
        f = i / float(steps)
        p = tuple(ta[k] + f * (tb[k] - ta[k]) for k in range(3))
        best = min(best, math.dist(p, cap))
    return best


def test_plan_poses_aimed_stays_clear_of_the_cap():
    """夹爪尖离瓶盖要有余量，**相邻姿态之间走过的路**也要有余量。"""
    from arm_grasp.collect import plan_poses_aimed
    for seed in range(6):
        ps = plan_poses_aimed(6, CAP, seed=seed)
        for j in ps:
            tip, _ = fk(j)
            assert math.dist(tip, CAP) >= 6.0, seed
        for a in range(len(ps)):
            assert _min_path_dist(ps[a], ps[(a + 1) % len(ps)], CAP) >= 3.0, seed


def test_path_clearance_helper_flags_a_path_through_the_cap():
    """路径**真的穿过**瓶盖时必须判 False。

    这里直接给**夹爪尖的两个点**（纯几何），不走 IK —— 因为当前姿态箱里
    夹爪尖始终比瓶盖顶高 5.8cm 以上，`_path_to_cap_ok` 实际上永远不触发；
    要让这段防御代码有非空语义，只能单独把几何那层拎出来测。
    """
    from arm_grasp.collect import _path_clear_of
    # 同一个高度两侧走，直线正穿瓶盖
    assert not _path_clear_of((11.0, 0.0, -12.3), (23.0, 0.0, -12.3), CAP, 3.0)
    # 抬高到瓶盖上方 8cm 就该放行
    assert _path_clear_of((11.0, 0.0, -4.0), (23.0, 0.0, -4.0), CAP, 3.0)
    # 端点就贴着瓶盖也不放行
    assert not _path_clear_of(CAP, (23.0, 0.0, -4.0), CAP, 3.0)


def test_plan_poses_aimed_is_deterministic_and_spread():
    from arm_grasp.collect import plan_poses_aimed
    a = plan_poses_aimed(6, CAP, seed=1)
    b = plan_poses_aimed(6, CAP, seed=1)
    assert a == b
    bases = [p['base'] for p in a]
    assert max(bases) - min(bases) > 15.0, bases      # 横向要扫开
    als = [_alpha_of(p) for p in a]
    assert max(als) - min(als) > 8.0, als             # 俯角也要有变化


def test_plan_poses_aimed_gives_up_on_a_cap_it_cannot_see():
    """瓶盖远到够不着时必须报错，而不是硬给一堆看不见它的姿态。"""
    from arm_grasp.collect import plan_poses_aimed
    with pytest.raises(RuntimeError):
        plan_poses_aimed(6, (500.0, 0.0, -12.3))
