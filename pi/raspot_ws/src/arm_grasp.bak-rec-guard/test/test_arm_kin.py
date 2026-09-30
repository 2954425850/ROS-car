import math

import pytest

from arm_grasp.arm_kin import (L1, L2, L3, L4, FIELD_LO, FIELD_HI, Unreachable,
                               ikine, fk, find_grasp_solution, to_fields,
                               from_fields)


def test_ik_fk_roundtrip_is_exact():
    """IK 出来的关节角，喂回 FK 必须回到原 (x,y,z) 和 alpha。"""
    worst = 0.0
    for x in (12, 14, 16, 18):
        for y in (-5, 0, 5):
            for z in (-6, -4, -2, 0):
                for al in (-80, -65, -50):
                    try:
                        j = ikine(x, y, z, al)
                    except Unreachable:
                        continue
                    tip, axis = fk(j)
                    worst = max(worst, math.dist(tip, (x, y, z)))
    assert worst < 1e-9, 'IK/FK 往返误差 %.3e cm' % worst


def test_factory_pose_decodes_to_known_grasp_point():
    """出厂姿态回读值 -> 末端应落在 (前方14.60, 高1.33)，alpha=-55.44。

    ⚠️ 2026-09-28 执行计划时修正（与 design §4.7 不一致，见报告）：
    design §4.7 写着这组 field 解码成 肩111.1/肘-91.0/腕-77.3 -> (14.21, 0.82)。
    但用同一份 §4.5 的映射（已对着幻尔官方源码逐字核对：
    `robot_arm.c: 500 + SERIAL_ANGLE_FACTOR*target_angle`，
    `robot_arm.h: SERIAL_ANGLE_FACTOR = 4.166666666666667 = 1000/240`，
    `kinematics.h: LINKAGE 2.89/10.43/8.9/17.7`）解出来是
    肩110.88/肘-89.28/腕-77.04 -> (14.598, 1.335)，alpha=-55.44。
    两组角度各自的 3D 落点都自洽，差别只在 §4.7 那一步解码。以源码为准。

    下面这两个数是**独立算出来的**（纯几何手算，不经 arm_kin.py），
    不是跑一遍代码把输出抄回来 —— 否则这个测试就只是在复读实现。
    """
    fields = [240, 496, 179, 128, 413, 6]     # 2026-09-28 实机 /arm/feedback
    j = from_fields(fields)
    tip, axis = fk(j)
    assert abs(math.hypot(tip[0], tip[1]) - 14.598) < 0.02, tip
    assert abs(tip[2] - 1.335) < 0.02, tip
    # alpha 也要钉住：它是"斜着扎"这个结论的全部依据
    alpha = math.degrees(math.asin(max(-1.0, min(1.0, axis[2]))))
    assert abs(alpha - (-55.44)) < 0.05, alpha


def test_base_scale_is_1000_over_360():
    """底座用本固件的比例尺，不是官方的 1000/240。已物理实测。"""
    j = dict(base=90.0, shoulder=90.0, elbow=0.0, wrist_pitch=0.0)
    assert abs(to_fields(j, 240, 496)[5] - 250.0) < 1e-9


def test_arm_joint_scale_is_1000_over_240():
    """肩/肘/腕用官方比例尺 1000/240（对着官方源码核对过）。"""
    j = dict(base=0.0, shoulder=0.0, elbow=90.0, wrist_pitch=-90.0)
    f = to_fields(j, 240, 496)
    assert abs(f[3] - (500 + 90 * 1000.0 / 240.0)) < 1e-9   # 肘 -> p4
    assert abs(f[2] - (500 - 90 * 1000.0 / 240.0)) < 1e-9   # 腕 -> p3
    assert abs(f[4] - (500 + 90 * 1000.0 / 240.0)) < 1e-9   # 肩(90-0) -> p5


def test_unreachable_raises():
    with pytest.raises(Unreachable):
        ikine(100.0, 0.0, 0.0, -90.0)


def test_from_fields_roundtrip():
    j = dict(base=12.0, shoulder=100.0, elbow=-60.0, wrist_pitch=-70.0)
    j2 = from_fields(to_fields(j, 240, 496))
    for k in j:
        assert abs(j[k] - j2[k]) < 1e-6, k


def test_base_yaw_follows_atan2_y_x():
    """底座 = atan2(y, x)，正=左（+y），与固件约定一致（已物理实测）。"""
    j = ikine(15.0, 15.0, -12.0, -60.0)
    assert abs(j['base'] - 45.0) < 1e-9, j
    j2 = ikine(15.0, -15.0, -12.0, -60.0)
    assert abs(j2['base'] - (-45.0)) < 1e-9, j2


def test_find_grasp_solution_rejects_field_illegal_alphas():
    """扫 alpha 时必须排除掉会撞固件钳位的解，并继续往后找。

    ★ 这条测试的第一版是**空的**，2026-09-28 变异自检抓住的：
      原来只查 (16,0,-12)，而那个点第一个 alpha 本身就合法，
      于是把 field 过滤改成恒真、测试照样全绿。现在改成两个
      "第一个 IK 解就是非法 field"的目标：
      (16,-6) free 解 p3=104、(18,-8) free 解 p3=92，都低于 FIELD_LO=125。
      过滤后必须跳过它们、走到 alpha=-87 的合法解。
    """
    cases = [(16.0, -6.0), (18.0, -8.0)]
    for x, z in cases:
        a_free, j_free = find_grasp_solution(x, 0.0, z, field_ok=False)
        f_free = to_fields(j_free, 240.0, 496.0)
        assert not all(FIELD_LO <= v <= FIELD_HI
                       for v in (f_free[2], f_free[3], f_free[4])), \
            '选点错了：这个目标的第一解本来就合法，测试点选错'
        a, j = find_grasp_solution(x, 0.0, z)
        assert a > a_free, (a, a_free)          # 非法解必须被跳过
        assert abs(a - (-87.0)) < 1e-9, a       # 且落在预期的合法解上
        f = to_fields(j, 240.0, 496.0)
        for v in (f[2], f[3], f[4]):
            assert FIELD_LO <= v <= FIELD_HI, f
        tip, _ = fk(j)
        assert math.dist(tip, (x, 0.0, z)) < 1e-9, tip


def test_find_grasp_solution_gives_up_when_nothing_is_legal():
    """真有目标一个合法 alpha 都没有时，必须抛 Unreachable 而不是硬给一个。"""
    with pytest.raises(Unreachable):
        find_grasp_solution(10.0, 0.0, -12.0)
