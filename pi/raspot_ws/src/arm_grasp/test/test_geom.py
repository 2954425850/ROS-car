# -*- coding: utf-8 -*-
"""geom（标定后的纯几何链路）的测试。

## 这套测试里哪条才算证据
* `test_pixel_to_target_recovers_a_known_point` 是**闭环**：用 project() 生成、
  又用反解收回来。**它只能证明代数自洽，证明不了几何正确** —— 这个项目在这上面
  栽过（`calib_solve` 的合成自检 px_rms=0 却拟合不上真实数据）。
* 真正算数的是**开环**那两条：把尺子量出来的常量（手眼 4 个数、‖手眼‖=12.4cm）
  跟标定 + 推导算出来的结果对。
"""
import math

import pytest

from arm_grasp import geom
from arm_grasp.arm_kin import fk
from arm_grasp.cam_model import tool_axes

POSE = dict(base=6.0, shoulder=110.88, elbow=-89.28, wrist_pitch=-77.04)


# ---------------- 开环：拿尺子量的常量验标定 + 推导 ----------------

def test_aim_pixel_is_the_documented_constant():
    """瞄准像素必须等于推导文档 §5.1 钉死的字面值。

    ★ 这条把字面值**钉死**：改了手眼常数或内参却忘了同步推导文档，它会红。
      用的是尺子量的 (0.0075, 0.0340, 0.1190) 和标定的 K，两边都是外部量。
    """
    # ★ 相机是**倒装**的（sigma = -1）：原始帧里夹爪在画面**底部**。
    #   2026-09-29 实拍核对：夹爪压在瓶盖上、位于 (约 550~650, 620~700) px(1280x720)。
    #   转正后的图上它才回到顶部 —— 设计文档记的 (0.55, 0.03) 是**转正后**的数，
    #   拿它当原始帧的数用会把 sigma 判反。
    u, v = geom.aim_pixel(closed=False)
    assert abs(u - 132.546) < 0.02, (u, v)
    assert abs(v - 177.450) < 0.02, (u, v)
    u2, v2 = geom.aim_pixel(closed=True)
    assert abs(u2 - 134.782) < 0.02 and abs(v2 - 167.236) < 0.02, (u2, v2)
    # 去畸变后的针孔值也要对得上
    uu, vv = geom.undistort(u, v)
    assert abs(uu - 132.741) < 0.02 and abs(vv - 176.094) < 0.02, (uu, vv)
    # 转正（180 度旋转）之后应回到"顶部略偏右"，和设计文档对得上
    assert abs((1 - u / 320) - 0.5858) < 1e-3, (u,)
    assert abs((1 - v / 180) - 0.0142) < 1e-3, (v,)


def test_depth_at_grasp_equals_the_handeye_norm():
    """爪尖贴上目标那一刻，算出的**欧氏**深度必须等于 ‖手眼平移‖ = 12.40 cm。

    ★ 开环：右端是**尺子量的常量**（0.0075/0.0340/0.1190 的模长），
      左端要走过 项目→去畸变→射线→平面交点 整条链。任何一环（含 SIGMA 符号、
      工具系定义、内参）错了都会红。这是本文件里最硬的一条。
    """
    assert abs(geom.handeye_range(False) - 0.1240) < 5e-4
    tip = geom.gripper_tip(POSE, closed=False)
    C = geom.camera_center(POSE)
    assert abs(math.dist(C, tip) - geom.handeye_range(False)) < 1e-12
    u, v = geom.project(POSE, tip)
    P, Z_axial, D = geom.target_from_pixel(POSE, u, v, tip[2])
    assert abs(D - geom.handeye_range(False)) < 1e-6, (D, geom.handeye_range(False))
    assert math.dist(P, tip) < 1e-6, (P, tip)
    assert 0.0 < Z_axial < D          # 轴向深度必然小于欧氏距离


def test_camera_center_is_independent_of_the_gripper_state():
    """相机挂在**腕**上 ⇒ 张开/夹紧两种读法必须算出同一点。

    ★ 这条钉的是那个 1.7cm 静默误差：拿 fk(L4=17.7) 的爪尖去配 CAM_OPEN 就会偏 1.7cm。
    """
    tip_cm, axis = fk(POSE)
    xh, yh, zh = tool_axes(tip_cm, axis)

    def center(tip_cm_, off):
        return tuple(tip_cm_[i] / 100.0 + off[0] * xh[i] + off[1] * yh[i]
                     + off[2] * zh[i] for i in range(3))

    back_cm = geom.L4_CLOSED - geom.L4_OPEN
    tip_open_cm = tuple(tip_cm[i] - back_cm * zh[i] for i in range(3))
    a = center(tip_cm, geom.CAM_CLOSED)
    b = center(tip_open_cm, geom.CAM_OPEN)
    assert math.dist(a, b) < 1e-12, (a, b)
    assert math.dist(a, geom.camera_center(POSE)) < 1e-12


def test_aim_pixel_does_not_depend_on_the_pose():
    """★ 核心结论：瞄准像素是**常数**，与臂什么姿态、目标在哪、离多远都无关。

    这条直接钉住「为什么不该去量一个抓取像素」—— 它是推出来的，不是独立信息。
    """
    ua, va = geom.aim_pixel(closed=False)
    for pose in (POSE,
                 dict(base=30.0, shoulder=70.0, elbow=-60.0, wrist_pitch=-40.0),
                 dict(base=-20.0, shoulder=95.0, elbow=-85.0, wrist_pitch=-80.0),
                 dict(base=0.0, shoulder=45.0, elbow=-70.0, wrist_pitch=-60.0)):
        tip = geom.gripper_tip(pose, closed=False)
        u, v = geom.project(pose, tip)
        assert abs(u - ua) < 0.02 and abs(v - va) < 0.02, (pose, (u, v), (ua, va))


def test_driving_the_target_to_the_image_centre_would_miss():
    """拿画面中心当瞄准点的横竖偏差必须落在文档记的量级（7.5mm / 34mm）。"""
    ua, va = geom.aim_pixel(closed=False)
    k = geom.K_AI320x180_CHN2
    Z = abs(geom.CAM_OPEN[2])      # 抓取那一刻的轴向深度 = 0.1190 m
    du_mm = (ua - k.cx) / k.fx * Z * 1000.0
    dv_mm = (va - k.cy) / k.fy * Z * 1000.0
    assert 6.0 < abs(du_mm) < 9.0, du_mm
    assert 32.0 < abs(dv_mm) < 36.0, dv_mm


# ---------------- 闭环 / 数值 ----------------

def test_pixel_to_target_recovers_a_known_point():
    """合成往返。⚠️ **闭环**：project() 生成、反解收回，只能证明代数自洽，
    不构成几何正确的证据（真正的证据在 test_depth_at_grasp_equals_the_handeye_norm）。"""
    Z = geom.CAP_HEIGHT
    for P in ((0.10, -0.04, Z), (0.13, 0.02, Z), (0.09, 0.06, Z)):
        u, v = geom.project(POSE, P)
        P2, _, _ = geom.target_from_pixel(POSE, u, v, Z)
        assert math.dist(P, P2) < 1e-6, (P, P2)


def test_undistort_inverts_distort():
    for u, v in ((10.0, 10.0), (310.0, 170.0), (160.0, 90.0), (40.0, 20.0),
                 (300.0, 12.0), (5.0, 175.0)):
        uu, vv = geom.undistort(u, v)
        u2, v2 = geom.distort(uu, vv)
        assert math.hypot(u2 - u, v2 - v) < 1e-6, (u, v, u2, v2)


def test_ray_that_is_too_shallow_is_refused():
    """射线太平必须**报错**，不能返回垃圾深度 —— 「自洽但错」就是从这儿进来的。"""
    flat = dict(base=0.0, shoulder=20.0, elbow=0.0, wrist_pitch=-20.0)   # alpha = 0
    with pytest.raises(ValueError):
        geom.target_from_pixel(flat, 160.0, 90.0, geom.CAP_HEIGHT)


def test_target_behind_the_camera_is_refused():
    """目标平面在相机背后（相机已经比目标低了）必须报错。"""
    with pytest.raises(ValueError):
        geom.target_from_pixel(POSE, 160.0, 90.0, 99.0)


def test_pixel_ray_direction_is_unit_and_points_forward():
    for u, v in ((160.0, 90.0), (40.0, 30.0), (280.0, 150.0)):
        C, d = geom.pixel_ray(POSE, u, v)
        assert abs(math.sqrt(sum(c * c for c in d)) - 1.0) < 1e-12
        _, axis = fk(POSE)
        assert sum(d[i] * axis[i] for i in range(3)) > 0.0     # 朝夹爪下扎方向


def test_intrinsics_are_square_pixel():
    """320x180 是根分辨率的**均匀 ÷4**（不是压扁）⇒ fx/fy 必须 ≈1。
    这条是「标错通道」的哨兵：真去标了 320x320（压扁）的话它会明显偏。"""
    k = geom.K_AI320x180_CHN2
    assert abs(k.fx / k.fy - 1.0) < 0.02, k.fx / k.fy


def test_stream_and_ai_frames_are_a_uniform_div4():
    """1280x720(检测/拍服用) 与 320x180(标定用) 必须是**均匀 ÷4**：
    归一化坐标跨分辨率不变，所以标定的 K 可以直接用在检测框上。

    ★ 这条是「标定能不能用在检测框上」的哨兵：本平台配成 320x320 时是**压扁**
      （纵向拉长 1.78x），那种情况下除法换算不成立，这条会红。
    """
    k = geom.K_AI320x180_CHN2
    assert geom.STREAM_W / k.w == geom.STREAM_H / k.h, '两个方向缩放不一致 = 压扁'
    for u, v in ((167.87, 17.87), (10.0, 170.0), (310.0, 5.0)):
        su, sv = geom.to_stream(u, v)
        assert abs(su / geom.STREAM_W - u / k.w) < 1e-12
        assert abs(sv / geom.STREAM_H - v / k.h) < 1e-12
        bu, bv = geom.to_ai(su, sv)
        assert abs(bu - u) < 1e-9 and abs(bv - v) < 1e-9
