import math

import pytest

from arm_grasp.cam_model import (CamParams, camera_pose, project, pixel_ray,
                                 ray_plane)

P = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0, roll=0.0,
              u0=640.0, v0=360.0)


def _axis(deg):
    return (math.cos(math.radians(deg)), 0.0, math.sin(math.radians(deg)))


def test_tip_projects_to_grasp_pixel():
    """定义式自洽：夹爪尖必须落在 (u_star, v_star)。"""
    tip = (14.0, 0.0, 0.8)
    axis = _axis(-57.0)
    u, v = project(P, tip, tip, axis)
    assert abs(u - P.u_star) < 1e-9 and abs(v - P.v_star) < 1e-9


def test_pixel_ray_is_inverse_of_project():
    """投一个点出去，再用像素拉一条射线回来，点必须在这条射线上。"""
    tip = (14.0, 0.0, 0.8)
    axis = _axis(-57.0)
    for pt in [(16.0, 0.0, -12.0), (13.5, 2.0, -12.5), (17.0, -3.0, -11.0)]:
        u, v = project(P, pt, tip, axis)
        C, d = pixel_ray(P, u, v, tip, axis)
        rel = [pt[i] - C[i] for i in range(3)]
        cross = [rel[1] * d[2] - rel[2] * d[1],
                 rel[2] * d[0] - rel[0] * d[2],
                 rel[0] * d[1] - rel[1] * d[0]]
        assert math.dist(cross, (0, 0, 0)) < 1e-9


def test_ray_plane_hits_the_original_point():
    tip = (14.0, 0.0, 0.8)
    axis = _axis(-57.0)
    for pt in [(16.0, 0.0, -12.0), (13.5, 2.0, -12.5)]:
        u, v = project(P, pt, tip, axis)
        C, d = pixel_ray(P, u, v, tip, axis)
        hit = ray_plane(C, d, pt[2])
        assert math.dist(hit, pt) < 1e-9


def test_roll_zero_means_image_up_is_world_up():
    """roll=0 时，世界竖直方向的位移应该只改变 v。"""
    tip = (14.0, 0.0, 0.8)
    axis = (0.0, 0.0, -1.0)                 # 正下扎，方便判断
    u1, v1 = project(P, (14.0, 0.0, -12.0), tip, axis)
    u2, v2 = project(P, (14.0, 1.0, -12.0), tip, axis)
    assert abs(v2 - v1) < 1e-9              # 左右移动不改 v
    assert abs(u2 - u1) > 1.0               # 但改 u


def test_roll_rotates_the_image_by_minus_roll():
    """roll 是「图像绕光轴转」的角，转的是**图像坐标系**。

    ★ 这条是 2026-09-28 补的：计划 Task 3 Step 5 的变异表里有一行
      「x/y 定义里 sr 符号同时取反 -> test_roll_zero_means_image_up_is_world_up」，
      但那个用例的 roll=0（sr=0），符号取反对它**没有任何影响** —— 抓不住。
      这里用 roll≠0、且令 u*=u0 / v*=v0（此时相机中心 C 与 roll 无关，
      相机退化成纯绕轴旋转）把符号钉死：
      图像坐标系转 +roll，同一个世界点在画面里的方位角必须**减少** roll。
      这是纯几何、不依赖实现的推导。
    """
    tip = (14.0, 0.0, 0.8)
    axis = (0.0, 0.0, -1.0)
    pt = (14.0, 3.0, -12.0)                 # 横向偏离光轴，方位角才有定义
    p0 = CamParams(f=1100.0, t=120.0, u_star=640.0, v_star=360.0, roll=0.0)
    u0_, v0_ = project(p0, pt, tip, axis)
    a0 = math.atan2(v0_ - 360.0, u0_ - 640.0)
    for roll in (0.3, -0.7, 1.1):
        p = CamParams(f=1100.0, t=120.0, u_star=640.0, v_star=360.0, roll=roll)
        u_, v_ = project(p, pt, tip, axis)
        a1 = math.atan2(v_ - 360.0, u_ - 640.0)
        d = (a0 - a1 + math.pi) % (2 * math.pi) - math.pi
        assert abs(d - roll) < 1e-12, (roll, d)


def test_ray_plane_rejects_backwards():
    with pytest.raises(ValueError):
        ray_plane((0.0, 0.0, 5.0), (0.0, 0.0, 1.0), 0.0)


def test_vector_roundtrip():
    assert abs(P.f - CamParams.from_vector(P.as_vector()).f) < 1e-9


def test_vector_roundtrip_with_principal():
    q = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0, roll=0.03,
                  u0=600.0, v0=380.0)
    r = CamParams.from_vector(q.as_vector(fit_principal=True), fit_principal=True)
    assert (r.u0, r.v0) == (600.0, 380.0)
    assert len(q.as_vector()) == 5 and len(q.as_vector(True)) == 7


def test_pixel_ray_direction_is_unit():
    tip = (14.0, 0.0, 0.8)
    C, d = pixel_ray(P, 100.0, 500.0, tip, _axis(-57.0))
    assert abs(math.sqrt(sum(c * c for c in d)) - 1.0) < 1e-12


def test_project_rejects_point_on_camera_plane():
    tip = (14.0, 0.0, 0.8)
    axis = _axis(-57.0)
    C, x, y, z = camera_pose(P, tip, axis)
    on_plane = tuple(C[i] + 3.0 * x[i] for i in range(3))
    with pytest.raises(ValueError):
        project(P, on_plane, tip, axis)


def test_camera_center_moves_with_roll_but_axis_does_not():
    """光轴必须与夹爪轴严格同向（这是全部退化假设的地基），roll 不能动它。"""
    tip = (14.0, 0.0, 0.8)
    axis = _axis(-57.0)
    for roll in (0.0, 0.4, -1.3):
        p = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0, roll=roll)
        C, x, y, z = camera_pose(p, tip, axis)
        n = math.sqrt(sum(c * c for c in axis))
        assert math.dist(z, tuple(c / n for c in axis)) < 1e-12
        # C 到夹爪尖沿轴的距离必须恒为 t
        rel = tuple(tip[i] - C[i] for i in range(3))
        assert abs(sum(rel[i] * z[i] for i in range(3)) - 120.0) < 1e-9


def test_camera_axes_are_right_handed():
    """(xhat, yhat, zhat) 必须是右手系 —— 图像不能被镜像。

    ★ 2026-09-28 补：`camera_pose` 里 `e2 = _cross(z, e1)` 若写成
      `_cross(e1, z)`，图像左右翻转。**翻转是反射，吸收不进 roll
      （roll 是旋转）**，所以模型会整体失配 —— 这条把它当场钉住。
      （注意 roll 符号取反是旋转、不改手性，由
       test_roll_rotates_the_image_by_minus_roll 负责抓。）
    """
    tip = (14.0, 0.0, 0.8)
    for deg in (-57.0, -20.0, -88.0):
        for roll in (0.0, 0.4, -1.3):
            p = CamParams(f=1100.0, t=120.0, u_star=704.0, v_star=22.0,
                          roll=roll)
            C, x, y, z = camera_pose(p, tip, _axis(deg))
            cx = (x[1] * y[2] - x[2] * y[1],
                  x[2] * y[0] - x[0] * y[2],
                  x[0] * y[1] - x[1] * y[0])
            assert math.dist(cx, z) < 1e-12, (deg, roll, cx, z)
            # 正交归一
            assert abs(sum(x[i] * y[i] for i in range(3))) < 1e-12


# ============ 2026-09-29 新增：alpha = -90 处的相机坐标翻转 ============


def _dot(a, b):
    return sum(a[i] * b[i] for i in range(3))


def _ang(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(a, b)))))


def test_camera_frame_is_continuous_across_straight_down():
    """夹爪轴越过竖直向下（alpha = -90 deg）时，相机坐标系不能翻 180 deg。

    相机是刚体拧在腕上的：alpha 连续变，它的姿态必须连续变。

    旧实现 `e1 = _norm(_cross(up, z))` 的归一化**丢掉了 sign(cos alpha)**：
        up x z 的水平分量 = (cos a cos tb, cos a sin tb, 0)，归一化后
        e1 = sign(cos a) * (臂平面法线)
    于是 alpha 一过 -90 deg，e1/e2 同时取反 = 相机绕光轴翻 180 deg。

    变异自检：把 camera_pose 里的 e1 改回 `_norm(_cross((0,0,1), z))`，
    本用例立刻变红（相邻两帧夹角约 180 deg）。

    注意：别用 -88 / -92 来测 —— |sin a| > 0.999 会命中 `up = (1,0,0)`
    那条退路，把翻转**掩盖**掉（两边都变成约 (0,1,0)），测不出来。
    必须取 [-87, -93] 之外。
    """
    tip = (14.0, 0.0, 0.8)
    prev = None
    for i in range(0, 21):                   # -80 .. -100
        deg = -80.0 - i
        _, x, y, z = camera_pose(P, tip, _axis(deg))
        if prev is not None:
            px, py, pz = prev
            assert _ang(x, px) < 15.0, ('相机 x 轴跳变', deg, _ang(x, px))
            assert _ang(y, py) < 15.0, ('相机 y 轴跳变', deg, _ang(y, py))
            assert _ang(z, pz) < 15.0, ('光轴跳变', deg, _ang(z, pz))
        prev = (x, y, z)


def test_camera_x_axis_is_the_arm_plane_normal_on_both_sides():
    """相机 x 轴必须恒等于臂平面法线 (-sin tb, cos tb, 0)，与 sign(cos alpha) 无关。

    上一条的「定义版」：不依赖连续性，直接钉住 e1 该是什么。
    相机是拧在腕上的刚体，它的左右方向只能跟着**底座 yaw** 走，
    不可能随 alpha 翻面。
    """
    tip = (14.0, 0.0, 0.8)                   # tb = 0 -> 法线 = (0,1,0)
    for deg in (-85.0, -95.0, -99.0, -107.0, -120.0):
        _, x, _, _ = camera_pose(P, tip, _axis(deg))
        assert _ang(x, (0.0, 1.0, 0.0)) < 1e-9, (deg, x)
