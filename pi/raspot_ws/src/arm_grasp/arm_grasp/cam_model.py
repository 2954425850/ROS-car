# -*- coding: utf-8 -*-
"""退化手眼相机模型。

## 为什么能退化成 5 个参数
光轴 ∥ 夹爪下扎方向，**由机械结构绝对保证（不是目测）**。所以相机的姿态
基本已知，只剩绕光轴的自转（roll）。未知量：

    f       焦距，像素
    t       相机沿光轴离夹爪尖的距离，cm
    u_star  夹爪尖落在哪个像素（横向偏移 + 主点，一起吸收）
    v_star  同上
    roll    图像相对臂竖直平面的滚转，弧度
  (+ u0, v0 主点，默认取画面中心，残差大时才开放拟合)

## 姿态无关
这 5 个量描述「相机相对 servo3 那根连杆的固定关系」，连杆位姿由 arm_kin.fk()
给出。臂动到别的姿态，fk 跟着变，**参数不用重标**。
"""
import math

DEFAULT_U0 = 640.0        # 1280x720 画面中心
DEFAULT_V0 = 360.0


class CamParams:
    __slots__ = ('f', 't', 'u_star', 'v_star', 'roll', 'u0', 'v0')

    def __init__(self, f, t, u_star, v_star, roll, u0=None, v0=None):
        self.f = float(f)
        self.t = float(t)
        self.u_star = float(u_star)
        self.v_star = float(v_star)
        self.roll = float(roll)
        self.u0 = float(DEFAULT_U0 if u0 is None else u0)
        self.v0 = float(DEFAULT_V0 if v0 is None else v0)

    def as_vector(self, fit_principal=False):
        if fit_principal:
            return [self.f, self.t, self.u_star, self.v_star, self.roll,
                    self.u0, self.v0]
        return [self.f, self.t, self.u_star, self.v_star, self.roll]

    @staticmethod
    def from_vector(vec, fit_principal=False):
        if fit_principal:
            f, t, us, vs, roll, u0, v0 = vec
            return CamParams(f, t, us, vs, roll, u0, v0)
        f, t, us, vs, roll = vec
        return CamParams(f, t, us, vs, roll)

    def __repr__(self):
        return ('CamParams(f=%.2f t=%.2f u*=%.2f v*=%.2f roll=%.5f '
                'u0=%.1f v0=%.1f)' % (self.f, self.t, self.u_star,
                                      self.v_star, self.roll, self.u0, self.v0))


def _norm(v):
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-12:
        raise ValueError('zero vector')
    return tuple(c / n for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def tool_axes(tip, axis):
    """工具系基向量 (xhat, yhat, zhat)：原点 = 爪尖，zhat = 夹爪下扎方向。

    ★★ xhat 必须取**臂平面法线**，不能由「世界竖直」搭。相机是拧在腕上的刚体，
       而归一化(世界竖直 x zhat) 会**丢掉 sign(cos alpha)**：
           (0,0,1) x zhat = cos(alpha) * (臂平面法线)
       归一化除以 |cos alpha| 后 = sign(cos alpha) * (臂平面法线)。
       于是夹爪轴一越过竖直向下（alpha = -90 deg），xhat/yhat **同时取反**
       ==> 相机绕光轴翻 180 deg。相机是刚体，不会这么动。

    实测代价（12 条样本、同一套参数，2026-09-29）：
        alpha > -90 的 6 条平均残差 113.5 px；alpha < -90 的 6 条 995.6 px
        改后同一批变成 113.5 / 186.3；整批 px_rms 258 -> 65 px
        射线共点散布（不依赖尺子）4.00 -> 1.30 cm
    这一条同时是「roll 到底是 0 还是 pi 定不下来」和「残差径向主导、符号全正」
    的根因 —— 有效 roll 在悬崖两侧差 pi，两边各测一次就各说各话。

    tb 取 atan2(tip_y, tip_x)，**不要**从 axis 的水平分量反解：那两分量本身
    就带着 sign(cos alpha)，反解会把同一个 bug 请回来。
    """
    z = _norm(axis)
    if math.hypot(tip[0], tip[1]) < 1e-9:
        raise ValueError('夹爪尖落在底座转轴上，相机绕光轴的帧没有定义')
    tb = math.atan2(tip[1], tip[0])
    e1 = (-math.sin(tb), math.cos(tb), 0.0)   # 臂平面法线，与 alpha 无关
    return e1, _cross(z, e1), z



def camera_pose(params, tip, axis):
    """返回 (C, xhat, yhat, zhat)。zhat = 光轴 = 夹爪下扎方向。

    C 由「夹爪尖必须投影到 (u_star, v_star)」反解：
        tip - C = t*zhat + L,  L ⊥ zhat
    代进投影式解出 L，就是下面那两行。**把 P = tip 代回去必然得到
    (u_star, v_star)** —— 这条自洽性有测试钉着。
    """
    e1, e2, z = tool_axes(tip, axis)
    cr, sr = math.cos(params.roll), math.sin(params.roll)
    x = tuple(cr * e1[i] + sr * e2[i] for i in range(3))
    y = tuple(-sr * e1[i] + cr * e2[i] for i in range(3))
    k = params.t / params.f
    off = tuple(k * ((params.u_star - params.u0) * x[i] +
                     (params.v_star - params.v0) * y[i]) for i in range(3))
    C = tuple(tip[i] - params.t * z[i] - off[i] for i in range(3))
    return C, x, y, z


def project(params, point, tip, axis):
    """世界点 -> 像素 (u, v)。"""
    C, x, y, z = camera_pose(params, tip, axis)
    v = _sub(point, C)
    d = _dot(v, z)
    if abs(d) < 1e-9:
        raise ValueError('point on the camera plane')
    return (params.u0 + params.f * _dot(v, x) / d,
            params.v0 + params.f * _dot(v, y) / d)


def pixel_ray(params, u, v, tip, axis):
    """像素 -> (相机中心 C, 单位方向)。"""
    C, x, y, z = camera_pose(params, tip, axis)
    d = tuple(z[i] + ((u - params.u0) / params.f) * x[i]
              + ((v - params.v0) / params.f) * y[i] for i in range(3))
    return C, _norm(d)


def ray_plane(C, direction, z_plane):
    """射线 ∩ 水平面 z = z_plane。射线不朝平面走时抛 ValueError。"""
    if abs(direction[2]) < 1e-9:
        raise ValueError('ray parallel to plane')
    s = (z_plane - C[2]) / direction[2]
    if s <= 0:
        raise ValueError('plane is behind the camera (s=%.4f)' % s)
    return (C[0] + s * direction[0], C[1] + s * direction[1], z_plane)
