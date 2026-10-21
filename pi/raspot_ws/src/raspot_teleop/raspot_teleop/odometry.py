# -*- coding: utf-8 -*-
"""底盘里程计积分（纯 Python，无 ROS 依赖）。

输入是 0x55 上行帧的四轮速 + 陀螺 gz（~50Hz），输出 odom 系下的 (x, y, θ) 与 (vx, wz)。

## 航向用陀螺，不用轮速差（默认 `yaw_source='gyro'`）
这台车是四驱滑移转向：实测只转到指令的 48~56%，折算的等效轮距随速度变
（1.35 / 1.52 m，见 yaw_loop.py 文件头），`track_eff` 至今没标定。
轮速差反解出来的 wz 因此**系统性偏大一倍左右**，拿它积分航向会把 SLAM 的初值带歪。
gz 是陀螺原始量（可信），静止漂移实测 0.17°/min。`yaw_source='wheel'` 留着做对照。

## 静止门
四个轮速都低于 `still_mps` 时认为车没动：航向不积分（不吃陀螺零偏），位置也不动。
原地转的时候轮子是在转的（左右反向），不会被这道门挡住。
"""
import math

STILL_MPS = 0.005      # 四轮都低于这个 = 静止（轮速分辨率 1 mm/s）
MAX_DT_S = 0.5         # 断流超过这么久不跨断层积分（同老驱动）


class OdomIntegrator:

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0          # 连续累加，不回绕（四元数里自己会处理）
        self.vx = 0.0
        self.wz = 0.0
        self._last_t = None

    def update(self, wheels, gz, t, track_eff=1.0, wheel_scale=1.0,
               yaw_source='gyro', still_mps=STILL_MPS):
        """喂一帧。wheels=(vA,vB,vC,vD) m/s（A/B 左，C/D 右），gz rad/s，t 单调秒。
        返回 (vx, wz)：车体系瞬时速度。"""
        va, vb, vc, vd = wheels
        v_left = (va + vb) * 0.5 * wheel_scale
        v_right = (vc + vd) * 0.5 * wheel_scale
        vx = (v_left + v_right) * 0.5
        if yaw_source == 'wheel':
            wz = (v_right - v_left) / track_eff
        else:
            wz = gz
        if all(abs(v) < still_mps for v in wheels):
            vx = wz = 0.0

        if self._last_t is None:
            dt = 0.0
        else:
            dt = t - self._last_t
            if dt <= 0.0 or dt > MAX_DT_S:
                dt = 0.0
        self._last_t = t

        if dt > 0.0:
            # 中点法：这一拍的平移按"转了一半"的朝向走，弧线比欧拉法准
            mid = self.theta + wz * dt * 0.5
            self.x += vx * math.cos(mid) * dt
            self.y += vx * math.sin(mid) * dt
            self.theta += wz * dt
        self.vx, self.wz = vx, wz
        return vx, wz

    def quaternion_z_w(self):
        """平面航向 → 四元数 (z, w)。"""
        return math.sin(self.theta * 0.5), math.cos(self.theta * 0.5)
