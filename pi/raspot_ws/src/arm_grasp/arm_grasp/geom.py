# -*- coding: utf-8 -*-
"""标定之后的**纯几何**链路：关节角 + 像素 → 目标三维坐标 / 深度。

## 和 cam_model.py 的分工（别搞混）
* `cam_model` = 退化手眼的**拟合**那套：参数 (f, t, u*, v*, roll) 全靠拟合。
* `geom`（本模块）= **标定之后**的路：手眼 4 个数是**尺子量的**、内参是**标定出来的**、
  工具系基向量共用 `cam_model.tool_axes`。**全链路没有任何拟合。**

## 已知量（全部实测，一个都不拟合）
* 手眼（相机在工具系里的位置，米）：张开 (−0.0075, +0.0340, −0.1190)
                                    夹紧 (−0.0075, +0.0340, −0.1360)
  两态 z 之差 1.7cm = L4 之差 ⇒ **相机是刚体挂在腕上的**，不是挂在爪尖上。
* 内参：chn2(AI)/320x180/ROTATE_180=True，RMS 0.329px（见 K_AI320x180_CHN2）
* `SIGMA = +1`：由手眼数 + 目测抓取像素反验钉死（推导文档 §2.5）。

## 推导文档
`Desktop\\raspot小车\\4.机械臂抓取\\2026-09-29-已知量推出未知量-纯几何推导.md`
"""
import json
import math
from collections import namedtuple

from .arm_kin import fk
from .cam_model import tool_axes

# ---------------- 手眼（尺子实测，米） ----------------
CAM_OPEN = (-0.0075, 0.0340, -0.1190)
CAM_CLOSED = (-0.0075, 0.0340, -0.1360)
L4_OPEN, L4_CLOSED = 16.0, 17.7        # cm；注意 arm_kin.fk 固定用 17.7（夹紧态）

# ---------------- 内参（2026-09-29 标定） ----------------
DIST_AI320 = (0.17965918630312014, -0.7166182076277368,
              0.00782884309815537, 0.0027021964720660633, 0.8938154550073525)

Intrinsics = namedtuple('Intrinsics', 'fx fy cx cy dist w h')

K_AI320x180_CHN2 = Intrinsics(
    fx=277.25526835902406, fy=276.3123247310423,
    cx=150.21484884942453, cy=97.14799560154957,
    dist=DIST_AI320, w=320, h=180)

# ★ 图像符号 σ ∈ {+1, −1}：相机自己的右手系要么与 (x̂_T, ŷ_T, ẑ_T) 同向，
#   要么绕光轴转 180°（两个轴同时反）。**实测：是后者，σ = −1。**
#
#   证据（2026-09-29 实拍，原始帧 = 标定帧 = 检测框所在帧）：
#     画面里夹爪**在底部**、压在蓝色瓶盖上（≈(550~650, 620~700)），
#     而 σ=+1 预测瞄准像素在 (671, 71)（画面**顶部**）—— 上下整个反了。
#     σ=−1 预测 (531, 704) ✓ 和图里一致。
#
#   这也就是「相机倒置、看图要转 180°」（原记忆那条，**是对的**）：
#     转正后夹爪在**顶部**、略偏右 ⇒ 归一化 (0.5246, 0.0993)，
#     正是设计文档里记的"抓取像素 ≈ (0.55, 0.03)"。
#     ⚠️ 那个数是在**转正后**的图上看的；拿它当原始帧的数用就会把 σ 判反（我踩过）。
SIGMA = -1.0

# 台面/目标面（基座安装面为 z=0）：
# ---------------- 跨分辨率换算（**唯一允许的一种**） ----------------
# 检测框 / 跟踪器 / capture_frame 都在**根分辨率 1280x720**；
# 标定的 K 在 **chn2 AI 通道 320x180**。两者是**均匀 ÷4**（1280/320 = 720/180 = 4，
# FOV 不变）—— 这正是 [[calib-match-consumer-channel]] 里说"允许"的那一种。
#
# ⚠️ **只对均匀缩放成立**：本平台配成 320x320 时是**压扁**（纵向拉长 1.78x），
#    那种情况下除法换算**不成立**。换通道/改配置后先确认 FOV 没变再用。
#
# 归一化坐标在两者里**完全相同** —— 拿不准时就在归一化坐标里比，避开这件事。
STREAM_W, STREAM_H = 1280, 720      # 检测/跟踪/拍照所在的根分辨率
AI_W, AI_H = 320, 180               # 标定所在的通道分辨率


def to_stream(u, v, k=K_AI320x180_CHN2):
    """标定系(320x180)像素 → 根分辨率(1280x720)像素。"""
    return u * (STREAM_W / k.w), v * (STREAM_H / k.h)


def to_ai(u, v, k=K_AI320x180_CHN2):
    """根分辨率(1280x720)像素 → 标定系(320x180)像素。"""
    return u * (k.w / STREAM_W), v * (k.h / STREAM_H)


def normalized(u, v, w=AI_W, h=AI_H):
    """像素 → 归一化（这套坐标跨分辨率不变）。"""
    return u / w, v / h


Z_MOUNT = 0.0        # 车身/安装面
CAP_HEIGHT = 0.016   # 瓶盖自身高 1.6cm（放在车身上 ⇒ 顶面 z = +0.016）


def load_intrinsics(path):
    """从 `5.K230/内参标定/*.json` 读一份内参。

    **换通道 / 换分辨率 / 改 ROTATE_180 之后走这条路，不要去改上面的常量。**
    内参是"像素坐标系"的属性，换了坐标系就是另一组数，而且重投影 RMS 照样好看。
    """
    with open(path, 'r', encoding='utf-8') as f:
        d = json.load(f)
    return Intrinsics(fx=d['fx_fy'][0], fy=d['fx_fy'][1],
                      cx=d['cx_cy'][0], cy=d['cx_cy'][1],
                      dist=tuple(d['dist']),
                      w=d['image_size'][0], h=d['image_size'][1])


# ---------------- 畸变 ----------------

def distort(u, v, k=K_AI320x180_CHN2):
    """去畸变像素 → 原始像素（OpenCV 5 参正向模型）。"""
    k1, k2, p1, p2, k3 = k.dist
    x = (u - k.cx) / k.fx
    y = (v - k.cy) / k.fy
    r2 = x * x + y * y
    rad = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    return (k.cx + k.fx * (x * rad + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)),
            k.cy + k.fy * (y * rad + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y))


def undistort(u, v, k=K_AI320x180_CHN2, iters=30):
    """原始像素 → 去畸变像素（上面那个的逆，不动点迭代）。"""
    k1, k2, p1, p2, k3 = k.dist
    x0 = (u - k.cx) / k.fx
    y0 = (v - k.cy) / k.fy
    x, y = x0, y0
    for _ in range(iters):
        r2 = x * x + y * y
        rad = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        if abs(rad) < 1e-12:
            raise ValueError('畸变模型在 (%.1f, %.1f) 处退化' % (u, v))
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (x0 - dx) / rad
        y = (y0 - dy) / rad
    return k.cx + k.fx * x, k.cy + k.fy * y


# ---------------- 位姿 ----------------

def gripper_tip(joints, closed=True):
    """爪尖在基座系下的位置，**米**。

    `arm_kin.fk()` 固定用 L4 = 17.7cm（夹紧态）。张开态要沿 ẑ_T 回退
    (17.7 − 16.0) = 1.7cm —— 这是「同一个物理爪尖、两种读法」的全部内容。
    """
    tip_cm, axis = fk(joints)
    if closed:
        return tuple(c / 100.0 for c in tip_cm)
    _, _, z = tool_axes(tip_cm, axis)
    back = (L4_CLOSED - L4_OPEN) / 100.0
    return tuple(tip_cm[i] / 100.0 - back * z[i] for i in range(3))


def camera_center(joints):
    """相机光心，基座系，米。

    ★ `fk()` 给的是**夹紧态**爪尖，所以这里必须配 `CAM_CLOSED`。
      相机挂在**腕**上 ⇒ 张开态用 (爪尖回退 1.7cm) + `CAM_OPEN` 算出来**是同一点**
      —— 这条自洽性有测试钉着（`test_camera_center_is_independent_of_...`）。
    """
    tip_cm, axis = fk(joints)
    xh, yh, zh = tool_axes(tip_cm, axis)
    c = CAM_CLOSED
    return tuple(tip_cm[i] / 100.0
                 + c[0] * xh[i] + c[1] * yh[i] + c[2] * zh[i]
                 for i in range(3))


def handeye_range(closed=True):
    """相机到爪尖的**欧氏**距离（米）。抓取那一刻的深度必须等于它。"""
    c = CAM_CLOSED if closed else CAM_OPEN
    return math.sqrt(sum(v * v for v in c))


def aim_pixel(closed=True, k=K_AI320x180_CHN2):
    """爪尖（= 抓取点）落在目标上时，目标应出现在哪个像素。**常数，与目标深度无关。**

    ★ 两态差 (2.2, 9.9) px，折到目标处 ≈ 4.9 mm。**哪一个才是真的抓取像素由检验 P3 定**
      （把瓶盖放两个高度、爪尖贴上，看目标落在哪个像素）。
    ★ 也别拿画面中心当瞄准点：折到目标处差 (7.5 mm 横, 34.0 mm 竖)，34mm 比瓶盖还大。
    """
    c = CAM_CLOSED if closed else CAM_OPEN
    dx, dy, dz = -c[0], -c[1], -c[2]        # 目标在爪尖处 ⇒ Δ = 爪尖 − 光心
    # ★ 必须再走一遍 distort()：检测器报的是**原始（含畸变）**像素，
    #   而上面那两行是针孔（去畸变）值。两者在这个位置差约 0.35 px。
    #   （变异自检抓到的：不套 distort，跨姿态一致性测试会红 0.18~0.33 px）
    return distort(k.cx + SIGMA * k.fx * dx / dz,
                   k.cy + SIGMA * k.fy * dy / dz, k)


# ---------------- 像素 ↔ 三维 ----------------

def pixel_ray(joints, u, v, k=K_AI320x180_CHN2):
    """像素 → (光心 C, 单位方向 d)，都在基座系。"""
    uu, vv = undistort(u, v, k)
    tip_cm, axis = fk(joints)
    xh, yh, zh = tool_axes(tip_cm, axis)
    a = SIGMA * (uu - k.cx) / k.fx
    b = SIGMA * (vv - k.cy) / k.fy
    d = tuple(a * xh[i] + b * yh[i] + zh[i] for i in range(3))
    n = math.sqrt(sum(c * c for c in d))
    if n < 1e-9:
        raise ValueError('退化射线')
    return camera_center(joints), tuple(c / n for c in d)


def target_from_pixel(joints, u, v, z_target_m, k=K_AI320x180_CHN2, min_down=0.3):
    """像素 + 目标所在平面的高度 → (目标基座系坐标, 轴向深度, 欧氏距离)，米。

    射线太平（`d_z > -min_down`）**直接报错**，绝不返回垃圾深度 ——
    那正是"自洽但错"的入口。`min_down=0.3` 对应视线与平面夹角 ≳17.5°。
    """
    C, d = pixel_ray(joints, u, v, k)
    if d[2] > -min_down:
        raise ValueError('射线太贴平面（d_z = %.3f），深度不可用' % d[2])
    t = (z_target_m - C[2]) / d[2]
    if t <= 0.0:
        raise ValueError('目标平面在相机背后（t = %.4f）' % t)
    P = tuple(C[i] + t * d[i] for i in range(3))
    _, axis = fk(joints)
    zt = axis
    Z_axial = t * sum(d[i] * zt[i] for i in range(3))
    if Z_axial <= 0.0:
        raise ValueError('轴向深度为负（光轴朝背离目标的一侧）')
    return P, Z_axial, t


def project(joints, P_base, k=K_AI320x180_CHN2):
    """基座系点 → 像素（含畸变）。正解，给检验 P3/P4 与自检用。"""
    C = camera_center(joints)
    tip_cm, axis = fk(joints)
    xh, yh, zh = tool_axes(tip_cm, axis)
    d = tuple(P_base[i] - C[i] for i in range(3))
    Z = sum(d[i] * zh[i] for i in range(3))
    if Z <= 1e-9:
        raise ValueError('点在相机平面之后（Z = %.4f）' % Z)
    X = sum(d[i] * xh[i] for i in range(3))
    Y = sum(d[i] * yh[i] for i in range(3))
    return distort(k.cx + SIGMA * k.fx * X / Z,
                   k.cy + SIGMA * k.fy * Y / Z, k)


# ---------------- 便利：把上面的数打出来看一眼 ----------------

def main():
    print('内参: fx=%.3f fy=%.3f cx=%.3f cy=%.3f  (%dx%d)'
          % (K_AI320x180_CHN2.fx, K_AI320x180_CHN2.fy, K_AI320x180_CHN2.cx,
             K_AI320x180_CHN2.cy, K_AI320x180_CHN2.w, K_AI320x180_CHN2.h))
    print('fx/fy = %.4f  (方形像元应 ≈1)' % (K_AI320x180_CHN2.fx / K_AI320x180_CHN2.fy))
    for closed in (False, True):
        u, v = aim_pixel(closed)
        print('瞄准像素  %s: (%7.2f, %7.2f)  归一化 (%.4f, %.4f)  相机→爪尖 %.4f m'
              % ('夹紧' if closed else '张开', u, v,
                 u / K_AI320x180_CHN2.w, v / K_AI320x180_CHN2.h,
                 handeye_range(closed)))
    pose = dict(base=6.0, shoulder=110.88, elbow=-89.28, wrist_pitch=-77.04)
    tip = gripper_tip(pose, closed=False)
    print('出厂姿态: 爪尖 = (%.4f, %.4f, %.4f) m' % tip)
    print('          光心 = (%.4f, %.4f, %.4f) m' % camera_center(pose))
    print('          爪尖投影 = %s' % (project(pose, tip),))
    print('瓶盖(车身上，顶面 z=%.3f): 目标面 z = %.3f' % (CAP_HEIGHT, CAP_HEIGHT))


if __name__ == '__main__':
    main()
