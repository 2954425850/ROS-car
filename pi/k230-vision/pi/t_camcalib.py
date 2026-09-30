# /home/cy/k230-vision/pi/t_camcalib.py —— 云台居中闭环的**实测标定**
#
# camcenter.py 里有两个数字**只能实测**，猜不得：
#   pan_sign / tilt_sign  —— 云台往左转，画面里的东西往哪边移
#   k_deg_per_unit        —— 每 1.0 的归一化偏差该转多少度
#
# ## 为什么不用"跟一个检测框"
#
# 第一版是盯住检测器给的框量的。实测发现两个问题：
#   1. 画面里未必有可用的目标（那次是一个人腿，一直在动）
#   2. 框本身会跳（检测器掉帧 63%，本项目早有实测）
#
# 改成**图像互相关**：拍一张 → 转云台 → 再拍一张 → 算画面平移了多少像素。
# 这直接给出符号和"每度移动多少"，**任何有纹理的静止场景都行**，不需要检测器认出什么。
#
# ## 前置条件（脚本会自己检查）
#
#   ⚠️ 场景必须**静止** —— 开跑前别碰画面里的东西，别让人走进来。
#      脚本会先连拍两张、确认位移≈0 才开始转，不静止就直接拒绝。
#
# 用法（在树莓派上）：
#     source /opt/ros/jazzy/setup.bash
#     cd /home/cy/k230-vision/pi && python3 t_camcalib.py

import os
import subprocess
import sys
import time

import cv2
import numpy as np

K230CTL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "k230ctl")
TMP = "/tmp/k230"
STEP_DEG = 10.0
SETTLE_S = 0.8


# ---------------- 拍照 ----------------
def snap(tag):
    path = os.path.join(TMP, "calib_%s.jpg" % tag)
    r = subprocess.run([K230CTL, "snap", "-o", path, "--attempts", "2"],
                       capture_output=True, text=True, timeout=60)
    if r.returncode != 0 or not os.path.exists(path):
        raise RuntimeError("snap 失败: %s" % (r.stderr or r.stdout)[:200])
    return path


# ---------------- 配准 ----------------
def shift_between(a_path, b_path):
    """b 相对 a 位移了多少像素。**dx>0 = 画面内容往右移了。**

    ⚠️ 用**特征点 + 部分仿射**，不是模板匹配。
    第一版用 matchTemplate（纯平移模型），实测匹配分掉到 0.45、同一动作两次测出
    相反的 dy、还串到另一轴上（tilt 那次 dx 到了 -320）——
    因为云台旋转会带进**画面旋转和透视变化**，纯平移模型根本不成立。
    改成 ORB + estimateAffinePartial2D（RANSAC），能容忍旋转和尺度。

    返回 (dx, dy, 旋转角(度), 内点数)。
    """
    a = cv2.imread(a_path, cv2.IMREAD_GRAYSCALE)
    b = cv2.imread(b_path, cv2.IMREAD_GRAYSCALE)
    if a is None or b is None:
        raise RuntimeError("读不到图")
    orb = cv2.ORB_create(3000)
    k1, d1 = orb.detectAndCompute(a, None)
    k2, d2 = orb.detectAndCompute(b, None)
    if d1 is None or d2 is None or len(k1) < 8 or len(k2) < 8:
        return (0, 0, 0.0, 0)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    ms = bf.match(d1, d2)
    if len(ms) < 8:
        return (0, 0, 0.0, len(ms))
    p1 = np.float32([k1[m.queryIdx].pt for m in ms])
    p2 = np.float32([k2[m.trainIdx].pt for m in ms])
    M, inl = cv2.estimateAffinePartial2D(p1, p2, method=cv2.RANSAC,
                                         ransacReprojThreshold=3.0)
    if M is None:
        return (0, 0, 0.0, 0)
    rot = float(np.degrees(np.arctan2(M[1, 0], M[0, 0])))
    return (int(round(M[0, 2])), int(round(M[1, 2])), rot,
            int(inl.sum()) if inl is not None else 0)


# ---------------- 真云台 ----------------
def make_nudge():
    sys.path.insert(0, "/home/cy/voice-chatbot")
    from car.controller import CarController

    class _Cfg:
        def get(self, key, default=None):
            return default

    ctrl = CarController(_Cfg())
    ctrl.start()
    return ctrl.camera_nudge


def selftest_shift():
    """先用**合成位移**验一遍 shift_between 的符号和幅值。

    为什么必须做：这种"正负号"最容易搞反，而且搞反了不会报错 ——
    只会把 pan_sign 标成反的，然后云台越转越远。用已知答案的数据先钉死它。
    """
    img = cv2.imread(snap("selftest"), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("自检取不到图")
    origin = os.path.join(TMP, "calib_selftest_origin.png")
    cv2.imwrite(origin, img)
    bad = []
    for sx, sy in ((30, 0), (-30, 0), (0, 25), (0, -25), (20, 15)):
        M = np.float32([[1, 0, sx], [0, 1, sy]])
        moved = os.path.join(TMP, "calib_selftest_moved.png")
        cv2.imwrite(moved, cv2.warpAffine(img, M, (img.shape[1], img.shape[0])))
        dx, dy, rot, inl = shift_between(origin, moved)
        ok = abs(dx - sx) <= 2 and abs(dy - sy) <= 2 and inl >= 20
        print("    合成位移 (%+4d,%+4d) -> 测出 (%+5d,%+5d) 旋转%+.2f° 内点%d  %s"
              % (sx, sy, dx, dy, rot, inl, "ok" if ok else "FAIL"))
        if not ok:
            bad.append((sx, sy, dx, dy))
    if bad:
        print("  ✗ 自检失败 —— shift_between 的符号/幅值不对，标定结果不可信")
        return False
    print("  ✓ 自检通过（dx>0 = 画面内容往右移）")
    return True


def main():
    print("=" * 72)
    if not os.path.exists(K230CTL):
        print("找不到 k230ctl: %s" % K230CTL)
        return 1

    print("0) shift_between 自检（合成位移）…")
    if not selftest_shift():
        return 1
    print()

    print("先确认场景是静止的（连拍两张、位移必须≈0）…")
    p1 = snap("still1")
    time.sleep(0.4)
    p2 = snap("still2")
    dx, dy, rot, inl = shift_between(p1, p2)
    print("  没转云台时：dx=%+d dy=%+d 旋转%+.2f° 内点=%d" % (dx, dy, rot, inl))
    if abs(dx) > 6 or abs(dy) > 6 or inl < 20:
        print("  ✗ 场景在动或没纹理（位移 %d,%d 内点 %d）—— 请让画面保持静止再跑"
              % (dx, dy, inl))
        return 1
    print("  ✓ 场景静止")

    nudge = make_nudge()
    print("CarController 就绪")

    # 把两轴送回**中位**，让"控制器假定的起点"和"云台的实际位置"对齐。
    #
    # 为什么不能归到限位：试过，反而更糟 —— 行程尽头对着的往往是天花板/白墙，
    # ORB 内点从 83 掉到 4~5，量出来全是垃圾。中位才是相机日常对着的方向。
    #
    # pan 有 center 指令；**tilt 没有**（他们 API 的一个缺口）—— 只能
    # "先顶到下限 800、再抬 700us 回来"凑出 1500。
    nudge("center")
    nudge("down", 180.0)                       # tilt 顶到下限
    nudge("up", 700.0 / (1000.0 / 90.0))       # 800us + 700us = 1500us
    print("  已回中位（pan/tilt 各 1500us）")
    time.sleep(SETTLE_S)

    h, w = cv2.imread(p1, cv2.IMREAD_GRAYSCALE).shape
    results = {}

    # ⚠️ 测的方向必须是 `_direction(sign=+1)` 在目标偏「+」侧时会选的那一个：
    #      pan : 目标在左(du>0) -> "left"      tilt: 目标在上(dv>0) -> "up"
    #    第一版这里给 pan 测的是 "right"（偏「-」侧）、给 tilt 测的是 "up"，
    #    却对两轴用了同一条判据 —— 结果把 tilt 标反了（云台会越转越远）。
    for axis, direction, back in (("pan", "left", "right"),
                                  ("tilt", "up", "down")):
        print()
        print("---- %s 轴：往 %s 转 %.0f°（目标偏+侧时该走的方向）----"
              % (axis, direction, STEP_DEG))
        got = None
        for attempt in range(1, 4):
            # 刚做过大动作（回中、或上一次测量）时第一张常常还没稳 ——
            # 实测见过内点 5、旋转 -94° 的垃圾结果。多试两次，别让一次抖动毁掉标定。
            time.sleep(SETTLE_S)
            pa = snap("a_%s_%d" % (axis, attempt))
            us, hit = nudge(direction, STEP_DEG)
            if hit:
                print("  第%d次：脉宽 %s 撞限位 —— 不算" % (attempt, us))
                nudge(back, STEP_DEG)
                continue
            time.sleep(SETTLE_S)
            pb = snap("b_%s_%d" % (axis, attempt))
            dx, dy, rot, inl = shift_between(pa, pb)
            nudge(back, STEP_DEG)      # 转回去，别把云台留在偏的地方
            print("  第%d次：dx=%+d dy=%+d 旋转%+.2f° 内点=%d"
                  % (attempt, dx, dy, rot, inl))
            if inl >= 25:
                got = (dx, dy, rot, inl)
                break
            print("       内点太少，重试")
        if got is None:
            print("  ✗ 三次都没测到可信结果 —— 换个有纹理的场景")
            continue
        dx, dy, rot, inl = got

        d = dx if axis == "pan" else dy
        if abs(d) < 3:
            print("  ✗ 几乎没动（%d 像素）—— 云台没响应？" % d)
            continue
        # 自然映射成立 <=> 这个命令把目标**送回中心**，也就是让内容朝「+」方向移动
        sign = 1 if d > 0 else -1
        per_deg = abs(d) / float(w if axis == "pan" else h) / STEP_DEG
        results[axis] = (sign, per_deg, d, inl)
        print("  → 推断 %s_sign = %+d" % (axis, sign))
        # pan 用宽归一化、tilt 用高归一化 —— 别混着乘回去
        print("     每度移动 %.6f 归一化（%s 轴，除以%s）"
              % (per_deg, axis, "宽" if axis == "pan" else "高"))
        print("     deadbeat k=%.0f  建议取一半 k=%.0f"
              % ((1.0 / per_deg) if per_deg else 0,
                 (0.5 / per_deg) if per_deg else 0))

    print()
    print("=" * 72)
    if not results:
        print("两轴都没测到")
        return 1
    print("填进 camcenter.CenterConfig：")
    for axis, (sign, per_deg, d, inl) in results.items():
        print("    %s_sign=%+d        # 每度 %.6f 归一化（%d 像素/度，内点 %d）"
              % (axis, sign, per_deg, per_deg * w, inl))
    print("CALIB_DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
