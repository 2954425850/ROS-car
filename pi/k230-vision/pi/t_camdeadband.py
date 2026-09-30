# /home/cy/k230-vision/pi/t_camdeadband.py —— 测云台"给多大的指令才动"
#
# 起因：端到端跑居中时，tilt 连着发了 3 次 4.5°（约 50us），**画面一动不动**；
# 而标定里发 10°（111us）却能测到 221 像素的位移。
# 差别很可能是**静摩擦/舵机死区** —— 小指令驱不动。
#
# 这个脚本对每个轴给一串递增的步长，量每次的实际画面位移，画出"指令 -> 响应"曲线。
# 拿到它才能定 min_step_deg 该设多大（现在的 2° 很可能根本推不动）。
#
# 用法：source /opt/ros/jazzy/setup.bash && python3 t_camdeadband.py

import os
import subprocess
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from t_camcalib import K230CTL, TMP, shift_between, snap   # noqa: E402

STEPS = (3.0, 6.0, 12.0, 25.0)
SETTLE_S = 0.9


def make_nudge():
    sys.path.insert(0, "/home/cy/voice-chatbot")
    from car.controller import CarController

    class _Cfg:
        def get(self, key, default=None):
            return default

    ctrl = CarController(_Cfg())
    ctrl.start()
    return ctrl.camera_nudge


def main():
    print("=" * 72)
    nudge = make_nudge()
    # 先回中位，让两轴都在已知姿态
    nudge("center")
    nudge("down", 180.0)
    nudge("up", 700.0 / (1000.0 / 90.0))
    time.sleep(1.0)
    print("已回中位")

    img = cv2.imread(snap("db0"), cv2.IMREAD_GRAYSCALE)
    h, w = img.shape

    for axis, fwd, back in (("pan", "left", "right"), ("tilt", "up", "down")):
        print()
        print("---- %s 轴：指令步长 -> 实际画面位移 ----" % axis)
        for deg in STEPS:
            pa = snap("db_a")
            us, hit = nudge(fwd, deg)
            time.sleep(SETTLE_S)
            pb = snap("db_b")
            dx, dy, rot, inl = shift_between(pa, pb)
            nudge(back, deg)
            time.sleep(SETTLE_S)
            d = dx if axis == "pan" else dy
            span = w if axis == "pan" else h
            verdict = "没动" if abs(d) < 3 else ("动了" if abs(d) > 8 else "勉强")
            print("  %5.1f° (%4dus) -> %+5d px  (%5.1f%% 画幅, 内点%4d)  %s%s"
                  % (deg, round(deg * 1000.0 / 90.0), d, 100.0 * abs(d) / span, inl,
                     verdict, "  撞限位" if hit else ""))

    print()
    print("提示：'没动'的那几档就是死区 —— min_step_deg 必须大于它")
    print("DEADBAND_DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
