# /home/cy/k230-vision/pi/t_pandead.py —— 补测 pan 轴的小步长响应
#
# 起因：t_camdeadband.py 里 pan 3.0° (33us) 那一次测出 +763 px / 内点 5 ——
# **内点 5 说明 ORB 配准失败了**，而 33us 抖出 763px 物理上不可能
# （同为 3° 的 tilt 只测到 50px，两轴像素灵敏度实测也几乎一样）。
# 所以 pan 在 6° 以下到底动不动，数据是空的。这个脚本补这一段，带重试。
#
# 用法：source /opt/ros/jazzy/setup.bash && python3 -u t_pandead.py
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from t_camcalib import make_nudge, shift_between, snap  # noqa: E402

STEPS = (1.0, 1.5, 2.0, 3.0, 4.0)
SETTLE = 0.9
MIN_INL = 25


def main():
    nudge = make_nudge()
    # 回中位：pan 有 center 指令；tilt 没有，只能"顶到下限再抬回来"
    nudge("center")
    nudge("down", 180.0)
    nudge("up", 700.0 / (1000.0 / 90.0))
    time.sleep(1.0)
    print("已回中位", flush=True)

    for deg in STEPS:
        us = round(deg * 1000.0 / 90.0)
        for attempt in range(1, 4):
            time.sleep(SETTLE)
            pa = snap("pd_a")
            _us, hit = nudge("left", deg)
            time.sleep(SETTLE)
            pb = snap("pd_b")
            dx, dy, rot, inl = shift_between(pa, pb)
            nudge("right", deg)          # 转回去，每次都从同一姿态起测
            print("pan %4.1f° (%3dus) 第%d次 -> dx=%+5d inl=%4d%s"
                  % (deg, us, attempt, dx, inl, "  撞限位" if hit else ""),
                  flush=True)
            if inl >= MIN_INL:
                break
            print("       内点太少，重试", flush=True)

    print("PANDONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
