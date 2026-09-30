# /home/cy/k230-vision/pi/t_gimsign.py —— 云台两轴**方向**的开环实测
#
# 起因：2026-09-21 实车跑闭环时，pan 的步长一路从 1.7° 涨到 4.0° 且方向不变
# （= 误差在变大，正反馈特征），怀疑符号反了。但闭环里"目标动了"和"符号反了"
# 分不开，所以这里做**开环**：发一条已知方向的指令，用 ORB 量画面位移，
# 完全不依赖跟踪器给的框。
#
# 判据（与 t_camcalib.py 口径一致）：
#   camcenter._direction 对 d>0（目标偏画面左/上）返回 "left" / "up"，
#   而"自然映射成立"<=> 这条命令把目标**送回中心**，也就是让**画面内容朝「+」方向移动**。
#   所以：left 应给出 dx > 0；up 应给出 dy > 0。
#   **反了就是符号反了。**
#
# 用法（在树莓派上，脱离 SSH 通道跑）：
#   systemd-run --user --unit=gimsign --collect \
#     --property=WorkingDirectory=/home/cy/k230-vision/pi \
#     --property=StandardOutput=append:/tmp/gimsign.log \
#     bash -lc 'source /opt/ros/jazzy/setup.bash && exec python3 -u t_gimsign.py'

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from t_camcalib import make_nudge, shift_between, snap  # noqa: E402

STEPS = (("left", "right", "pan"), ("right", "left", "pan"),
         ("up", "down", "tilt"), ("down", "up", "tilt"))
DEG = 10.0
SETTLE = 0.9


def main():
    nudge = make_nudge()
    nudge("center")
    nudge("down", 180.0)
    nudge("up", 700.0 / (1000.0 / 90.0))
    time.sleep(1.0)
    print("已回中位", flush=True)

    out = {}
    for direction, back, axis in STEPS:
        time.sleep(SETTLE)
        pa = snap("gs_a")
        us, hit = nudge(direction, DEG)
        time.sleep(SETTLE)
        pb = snap("gs_b")
        dx, dy, rot, inl = shift_between(pa, pb)
        nudge(back, DEG)
        d = dx if axis == "pan" else dy
        out[(axis, direction)] = (d, inl)
        print("%-5s 往 %-5s 转 %4.0f°  命令脉宽 %sus  ->  dx=%+5d dy=%+5d  内点%4d%s"
              % (axis, direction, DEG, us, dx, dy, inl, "  撞限位" if hit else ""),
              flush=True)

    print("", flush=True)
    print("---- 判据 ----", flush=True)
    for axis, direction in (("pan", "left"), ("tilt", "up")):
        d, inl = out.get((axis, direction), (0, 0))
        if inl < 25:
            print("%s/%s: 内点 %d 太少，这次测量不可信，要重测" % (axis, direction, inl),
                  flush=True)
            continue
        want = ">0"
        ok = d > 0
        print("%s 往 %s 转 -> %+d 像素（要 %s）  =>  %s"
              % (axis, direction, d, want,
                 "自然映射成立（sign=+1 对）" if ok else "**反了！sign 应为 -1**"), flush=True)
    print("GIMSIGN_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
