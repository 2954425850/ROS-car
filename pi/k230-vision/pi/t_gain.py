# /home/cy/k230-vision/pi/t_gain.py —— 量环路的**实际增益**（用它分辨 H1'）
#
# 目的：给一个已知的镜头位移，量跟踪框在画面里实际移动了多少，
#       算出「实得位移 / 应有的位移」这个比值。
#   比值 >1  -> 每次修正都过冲，逐帧发指令必然累积成单向跑飞（H1' 成立）
#   比值 ≈0.5 -> t_camcalib 标的"deadbeat 的一半"是对的，问题不在增益
#
# 为什么用 8557 锁苹果，而不是用检测器：
#   检测器**不报这颗苹果**（小目标，分数贴 0.30 门限）—— 实测 8 格扫描全 objs=0，
#   而苹果明明在画面里（快照可见）。跟踪器是类别无关的，锁得住。
#
# 为什么不用人：人在动，"框位移是镜头动的还是人动的"分不开（前面几次都栽在这）。
import json
import subprocess
import sys
import time

PI = "/home/cy/k230-vision/pi"
K230CTL = PI + "/k230ctl"
sys.path.insert(0, PI)

from t_camcalib import make_nudge  # noqa: E402

BOX = (0.700, 0.675, 0.770, 0.785)     # 地板上那颗绿苹果（归一化 l,t,r,b）
DEG = 12.0
US_PER_DEG = 1000.0 / 90.0
DEG_PER_UNIT_PAN = 38.0                # camcenter 标定值
SNAP_A = "/tmp/k230/g_a.jpg"
SNAP_B = "/tmp/k230/g_b.jpg"


def snap(path):
    r = subprocess.run([K230CTL, "snap", "-o", path, "--attempts", "3"],
                       capture_output=True, text=True, timeout=60)
    return r.returncode == 0


def read_box():
    try:
        with open("/tmp/k230/latest-result.json") as f:
            d = json.load(f)
    except Exception:
        return None
    for o in (d.get("objs") or []):
        if isinstance(o, dict) and o.get("src") == "track":
            return o["box"], d.get("frame")
    return None


def cx(b):
    return (b[0] + b[2]) / 2.0


def main():
    r = subprocess.run([K230CTL, "box"] + ["%.3f" % v for v in BOX],
                       capture_output=True, text=True, timeout=30)
    print("锁苹果: %s%s" % (r.stdout.strip(), r.stderr.strip()), flush=True)
    time.sleep(2.0)

    g = read_box()
    if g is None:
        print("没锁上 —— 中止", flush=True)
        return 1
    print("锁上了: box=[%.3f,%.3f,%.3f,%.3f]" % tuple(g[0]), flush=True)

    n = make_nudge()
    n("center")
    time.sleep(0.8)
    n("center")          # 预热第二条（第一条会被 DDS 发现期吃掉）
    time.sleep(1.5)

    snap(SNAP_A)
    print("已拍 A", flush=True)

    # 基线
    t0 = time.time()
    xs = []
    while time.time() - t0 < 1.0:
        g = read_box()
        if g:
            xs.append(cx(g[0]))
        time.sleep(0.02)
    if not xs:
        print("基线段没目标 —— 中止", flush=True)
        return 1
    base = sum(xs) / len(xs)
    print("基线 cx = %.4f（%d 次）" % (base, len(xs)), flush=True)

    tcmd = time.time()
    us, hit = n("left", DEG)
    print("发出 pan left %.0f°  ->  %sus  hit=%s" % (DEG, us, hit), flush=True)
    print("  理论位移 = -%.0f°  =>  Δcx 应约 %+.4f（按 %.0f px/度算）"
          % (0.0, 0.0, 0.0) if False else
          "  该指令对应 Δcx 的理论值 = %+.4f（%.0f° × %.1f px/度 ÷ 1280）"
          % (DEG * (17.0 / 1280.0), DEG, 17.0), flush=True)
    print("", flush=True)
    print("   t(s)   frame   cx       Δcx", flush=True)

    seen = set()
    while time.time() - tcmd < 3.5:
        g = read_box()
        if g and g[1] not in seen:
            seen.add(g[1])
            print("  %+5.2f  %-6s  %.4f   %+.4f" % (time.time() - tcmd, g[1], cx(g[0]), cx(g[0]) - base),
                  flush=True)
        time.sleep(0.01)

    snap(SNAP_B)
    print("已拍 B", flush=True)
    subprocess.run([K230CTL, "auto"], capture_output=True, text=True, timeout=30)
    print("GAIN_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
