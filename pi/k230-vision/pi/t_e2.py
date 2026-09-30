# /home/cy/k230-vision/pi/t_e2.py —— 量闭环的**反馈延迟**（用静止目标）
#
# 目的（2026-09-21）：分辨两个候选根因
#   H1 反馈延迟 > 指令间隔  -> 框会"慢慢移、后到位"
#   H3 跟踪框粘在图像坐标上 -> 框根本不动
#
# 为什么必须用静止物体：用人当目标时，**他自己在动**，框的位移
# "是镜头动的还是人动的"分不开（前面几次实测就是这么被污染的）。
# 这里用 8557 外部入口把跟踪器锁在一把椅子上 —— 椅子的框位置是**镜头姿态的纯函数**。
#
# 注意：8557 的 pt/box 会关掉 det_auto（文档明写）。跑完补一条 auto 恢复。
import json
import subprocess
import sys
import time

PI = "/home/cy/k230-vision/pi"
K230CTL = PI + "/k230ctl"
sys.path.insert(0, PI)

from t_camcalib import make_nudge  # noqa: E402

BOX = (0.19, 0.10, 0.50, 0.76)      # 椅子靠背+西瓜那一块（归一化 l,t,r,b）


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
    # 1) 锁到椅子上
    r = subprocess.run([K230CTL, "box"] + ["%.3f" % v for v in BOX],
                       capture_output=True, text=True, timeout=30)
    print("锁椅子: rc=%s out=%s%s" % (r.returncode, r.stdout.strip(), r.stderr.strip()), flush=True)
    time.sleep(2.0)

    got = read_box()
    if got is None:
        print("没锁上（画面里没有 src==track）—— 中止", flush=True)
        return 1
    print("锁上了: box=[%.3f,%.3f,%.3f,%.3f]" % tuple(got[0]), flush=True)

    n = make_nudge()
    # 2) 预热两条（第一条大概率丢，见 t_gimsen 的结论），并让 pan 停在 1500
    n("center")
    time.sleep(1.0)
    n("center")
    time.sleep(1.5)

    # 3) 基线：采 1 秒
    t0 = time.time()
    xs = []
    while time.time() - t0 < 1.0:
        g = read_box()
        if g:
            xs.append(cx(g[0]))
        time.sleep(0.02)
    if not xs:
        print("基线段里没有目标 —— 中止", flush=True)
        return 1
    base = sum(xs) / len(xs)
    print("基线 cx = %.4f（%d 次采样）" % (base, len(xs)), flush=True)

    # 4) 发一条指令
    tcmd = time.time()
    us, hit = n("left", 15.0)
    print("发出 pan left 15°  ->  %sus  hit=%s" % (us, hit), flush=True)
    print("", flush=True)
    print("  t(s)   frame   cx       Δcx(相对基线)", flush=True)

    # 5) 采 3 秒
    seen = set()
    while time.time() - tcmd < 3.0:
        g = read_box()
        if g and g[1] not in seen:
            seen.add(g[1])
            print("  %+5.2f  %-6s  %.4f   %+.4f"
                  % (time.time() - tcmd, g[1], cx(g[0]), cx(g[0]) - base), flush=True)
        time.sleep(0.01)

    # 6) 恢复 det_auto
    r = subprocess.run([K230CTL, "auto"], capture_output=True, text=True, timeout=30)
    print("", flush=True)
    print("已发 auto 恢复自动锁定: %s%s" % (r.stdout.strip(), r.stderr.strip()), flush=True)
    print("E2_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
