# /home/cy/k230-vision/pi/t_e2b.py —— 用**检测框**（不依赖跟踪锁）量环的反馈延迟
#
# 为什么换掉跟踪框：t_e2.py 用 8557 锁椅子之后，发一条 pan 指令，
# 跟踪器 3 秒内一个 src==track 都没输出（锁丢了）。椅子靠背是大面积纯深色，
# NanoTrack 在上面本来就容易漂 —— 目标选得不好，结果不算数。
#
# 检测框是**逐帧独立算的**，不需要跟踪锁，不依赖跟踪器的漂移判据；
# 它随镜头运动的变化就是纯的"镜头姿态 -> 画面坐标"传递，正好用来量
# 从"发指令"到"画面反映出结果"的延迟。
#
# 判据：
#   Δcx 很快出现并稳定  -> 延迟小（H1 不成立，得去查 H3）
#   Δcx 出现得慢        -> 延迟大（H1 成立：必须 act-and-wait）
#   Δcx 始终不动        -> 指令没落地 / 相机没动
import json
import sys
import time

PI = "/home/cy/k230-vision/pi"
sys.path.insert(0, PI)

from t_camcalib import make_nudge  # noqa: E402

CLS = "chair"


def read_cx():
    """返回 (cx, frame)，取指定类别的检测框（不信 src==track）。"""
    try:
        with open("/tmp/k230/latest-result.json") as f:
            d = json.load(f)
    except Exception:
        return None
    for o in (d.get("objs") or []):
        if isinstance(o, dict) and o.get("cls") == CLS and not o.get("src"):
            l, t, r, b = o["box"]
            return (l + r) / 2.0, d.get("frame")
    return None


def main():
    n = make_nudge()
    # 预热两条（第一条大概率丢），pan 停 1500
    n("center")
    time.sleep(1.0)
    n("center")
    time.sleep(1.5)

    # 基线
    t0 = time.time()
    xs = []
    while time.time() - t0 < 1.0:
        g = read_cx()
        if g:
            xs.append(g[0])
        time.sleep(0.02)
    if not xs:
        print("画面里没有 %s 检测框 —— 中止" % CLS, flush=True)
        return 1
    base = sum(xs) / len(xs)
    print("基线 cx = %.4f（%d 次采样）" % (base, len(xs)), flush=True)

    tcmd = time.time()
    us, hit = n("left", 15.0)
    print("发出 pan left 15°  ->  %sus  hit=%s" % (us, hit), flush=True)
    print("", flush=True)
    print("   t(s)   frame   cx       Δcx", flush=True)

    seen = set()
    while time.time() - tcmd < 3.5:
        g = read_cx()
        if g and g[1] not in seen:
            seen.add(g[1])
            print("  %+5.2f  %-6s  %.4f   %+.4f" % (time.time() - tcmd, g[1], g[0], g[0] - base),
                  flush=True)
        time.sleep(0.01)

    print("E2B_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
