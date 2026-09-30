# /home/cy/k230-vision/pi/t_stick.py —— 判别"跟踪框在镜头运动时跟不跟得住"
#
# 判据：**同一帧里**同时读「检测器给的 person 框」和「src==track 的跟踪框」。
#   检测框是逐帧独立算的，**必然**跟着画面走；
#   镜头动一下之后：
#     两者一起动  -> 跟踪器跟得住（H3 否）
#     检测框动了、跟踪框没动 -> **跟踪框粘在图像坐标上**（H3 成立）
#
# 这个设计不需要静止目标、也不用外部锁：
# 人自己在动也没关系 —— 两个框在同一帧上，人的位移对两者影响相同。
import json
import sys
import time

PI = "/home/cy/k230-vision/pi"
sys.path.insert(0, PI)

from t_camcalib import make_nudge  # noqa: E402

DEG = 12.0


def read_frame():
    try:
        with open("/tmp/k230/latest-result.json") as f:
            d = json.load(f)
    except Exception:
        return None
    objs = d.get("objs") or []
    det = trk = None
    for o in objs:
        if not isinstance(o, dict):
            continue
        if o.get("src") == "track":
            trk = o["box"]
        elif o.get("cls") == "person":
            det = o["box"]
    return d.get("frame"), det, trk


def cx(b):
    return (b[0] + b[2]) / 2.0


def main():
    n = make_nudge()
    n("center")
    time.sleep(0.8)
    n("center")
    time.sleep(1.5)

    print("  t(s)   frame   det_cx   trk_cx   (初值是命令前)", flush=True)
    seen = set()
    t0 = time.time()
    issued = False
    base = None
    while time.time() - t0 < 5.0:
        r = read_frame()
        if r and r[0] not in seen:
            seen.add(r[0])
            fr, det, trk = r
            t = time.time() - t0
            if not issued and t > 1.0 and det is not None and trk is not None:
                us, hit = n("left", DEG)
                print("  ---- 发出 pan left %.0f° -> %sus hit=%s ----" % (DEG, us, hit), flush=True)
                issued = True
                base = (cx(det), cx(trk))
            ds = "%.4f" % cx(det) if det else "  n/a "
            ts = "%.4f" % cx(trk) if trk else "  n/a "
            extra = ""
            if base and det and trk:
                extra = "   Δdet=%+.4f  Δtrk=%+.4f" % (cx(det) - base[0], cx(trk) - base[1])
            print("  %+5.2f  %-6s  %s   %s%s" % (t, fr, ds, ts, extra), flush=True)
        time.sleep(0.01)

    print("STICK_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
