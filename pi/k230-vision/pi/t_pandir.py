# 人可见时，直接测 pan 指令方向：发一条已知指令，看跟踪框在画面里往哪走。
# 判据：命令 "left" => 相机左转 => 画面内容右移 => 框中心 cx **增大**。
import json
import sys
import time

sys.path.insert(0, "/home/cy/k230-vision/pi")

from t_camcalib import make_nudge  # noqa: E402


def read_box(timeout=3.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            with open("/tmp/k230/latest-result.json") as f:
                d = json.load(f)
        except Exception:
            time.sleep(0.05)
            continue
        for o in (d.get("objs") or []):
            if isinstance(o, dict) and o.get("src") == "track":
                l, t, r, b = o["box"]
                return (l + r) / 2.0, (t + b) / 2.0, d.get("frame")
        time.sleep(0.05)
    return None, None, None


def probe(n, direction, deg):
    cx0, cy0, f0 = read_box()
    if cx0 is None:
        print("  没有跟踪目标，测不了", flush=True)
        return
    time.sleep(0.4)
    us, hit = n(direction, deg)
    time.sleep(1.6)
    cx1, cy1, f1 = read_box()
    if cx1 is None:
        print("  转完之后目标没了", flush=True)
        return
    print("  %-5s %4.0f° (->%sus hit=%s)   cx: %.3f -> %.3f  Δ=%+.3f   "
          "cy: %.3f -> %.3f  Δ=%+.3f   frame %s->%s%s"
          % (direction, deg, us, hit, cx0, cx1, cx1 - cx0,
             cy0, cy1, cy1 - cy0, f0, f1,
             "   <<< 目标换了（frame 跳变后不一定是同一个）" if f1 and f0 and f1 - f0 > 12 else ""),
          flush=True)


n = make_nudge()
time.sleep(0.6)
print("pan 方向测试（left 应让 cx 增大）", flush=True)
probe(n, "left", 12.0)
time.sleep(0.8)
probe(n, "right", 12.0)
print("PANDIR_DONE", flush=True)
