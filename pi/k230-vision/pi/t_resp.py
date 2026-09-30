# /home/cy/k230-vision/pi/t_resp.py —— 量这个环的**真实响应延迟**
#
# 起因（2026-09-21）：持续跟踪环每帧都发一条满幅指令（间隔 0.15~0.20s），
# 但舵机动作 + 板子推理 + 推流加起来有几百毫秒延迟 —— 环拿着旧画面算下一条，
# 指令层层叠加（目标只偏 0.188，累计发了 63°），把目标直接甩出画面。
#
# 这个脚本量那个延迟：发**一条** pan 指令，然后逐帧记录目标框中心 cx 的变化。
# 从曲线能看出：延迟多少、多久稳定下来 —— 那是 settle 门控该设多久的依据。
import json
import sys
import time

sys.path.insert(0, "/home/cy/k230-vision/pi")

from t_camcalib import make_nudge  # noqa: E402


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


def cx_of(box):
    return (box[0] + box[2]) / 2.0


def main():
    n = make_nudge()
    time.sleep(0.5)

    # 基线：采 0.8 秒
    t0 = time.time()
    xs = []
    while time.time() - t0 < 0.8:
        r = read_box()
        if r:
            xs.append(cx_of(r[0]))
        time.sleep(0.02)
    if not xs:
        print("画面里没有 src==track 的目标，量不了 —— 请站在画面里", flush=True)
        return 1
    cx0 = sum(xs) / len(xs)
    print("基线 cx = %.4f  （%.0f ms 内采了 %d 次）" % (cx0 * 100, 800, len(xs)), flush=True)

    tcmd = time.time()
    us, hit = n("left", 10.0)
    print("发出  pan left 10°  ->  %sus  hit=%s" % (us, hit), flush=True)
    print("", flush=True)
    print("  t(s)   frame   cx      Δcx(相对基线)   Δcy", flush=True)

    seen = set()
    while time.time() - tcmd < 2.6:
        r = read_box()
        if r and r[1] not in seen:
            seen.add(r[1])
            box, fr = r
            cx = cx_of(box)
            cy = (box[1] + box[3]) / 2.0
            print("  %+5.2f  %-6s  %.4f   %+.4f        %+.4f"
                  % (time.time() - tcmd, fr, cx, cx - cx0, cy), flush=True)
        time.sleep(0.01)

    print("", flush=True)
    print("判读：Δcx 开始明显变化的那一刻 = 环的延迟；Δcx 不再变的时刻 = settle 时间", flush=True)
    print("RESP_DONE", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
