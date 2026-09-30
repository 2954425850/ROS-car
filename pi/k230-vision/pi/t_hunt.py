# 把云台往下转，一格一格找画面里的人（tilt 中位 1500 是朝天花板的）
import json
import os
import sys
import time

sys.path.insert(0, "/home/cy/k230-vision/pi")

from t_camcalib import make_nudge  # noqa: E402

US_PER_DEG = 1000.0 / 90.0

n = make_nudge()
n("center")            # pan 回中
time.sleep(0.8)

cur = 1500.0
found = False
for i in range(14):
    n("down", 8.0)     # tilt 往下 8°
    cur -= 8.0 * US_PER_DEG
    time.sleep(1.3)
    try:
        d = json.load(open("/tmp/k230/latest-result.json"))
    except Exception as e:
        print("  读结果失败 %s" % e, flush=True)
        continue
    objs = d.get("objs") or []
    persons = [o for o in objs if o.get("cls") == "person"]
    trk = [o for o in objs if o.get("src") == "track"]
    print("tilt≈%-6.0fus  objs=%d  person=%d  track=%d   %s"
          % (cur, len(objs), len(persons), len(trk),
             ", ".join("%s[%.2f,%.2f,%.2f,%.2f]" % (o.get("cls"), *o["box"])
                       for o in objs[:3])),
          flush=True)
    if persons or trk:
        found = True
        break

print("", flush=True)
print("找到人: %s  最终 tilt≈%.0fus" % (found, cur), flush=True)
print("HUNT_DONE", flush=True)
