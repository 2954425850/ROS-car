# /sdcard/k230vision/t_payload_depth.py —— attach_depths 的纯逻辑测试
import sys
sys.path.insert(0, "/sdcard/k230vision")

import tof_locate as tl

FAILS = []


def chk(cond, label):
    if not cond:
        FAILS.append(label)
        print("  FAIL  %s" % label)


def make_frame(rows, cols, value):
    return {"rows": rows, "cols": cols, "payload": bytes([value]) * (rows * cols)}


IDENT = {"ax": 1.0, "bx": 0.0, "ay": 1.0, "by": 0.0}

print("=" * 74)
print("A) 给 chair 框挂上 depth_mm，别的类不挂")
f = make_frame(100, 100, 110)
objs = [{"cls": "chair", "box": [0.4, 0.4, 0.6, 0.6]},
        {"cls": "person", "box": [0.1, 0.1, 0.2, 0.2]}]
tl.attach_depths(objs, f, 4, IDENT, classes=("chair",), max_boxes=3)
chk(objs[0].get("depth_mm") == 440, "chair 应挂上 440（实得 %s）" % objs[0].get("depth_mm"))
chk("depth_mm" not in objs[1], "person 不该挂 depth_mm")

print()
print("B) 无回波 → depth_mm 是 None（不是删掉键）")
f2 = make_frame(100, 100, 0)
objs2 = [{"cls": "chair", "box": [0.4, 0.4, 0.6, 0.6]}]
tl.attach_depths(objs2, f2, 4, IDENT, classes=("chair",), max_boxes=3)
chk("depth_mm" in objs2[0] and objs2[0]["depth_mm"] is None,
    "无回波时键必须在、值是 None（实得 %r）" % objs2[0])

print()
print("C) frame 为 None（ToF 没起来）→ 键仍是 None，绝不抛异常")
objs3 = [{"cls": "chair", "box": [0.4, 0.4, 0.6, 0.6]}]
tl.attach_depths(objs3, None, 4, IDENT, classes=("chair",), max_boxes=3)
chk(objs3[0].get("depth_mm", "missing") is None, "frame=None 时应挂 None")

print()
print("D) max_boxes 限量：只算面积最大的那几个")
f4 = make_frame(100, 100, 105)
objs4 = [{"cls": "chair", "box": [0.0, 0.0, 0.1, 0.1]},
         {"cls": "chair", "box": [0.0, 0.0, 0.5, 0.5]},
         {"cls": "chair", "box": [0.5, 0.5, 0.9, 0.9]}]
tl.attach_depths(objs4, f4, 4, IDENT, classes=("chair",), max_boxes=1)
chk(objs4[1].get("depth_mm") == 420, "面积最大的那个应被算（实得 %s）"
    % objs4[1].get("depth_mm"))
chk("depth_mm" not in objs4[0] and "depth_mm" not in objs4[2],
    "超量的框不该被动过")

print()
print("★ 变异锚点")
print("  1) 把 attach_depths 里 result is None 的分支改成 'continue'（不挂键）⇒ B/C 段红")
print("  2) 去掉 max_boxes 排序取前 N ⇒ D 段红")
print()
print("=" * 74)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for x in FAILS:
        print("   -", x)
else:
    print("结果: 全部通过")
print("PAYLOAD_DONE")
