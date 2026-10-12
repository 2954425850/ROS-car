# /sdcard/k230vision/t_tof_locate.py —— tof_locate.py 的纯逻辑测试
#
# 不碰 UART、不需要外设。本测试**能红**：变异锚点见文件末尾。
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


def put(frame, r0, r1, c0, c1, value):
    p = bytearray(frame["payload"])
    for r in range(r0, r1):
        for c in range(c0, c1):
            p[r * frame["cols"] + c] = value
    frame["payload"] = bytes(p)


IDENT = {"ax": 1.0, "bx": 0.0, "ay": 1.0, "by": 0.0}
BOX = [0.4, 0.4, 0.6, 0.6]

print("=" * 74)
print("A) 目标面(105) + 更远的桌面(110)：必须取到 105，不能被桌面拉走")
# 100x100：桌面 110 计数，中央 20x20 目标面 105。
# 105 与 110 只差 5 计数 —— 瓶盖顶面在桌面上方 12.5mm @ UNIT=4 ≈ 3 计数，就是这个量级。
f = make_frame(100, 100, 110)
put(f, 40, 60, 40, 60, 105)
chk(tl.locate_depth(f, BOX, 4, IDENT) == 420,
    "应取到 105*4=420mm（实得 %s）" % tl.locate_depth(f, BOX, 4, IDENT))

print()
print("B) 近场假回波（12 计数 = 48mm）落在框内：必须被 150mm 量程下限滤掉")
f2 = make_frame(100, 100, 110)
put(f2, 40, 60, 40, 60, 105)
put(f2, 45, 48, 45, 48, 12)            # 9 个假点，就在框里
chk(tl.locate_depth(f2, BOX, 4, IDENT) == 420,
    "框内假近点应被量程下限滤掉（实得 %s）" % tl.locate_depth(f2, BOX, 4, IDENT))

print()
print("C) 爪尖那一档（30 计数 = 120mm）也必须被滤掉 —— 而且是一大片，不是「少数像素」")
f3 = make_frame(100, 100, 110)
put(f3, 40, 60, 40, 60, 105)
put(f3, 42, 58, 42, 58, 30)            # 16x16 = 256 像素的 120mm，比目标面还大
chk(tl.locate_depth(f3, BOX, 4, IDENT) == 420,
    "大片 120mm 假回波应被量程下限滤掉（实得 %s）" % tl.locate_depth(f3, BOX, 4, IDENT))

print()
print("D) 目标只占框内一小部分 ⇒ 取最小仍必须拿到目标（中位数会在这里翻车）")
f4 = make_frame(100, 100, 110)
put(f4, 45, 55, 45, 55, 105)           # 100/1024 ≈ 10%
chk(tl.locate_depth(f4, BOX, 4, IDENT) == 420,
    "少数面积的目标面必须被取到（实得 %s）" % tl.locate_depth(f4, BOX, 4, IDENT))

print()
print("E) 无效值 0 / 255 必须滤掉")
f5 = make_frame(100, 100, 0)
put(f5, 40, 60, 40, 60, 255)          # 全是无效
chk(tl.locate_depth(f5, BOX, 4, IDENT) is None, "全 0/255 应返回 None")
f6 = make_frame(100, 100, 255)
put(f6, 45, 55, 45, 55, 105)
chk(tl.locate_depth(f6, BOX, 4, IDENT) == 420,
    "255 是无效值，不该参与统计（实得 %s）" % tl.locate_depth(f6, BOX, 4, IDENT))

print()
print("F) 框整体出画 → None（不是裁成一个边条硬算）")
chk(tl.box_to_region([1.2, 0.4, 1.4, 0.6], 100, 100, IDENT) is None,
    "完全出界的框应返回 None")
r = tl.box_to_region([0.95, 0.4, 1.2, 0.6], 100, 100, IDENT)
chk(r is not None and r[2] >= 0 and r[3] <= 100, "部分出界应裁到界内（实得 %s）" % (r,))

print()
print("G) 映射参数真的被用上（ax=2 时同一个框落到两倍宽的区域）")
r1 = tl.box_to_region([0.2, 0.2, 0.4, 0.4], 100, 100, IDENT)
r2 = tl.box_to_region([0.2, 0.2, 0.4, 0.4], 100, 100,
                      {"ax": 2.0, "bx": 0.0, "ay": 2.0, "by": 0.0}, pad=0.0)
chk((r2[3] - r2[2]) > (r1[3] - r1[2]), "ax=2 的横向区域应更宽")

print()
print("H) frame=None（ToF 没起来）→ None，不抛异常")
chk(tl.locate_depth(None, BOX, 4, IDENT) is None, "frame=None 应返回 None")

print()
print("★ 变异锚点（说明测试是能红的）")
print("  1) 把 region_depth 的「取最小」换成「取中位数」⇒ A/D 立刻变红（会得到 440）")
print("  2) 把 MIN_MM 从 150 改成 0 ⇒ B/C 立刻变红（会得到 48 / 120）")
print("  3) 把 val_max 从 254 提到 255 ⇒ E 变红")
print("  4) 把 box_to_region 的越界裁剪删掉 ⇒ F 段变红")
print()
print("=" * 74)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for x in FAILS:
        print("   -", x)
else:
    print("结果: 全部通过")
print("LOCATE_DONE")
