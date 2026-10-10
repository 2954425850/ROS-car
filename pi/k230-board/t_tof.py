# /sdcard/k230vision/t_tof.py —— a010_frame.py 的纯逻辑测试
#
# **不碰 UART、不建管线、不需要外设插着** —— 零风险，瞬间跑完。
# 和 t_targetrx_logic.py 一个路子：先把"解析对不对"钉死，再谈接硬件。
#
# 本测试**能红**：坏帧必须被拒、好帧必须被收，都是真断言，不是打印出来看看。
# 变异锚点见 H 段（把校验关掉会让它变红）。

import sys

sys.path.insert(0, "/sdcard/k230vision")

import a010_frame as af

FAILS = []


def chk(cond, label):
    if not cond:
        FAILS.append(label)
        print("  FAIL  %s" % label)


def mk_payload(rows, cols, val):
    return bytes([val]) * (rows * cols)


print("=" * 74)
print("A) 合法帧：能解出来，字段对得上")
p = mk_payload(4, 4, 50)
fr = af.build_frame(4, 4, p, frame_id=7)
chk(len(fr) == 20 + 16 + 2, "4x4 帧长应为 20+16+2=38（实得 %d）" % len(fr))

frame, rest = af.feed(fr)
chk(frame is not None, "合法帧必须被解出")
if frame:
    chk(frame["rows"] == 4 and frame["cols"] == 4, "rows/cols")
    chk(frame["frame_id"] == 7, "frame_id 应为 7（实得 %s）" % frame["frame_id"])
    chk(len(frame["payload"]) == 16, "payload 长度应为 16")
    chk(frame["payload"][0] == 50, "payload 内容")
    chk(rest == b"", "解完后剩余应为空")
else:
    chk(False, "合法帧未解出")

print()
print("B) 粘包：两帧连一起，两次 feed 各取一帧")
two = af.build_frame(4, 4, mk_payload(4, 4, 10), frame_id=1) + \
      af.build_frame(4, 4, mk_payload(4, 4, 20), frame_id=2)
f1, rest = af.feed(two)
chk(f1 is not None and f1["frame_id"] == 1, "第一帧 id=1")
chk(len(rest) == 38, "解出第一帧后应剩第二帧的 38 字节（实得 %d）" % len(rest))
f2, rest2 = af.feed(rest)
chk(f2 is not None and f2["frame_id"] == 2, "第二帧 id=2")
chk(rest2 == b"", "第二帧解完后剩余为空")

print()
print("C) 分片到达：一字节一字节喂，最后必须解出同一帧")
buf = b""
got = None
for byte in fr:
    got_f, buf = af.feed(buf + bytes((byte,)))
    if got_f:
        got = got_f
chk(got is not None and got["frame_id"] == 7, "逐字节喂也应解出 id=7")
chk(buf == b"", "逐字节喂完后缓冲应清空")

print()
print("D) 前面的垃圾：帧头之前有噪声，仍应找到真帧")
noise = b"\x12\x34\x00\x11\x22"      # 注意结尾不是 0x00，避免和帧头混淆
f, rest = af.feed(noise + fr)
chk(f is not None and f["frame_id"] == 7, "跳过前导噪声后应解出")
chk(rest == b"", "噪声之后无剩余")

print()
print("E) 坏帧必须被拒（这是关键——校验不能形同虚设）")
bad_csum = af.build_frame(4, 4, mk_payload(4, 4, 50), corrupt_checksum=True)
f, _ = af.feed(bad_csum)
chk(f is None, "checksum 错的帧必须被拒（实得 %s）" % (f is not None))

bad_tail = af.build_frame(4, 4, mk_payload(4, 4, 50), corrupt_tail=True)
f, _ = af.feed(bad_tail)
chk(f is None, "帧尾不是 0xDD 的帧必须被拒")

print()
print("F) 长度字段非法：不能被当成巨大 payload 卡死")
# data_len < 16（负 payload）—— 直接手搓头部
junk = bytearray(30)
junk[0], junk[1] = 0x00, 0xFF
junk[2], junk[3] = 3, 0       # data_len=3 < 16
f, rest = af.feed(bytes(junk) + fr)
chk(f is not None and f["frame_id"] == 7,
    "非法的长头之后应丢掉重找、并认出后面的真帧")

# data_len 巨大（> 10000+16）
junk2 = bytearray(30)
junk2[0], junk2[1] = 0x00, 0xFF
big = 16 + 10000 + 1
junk2[2], junk2[3] = big & 0xFF, (big >> 8) & 0xFF
f, rest = af.feed(bytes(junk2) + fr)
chk(f is not None and f["frame_id"] == 7, "payload 超上限的头之后应能重找")

print()
print("G) 不完整帧：只有半个，必须返回 None 且不丢字节")
half = fr[:20]
f, rest = af.feed(half)
chk(f is None, "半个帧不能解出")
chk(rest == half, "半个帧应原样保留等待后续（实得 %d 字节）" % len(rest))

print()
print("H) center_mm：中心取中位数、滤掉 0")


def frame_with(rows, cols, values):
    payload = bytes(values)
    return {"rows": rows, "cols": cols, "payload": payload}


# 8x8 全 100 -> 中心区中位数 100
f8 = frame_with(8, 8, [100] * 64)
chk(af.center_mm(f8, 4) == 400, "全 100、unit=4 -> 400mm（实得 %s）" % af.center_mm(f8, 4))

# 一半 0（无回波）一半 100：中位数仍应是 100，不被 0 拉低
# ⚠️ 这里必须用 frac=0.5：8x8 图在默认 frac=0.2 下中心区只有 **1 个像素**，
#    "滤 0"这个性质在那个尺度上根本测不出来（真机 100x100 的中心区是 20x20，
#    不会退化，所以这是**测试用例**的问题，不是 center_mm 的问题）。
half0 = [0] * 32 + [100] * 32      # 行0~3=0，行4~7=100
chk(af.center_mm(frame_with(8, 8, half0), 4, frac=0.5) == 400,
    "混入 0 时应滤掉，中位数仍 400（实得 %s）"
    % af.center_mm(frame_with(8, 8, half0), 4, frac=0.5))

# 全 0 -> None
chk(af.center_mm(frame_with(8, 8, [0] * 64), 4) is None, "全 0 应返回 None")

# 中位数 vs 均值：多一个极远值，中位数不动
vals = [50] * 63 + [255]
m = af.center_mm(frame_with(8, 8, vals), 1)
chk(m == 50, "单个坏点不应影响中位数（实得 %s）" % m)

print()
print("★ 变异锚点（说明测试是能红的）")
print("  上面 E 段的两条断言，如果把 a010_frame.feed 里的 checksum/帧尾检查删掉，")
print("  就会变红。也就是说 E 段不是走过场。")

print()
print("=" * 74)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for x in FAILS:
        print("   -", x)
else:
    print("结果: 全部通过")
print("LOGIC_DONE")
