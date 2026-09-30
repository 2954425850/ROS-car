# /sdcard/k230vision/t_obstacles_logic.py —— obstacles.py 的纯逻辑测试
#
# **不碰 socket、不需要网卡、不建管线** —— 零风险，瞬间跑完。
# 跑法（板上）：exec(open('/sdcard/k230vision/t_obstacles_logic.py').read())
#
# 为什么不测 socket：这个平台 127.0.0.1 不通、连自己的 WiFi 地址也会阻塞，
# 板上做不了回环（同 t_targetrx_logic.py）。socket 那层只能靠真实外部连接验 ——
# 而且那一层我们**没有自己写**，直接复用了 reporter.Reporter（已验证）。
#
# 本测试**能红**：下面 ★ 标的是变异锚点 —— 把实现改成"看起来更合理"的写法，
# 那几条必须变红。全是真断言，不是打印出来看看。

import sys

sys.path.insert(0, "/sdcard/k230vision")

from obstacles import depth_zone, priority_of, free_zones, build, encode
import ujson

# 与 config.py 的出厂默认值一致（改动 config 时这里也要跟着改）
NH, MH = 0.45, 0.18
HIGH, LOW = ("person",), ("chair", "couch")
ZONES, BOTTOM_MIN, H_MIN = 10, 0.80, 0.10

FAILS = []


def chk(cond, label):
    if not cond:
        FAILS.append(label)
        print("  FAIL  %s" % label)


def eq(got, want, label):
    chk(got == want, "%s（实得 %r / 期望 %r）" % (label, got, want))


print("=" * 74)
print("0) 浮点精度（决定容差怎么定）")
print("   0.7-0.4      = %.10f" % (0.7 - 0.4))
print("   0.7-0.4==0.3 = %s" % ((0.7 - 0.4) == 0.3))
print("   （本平台是**单精度**，见 README §7.7。所以下面凡跟阈值比的用例，")
print("     都用明确落在档内的值，不拿「正好等于阈值」当断言。）")
print()

print("A) depth_zone —— 判据是**框高比**，不是面积、也不是框宽")
# 框高 0.55 / 0.30 / 0.06
eq(depth_zone([0.0, 0.40, 0.5, 0.95], NH, MH), "near", "h=0.55 -> near")
eq(depth_zone([0.0, 0.60, 0.5, 0.90], NH, MH), "mid", "h=0.30 -> mid")
eq(depth_zone([0.0, 0.90, 0.5, 0.96], NH, MH), "far", "h=0.06 -> far")

# ★ 变异锚点：如果实现改成"按框宽判"或"按面积判"，下面第一条必须变红 ——
#   这条框**又宽又扁**（宽 0.9、高只有 0.06），按宽判会误判成 near。
eq(depth_zone([0.05, 0.90, 0.95, 0.96], NH, MH), "far",
   "★ 宽 0.9 高 0.06 的扁框必须是 far（改成按宽判就会红）")
# ★ 反向：又窄又高的框，按宽判会误判成 far
eq(depth_zone([0.48, 0.20, 0.52, 0.95], NH, MH), "near",
   "★ 宽 0.04 高 0.75 的窄高框必须是 near（改成按宽判就会红）")
print()

print("B) priority_of —— 按**类别**排，不按框大小")
eq(priority_of("person", HIGH, LOW), "high", "person -> high（安全相关）")
eq(priority_of("chair", HIGH, LOW), "low", "chair -> low（软目标，可推开）")
eq(priority_of("dog", HIGH, LOW), "mid", "dog -> mid（其余）")
print()

print("C) free_zones —— 粗糙代理量，不是「可通行区域」真值")
# 框底 0.95 压到下部（>= 0.80）且框高 0.55 >= 0.10 -> 占格
eq(free_zones([{"cls": "chair", "box": [0.0, 0.40, 0.30, 0.95]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.4, 1.0]],
   "占 u∈[0,0.3] 的近处障碍 -> 剩 [0.4,1.0]")

# 远处小目标：框底 0.15 < 0.80 -> 不占格
eq(free_zones([{"cls": "dog", "box": [0.8, 0.10, 0.9, 0.15]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.0, 1.0]],
   "远处小目标（框底 0.15）不占格 -> 全通")

# 框底够低但**太小**（h=0.04 < 0.10）-> 也不占格
eq(free_zones([{"cls": "dog", "box": [0.2, 0.92, 0.3, 0.96]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.0, 1.0]],
   "贴地但极小的框（h=0.04）不占格 -> 全通")

# ★★ 下面两条**成对**，专门隔离 bottom_min。
#     上面那两条"不占格"的用例**同时被 h_min 挡着**，所以验不到 bottom_min ——
#     2026-10-01 变异自检发现的缺口：把 `if b < bottom_min: continue` 整段删掉，
#     上面全绿。这两条就是补上的那一对。
#     同一个框，只有框底位置不同，结论必须相反。
eq(free_zones([{"cls": "chair", "box": [0.0, 0.30, 0.30, 0.60]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.0, 1.0]],
   "★ 框够大（h=0.30）但框底只到 0.60（没压到画面下部）-> **不**占格")
eq(free_zones([{"cls": "chair", "box": [0.0, 0.30, 0.30, 0.90]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.4, 1.0]],
   "★ 同一个框、框底压到 0.90 -> **占**格（两条成对才咬得住 bottom_min）")

# 每一格都被占 -> 空列表。**调用方必须能处理空**（空 = 无路可走，不是"没数据"）
eq(free_zones([{"cls": "x", "box": [0.0, 0.40, 1.0, 1.0]}],
              ZONES, BOTTOM_MIN, H_MIN), [],
   "横跨整幅的近处障碍 -> 空列表")

# 左右各一块，中间留空
eq(free_zones([{"cls": "a", "box": [0.0, 0.40, 0.15, 0.9]},
               {"cls": "b", "box": [0.85, 0.40, 1.0, 0.9]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.2, 0.8]],
   "左右各一块 -> 中间 [0.2,0.8]")

# 右边界压到 1.0：i1 不能算到 n（那会 IndexError）
r = free_zones([{"cls": "x", "box": [0.95, 0.40, 1.0, 1.0]}],
               ZONES, BOTTOM_MIN, H_MIN)
chk(r is not None, "右边界 = 1.0 时不越界（实得 %r）" % (r,))

# ★ 变异锚点：右端**闭区间**（碰到格线也算占住）是**有意偏保守**的。
#   如果实现改成 `range(i0, i1)`（右端开），这条必须变红。
#   保守方向的意义：宁可把一格误报成"不通"，也不要把障碍边缘报成"可通行"。
eq(free_zones([{"cls": "x", "box": [0.0, 0.40, 0.30, 0.95]}],
              ZONES, BOTTOM_MIN, H_MIN), [[0.4, 1.0]],
   "★ 右沿正好落在 u=0.3 格线上时，那一格也算被占（改成右端开区间就会红）")

# 畸形的对象不该把主循环炸掉（真实数据里一定会有缺字段的）
for bad in ([{"cls": "x"}], [{"cls": "x", "box": [1, 2, 3]}],
            [{"cls": "x", "box": None}], [{}]):
    r = free_zones(bad, ZONES, BOTTOM_MIN, H_MIN)
    chk(r == [[0.0, 1.0]], "畸形输入 %r -> 全通、不抛（实得 %r）" % (bad, r))
print()

print("D) build —— 排序是给人看的，Pi 侧不该依赖顺序")
b = build([{"cls": "dog", "score": 0.9, "box": [0.0, 0.90, 0.5, 0.96]},
           {"cls": "chair", "score": 0.7, "box": [0.0, 0.40, 0.5, 0.95]}],
          NH, MH, HIGH, LOW, ZONES, BOTTOM_MIN, H_MIN)
eq([d["depth_zone"] for d in b], ["near", "far"], "near 排在 far 前面")
eq([d["priority"] for d in b], ["low", "mid"], "priority 跟着类别走")
eq(sorted(b[0].keys()),
   ["box", "depth_zone", "h_ratio", "priority", "score", "type"], "字段齐全")

# 同一 depth_zone 内：high 优先级的排前面
b2 = build([{"cls": "chair", "score": 0.9, "box": [0.0, 0.40, 0.5, 0.95]},
            {"cls": "person", "score": 0.5, "box": [0.6, 0.40, 0.9, 0.95]}],
           NH, MH, HIGH, LOW, ZONES, BOTTOM_MIN, H_MIN)
eq(b2[0]["type"], "person", "同为 near 时 person（high）排椅子（low）前面")

eq(build([{"cls": "x", "box": None}, {"cl": "?"}],
         NH, MH, HIGH, LOW, ZONES, BOTTOM_MIN, H_MIN), [],
   "全都是畸形 -> 空列表")
print()

print("E) encode —— 一行 NDJSON 的 bytes")
raw = encode([{"type": "chair"}], [[0.0, 1.0]], 7, 123)
chk(not raw.endswith(b"\n"), "encode **不**自己加换行（换行由 Reporter 补）")
d = ujson.loads(raw)
eq(sorted(d.keys()), ["frame", "free_zones", "obstacles", "ts"], "字段齐全")
eq(d["frame"], 7, "frame 透传")
eq(d["ts"], 123, "ts 透传")
eq(d["free_zones"], [[0.0, 1.0]], "free_zones 透传")
print()

print("=" * 74)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for f in FAILS:
        print("   -", f)
else:
    print("结果: 全部通过")
print("LOGIC_DONE")
