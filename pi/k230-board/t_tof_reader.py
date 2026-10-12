# /sdcard/k230vision/t_tof_reader.py —— TofReader 的纯逻辑测试（假 UART，不碰硬件）
#
# ★ 与计划的差异（2026-10-12 实施时实测补的，见下方"变异锚点"）：
#   计划初稿的 A/D 两段**挡不住它自己列的两条变异** —— 实测两条变异都全绿。原因：
#     * A 段只送**一帧**且帧后没有尾数据 ⇒ `feed` 在"没收全"时返回的就是整段 buf，
#       把 rest 丢掉（`fr, _ = ...`）与回写**行为完全一样**，A 段抓不到。
#     * D 段的假 UART 数据用光后 `read` 返回 None，`while True` 照样 `break`
#       ⇒ 不会挂住，D 段也抓不到"没有上界"。
#   所以这里把 A 段改成**两帧首尾相接**（尾帧跨块 ⇒ 不回写 rest 就永远解不出第二帧），
#   并给假 UART 加一个 `reads` 计数、D 段断言**每拍读的块数 ≤ max_reads**。
#   现在两条变异都真红。
import sys
sys.path.insert(0, "/sdcard/k230vision")

import a010_frame as af
import tof as T

FAILS = []


def chk(cond, label):
    if not cond:
        FAILS.append(label)
        print("  FAIL  %s" % label)


class FakeUart:
    """按块吐出预置字节；空了就返回 None（模拟"没有新数据"）。

    记 `reads`：用来断言 pump 每拍最多读 `max_reads` 块（非阻塞的关键）。
    """
    def __init__(self, chunks):
        self.chunks = list(chunks)
        self.reads = 0

    def read(self, n):
        self.reads += 1
        if not self.chunks:
            return None
        return self.chunks.pop(0)


class FakeDev:
    def __init__(self, chunks):
        self.uart = FakeUart(chunks)


print("=" * 74)
print("A) 两帧首尾相接、跨 10 字节块到达：必须解出 2 帧，且 latest 是第二帧")
# 第二帧跨块到达 —— 若 feed 的 remainder 不回写 self.buf（`fr, _ = ...`），
# 第一帧解出后残字节被丢，第二帧**永远解不出来**（还会反复重解第一帧）。
frame_bytes = af.build_frame(4, 4, bytes([50]) * 16, frame_id=1)
frame2_bytes = af.build_frame(4, 4, bytes([60]) * 16, frame_id=2)
stream = frame_bytes + frame2_bytes
chunks = [stream[i:i + 10] for i in range(0, len(stream), 10)]
r = T.TofReader(FakeDev(chunks))
n = 0
for _ in range(len(chunks) + 2):
    n += r.pump(max_reads=1)
chk(n == 2, "两条首尾相接的帧最终应解出 2 帧（实得 %d）" % n)
chk(r.latest_frame is not None and r.latest_frame["frame_id"] == 2,
    "latest_frame 应是 id=2（实得 %s）"
    % (r.latest_frame and r.latest_frame["frame_id"]))

print()
print("B) 没有数据时 pump 立刻返回 0，不空转")
r2 = T.TofReader(FakeDev([]))
chk(r2.pump() == 0, "空 UART 的 pump 应返回 0")

print()
print("C) 坏帧之后能重同步到后面那帧好帧")
bad = af.build_frame(4, 4, bytes([50]) * 16, frame_id=9, corrupt_checksum=True)
good = af.build_frame(4, 4, bytes([60]) * 16, frame_id=2)
r3 = T.TofReader(FakeDev([bad + good]))
r3.pump(max_reads=1)
chk(r3.latest_frame is not None and r3.latest_frame["frame_id"] == 2,
    "坏帧应被丢、重同步到 id=2（实得 %s）"
    % (r3.latest_frame and r3.latest_frame["frame_id"]))

print()
print("D) bail out 的上界：一块坏数据不能把 pump 拖成无限循环")
d4 = FakeDev([b"\x00" * 4096] * 100)
r4 = T.TofReader(d4)
n4 = r4.pump(max_reads=4)          # 只读 4 次就该返回
chk(n4 == 0, "全是垃圾的 pump 应返回 0 而不是挂住")
chk(d4.uart.reads <= 4,
    "pump 每拍最多读 max_reads 块（实得读了 %d 次）" % d4.uart.reads)

print()
print("★ 变异锚点（这两条都必须真红，已实测）")
print("  1) pump 的 for 换成 'while True'（去掉 max_reads 上界）⇒ D 段红"
      "（reads 远超 max_reads；真硬件上会直接挂住）")
print("  2) pump 里 feed 的 rest 丢掉（`fr, _ = ...`，不回写 self.buf）⇒ A 段红"
      "（第二帧解不出来、latest 停在 id=1）")
print()
print("=" * 74)
if FAILS:
    print("结果: %d 项失败" % len(FAILS))
    for x in FAILS:
        print("   -", x)
else:
    print("结果: 全部通过")
print("READER_DONE")
