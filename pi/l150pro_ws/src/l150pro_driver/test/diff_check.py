"""与固件侧的对拍：把刺激字节流喂进 Python 解码器，逐字段比对固件的期望值。

用法：
    python test/diff_check.py <stimulus.txt> <expected.txt>

stimulus: 每行一个十六进制字节流（空格分隔）
expected: 第 1 行表头，之后每行一个用例

字段（逗号分隔）：
    frames,crc_err,head_err,vx,vy,wz,flags,seq,sv_frames,sv_crc_err,s1..s6

vx/vy/wz 是【原始线值 int16】，不经浮点往返，所以可以判严格相等。
flags/seq/s1..s6 是最后一条成功解出的帧的值，无该类型帧时为 -1。
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from l150pro_driver.protocol import (  # noqa: E402
    MotionRxParser, ServoRxParser,
)

FIELDS = ["frames", "crc_err", "head_err", "vx", "vy", "wz", "flags", "seq",
          "sv_frames", "sv_crc_err", "s1", "s2", "s3", "s4", "s5", "s6"]


def parse_stimulus(path):
    cases = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            cases.append(bytes(int(t, 16) for t in line.split()))
    return cases


def parse_expected(path):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        lines = [ln.strip() for ln in f if ln.strip()]
    header = lines[0].split(",")
    if header != FIELDS:
        print(f"警告：期望文件的表头与脚本不一致\n  文件: {header}\n  脚本: {FIELDS}")
    for ln in lines[1:]:
        rows.append([int(x) for x in ln.split(",")])
    return rows


def run_case(data: bytes):
    m = MotionRxParser()
    s = ServoRxParser()
    m.feed(data)
    s.feed(data)

    vx, vy, wz, flags, seq = m.last if m.last else (-1, -1, -1, -1, -1)
    if s.last:
        us, sseq = s.last
        s1, s2, s3, s4, s5, s6 = us
        sv_seq = sseq
    else:
        s1 = s2 = s3 = s4 = s5 = s6 = -1
        sv_seq = -1

    # expected 里没有 sv_seq，用不到，仅内部保留
    _ = sv_seq
    return [m.frames, m.crc_err, m.head_err, vx, vy, wz, flags, seq,
            s.frames, s.crc_err, s1, s2, s3, s4, s5, s6]


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)

    stimulus = parse_stimulus(sys.argv[1])
    expected = parse_expected(sys.argv[2])
    n = min(len(stimulus), len(expected))

    if len(stimulus) != len(expected):
        print(f"注意：刺激 {len(stimulus)} 条，期望 {len(expected)} 条，只比对前 {n} 条")

    bad = []
    for i in range(n):
        got = run_case(stimulus[i])
        exp = expected[i]
        if got != exp:
            diff = [(FIELDS[j], exp[j], got[j])
                    for j in range(len(FIELDS)) if got[j] != exp[j]]
            bad.append((i, diff, stimulus[i]))

    print(f"对拍完成：{n} 个用例，{len(bad)} 个不一致")
    if bad:
        print()
        for i, diff, data in bad[:25]:
            hx = data.hex(" ")
            if len(hx) > 90:
                hx = hx[:90] + f" ...(共{len(data)}字节)"
            print(f"--- 用例 #{i}  字节流: {hx}")
            for name, e, g in diff:
                print(f"      字段 {name}: 固件={e}  Python={g}")
        if len(bad) > 25:
            print(f"\n…… 还有 {len(bad) - 25} 个不一致未显示")
        sys.exit(1)
    print("两侧完全一致 —— 协议实现对齐")


if __name__ == "__main__":
    main()
