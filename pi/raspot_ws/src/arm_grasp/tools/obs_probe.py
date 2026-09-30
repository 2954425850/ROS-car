# tools/obs_probe.py
# -*- coding: utf-8 -*-
"""只读观测探针 = **T7 + T8 的实机验收工具**：看 8556 的框、量速率/缺条率/抖动/框宽，并能锁框(8557)。

    python3 tools/obs_probe.py --seconds 20                    # 只看
    python3 tools/obs_probe.py --seconds 20 --lock 0.42,0.55,0.51,0.72   # 先锁这个框再观察

**不动臂、不动服务、不发 /arm/command。** 锁框只影响板子的跟踪状态，跑完自动 `{"cmd":"stop"}`。

判据（**能红**）：
  * **缺条率 ≤5%** —— 板子跟丢时**整条不发**（`app.py` 只在 `not tk["lost"]` 时 append）⇒ 缺条即丢失；
  * **框心抖动 ≤3px RMS**（AI 帧 320×180 系）；
  * **框宽膨胀 ≤1.5×**（记忆里实测：跟到背景时框宽 32→87，而 `score` 仍是 0.999 ⇒ 判漂移只能看框宽）。

⚠️ **别用 `lost` / `ar_dev`** —— 2026-10-01 实测板子**不发**这两个字段（它们是 `tracker.update()`
的内部量、没进 `t_objs`），写了恒为 `None`，那条判据**永远是绿的 = 假绿**。
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
_HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if not os.path.isdir('/home/cy/raspot_ws/src/arm_grasp') and os.path.isdir(_HERE):
    sys.path.insert(0, _HERE)

from arm_grasp import observe
from arm_grasp.collect import k230_host_from_result


def _std(vals):
    if len(vals) < 2:
        return 0.0
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / len(vals))


def main(argv=None):
    ap = argparse.ArgumentParser(description='只读观测探针（不动臂）')
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--want', default=None, help='用哪个框筛（l,t,r,b 归一化）；不给就按规则挑')
    ap.add_argument('--lock', default=None, help='先锁这个框再观察（l,t,r,b 归一化）')
    ap.add_argument('--min-present', type=float, default=0.95, help='有框帧占比下限（=1-缺条率）')
    ap.add_argument('--max-jitter', type=float, default=3.0, help='框心抖动上限（AI 帧 px）')
    ap.add_argument('--max-grow', type=float, default=1.5, help='框宽膨胀上限（倍）')
    args = ap.parse_args(argv)

    want = [float(v) for v in args.want.split(',')] if args.want else None
    host = k230_host_from_result()
    print('板子 %s' % host)

    lock_used = False
    if args.lock:
        box = [float(v) for v in args.lock.split(',')]
        print('锁框应答：', observe.k230_cmd(host, {'box': box}))
        lock_used = True
        time.sleep(0.5)

    t0, last, n, miss = time.time(), None, 0, 0
    pts, widths, classes, srcs = [], [], [], []
    try:
        while time.time() - t0 < args.seconds:
            r = observe.fresh_result()
            if r is None or r.get('frame') == last:
                time.sleep(0.02)
                continue
            last = r['frame']
            n += 1
            b = observe.pick_box(r, want)
            if b is None:
                miss += 1
                print('#%-5d **缺**（本帧 objs=%d）' % (r['frame'], len(r.get('objs') or [])))
                continue
            u, v = observe.box_center_ai(b['box'])
            w = b['box'][2] - b['box'][0]
            pts.append((u, v))
            widths.append(w)
            classes.append(b.get('cls'))
            srcs.append(b.get('src'))
            print('#%-5d %-12s src=%-5s box=[%.3f %.3f %.3f %.3f]  中心AI=(%5.1f,%5.1f)  宽%5.1fpx'
                  % (r['frame'], b.get('cls'), b.get('src'),
                     b['box'][0], b['box'][1], b['box'][2], b['box'][3], u, v, w))
            time.sleep(0.05)
    finally:
        pass

    dur = max(1e-6, time.time() - t0)
    print('\n================ 汇总 ================')
    print('时长 %.1fs   看到 %d 帧   上报速率 %.2f Hz' % (dur, n, n / dur))
    if not pts:
        print('❌ 一帧可用的框都没有')
        ok = False
    else:
        ju, jv = _std([p[0] for p in pts]), _std([p[1] for p in pts])
        jit = math.sqrt(ju * ju + jv * jv)
        grow = max(widths) / max(1e-9, widths[0])
        present = 1.0 - miss / float(max(1, n))
        print('类别 %s   src 集合 %s' % (sorted(set(classes)), sorted(set(srcs))))
        print('有框帧占比 %.1f%%（缺 %d 帧）   框心抖动 %.2f px(AI, u%.2f/v%.2f)   框宽 %.0f→%.0fpx（%.2f×）'
              % (present * 100, miss, jit, ju, jv, widths[0], max(widths), grow))
        ok = (present >= args.min_present and jit <= args.max_jitter
              and grow <= args.max_grow)
        print('判据：有框 ≥%.0f%% / 抖动 ≤%.1fpx / 膨胀 ≤%.2f× ⇒ %s'
              % (args.min_present * 100, args.max_jitter, args.max_grow,
                 '✅ 通过' if ok else '❌ 不通过'))
    if lock_used:
        print('释放跟踪器：', observe.k230_cmd(host, {'cmd': 'stop'}))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
