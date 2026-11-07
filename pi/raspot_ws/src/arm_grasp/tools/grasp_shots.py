# -*- coding: utf-8 -*-
"""看预合爪快照（`arm_grasp/shot.py` 存的那些 json + jpg）。

    grasp_shots.py                       # 在 Pi 上直接跑（默认目录）
    grasp_shots.py --dir ~/shots --overlay --out /tmp/overlay
    grasp_shots.py --dir ~/shots --json  # 原始逐条，给别的脚本吃

每行一条：结果码 / 横向残差 / 落点总偏差 / 沿轴 / 径向 / 冻结后 O 还会漂多少 / 像素残差。

**判读**（毫米，基座系）
  lat  横向（垂直夹爪轴）—— 用户抱怨的"不固定方向偏移"就是这个；
  tot  爪尖 → 物体真值 的总距离；
  dO   进 close 之后若继续更新，O 还会被拉走多少 —— **冻结的代价**；
  ax   沿轴（+ = 目标比爪尖更深）。
  `dO` 大 ⇒ 冻结太早 / 观测太脏（该停稳了再看一眼）；
  `dO` 小、`tot` 大 ⇒ 冻结之后臂没走到位（回差 / 下垂 / 底座滞后）。

⚠️ 这几个数都是用**同一套几何模型**（FK + 手眼 + 内参）算的 —— 模型偏了它也偏。
   所以 `--overlay` 出来的图必须看：图上红线是"模型认为物体该在这儿"，
   人眼能直接判"爪尖实际上是不是套住了它"。
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from arm_grasp import geom, shot                      # noqa: E402


def _table(d):
    recs = shot.load_all(d)
    if not recs:
        print('没有快照：%s' % os.path.expanduser(d))
        return 1
    print('时间                 结果            ' + ' '.join(
        ['%6s' % k for k in ('lat', 'tot', 'ax', 'rad', 'dO', 'px[0]', 'px[1]')]))
    for r in recs:
        dev = shot.deviation(r)
        res = r.get('result') or {}
        row = [res.get('code') or '?']
        for k in ('lat', 'd_tip', 'axial', 'radial', 'd_O'):
            v = dev.get(k)
            row.append('%6.1f' % (v * 1000.0) if v is not None else '     -')
        px = dev.get('px')
        row.append('%6.1f' % px[0] if px else '     -')
        row.append('%6.1f' % px[1] if px else '     -')
        print('%-20s %-14s %s' % (r.get('t_wall') or '?', row[0], ' '.join(row[1:])))
    print()
    agg = shot.aggregate(recs)
    for g in ('all', 'ok'):
        a = agg.get(g) or {}
        if not a.get('n'):
            continue
        parts = ['%-4s n=%-3d' % (g, a['n'])]
        for k, name in (('lat', '横向'), ('d_tip', '落点'), ('d_O', 'O漂'), ('axial', '沿轴')):
            if k in a:
                parts.append('%s p50=%4.1f p95=%4.1f' % (
                    name, a[k]['p50'] * 1000.0, a[k]['p95'] * 1000.0))
        print('  ' + ' | '.join(parts))
    return 0


def _overlay(d, out):
    try:
        import cv2
        import numpy as np
    except ImportError:
        print('没有 cv2，跳过画图')
        return 0
    os.makedirs(out, exist_ok=True)
    n = 0
    for r in shot.load_all(d):
        img_name = r.get('image')
        if not img_name:
            continue
        src = os.path.join(os.path.expanduser(d), img_name)
        img = _imread(src)
        if img is None:
            print('  读不了图：%s' % src)
            continue
        h, w = img.shape[:2]
        sx, sy = w / float(geom.STREAM_W), h / float(geom.STREAM_H)
        o = r.get('obs') or {}
        if o.get('box'):
            b = [int(o['box'][0] * geom.STREAM_W * sx), int(o['box'][1] * geom.STREAM_H * sy),
                 int(o['box'][2] * geom.STREAM_W * sx), int(o['box'][3] * geom.STREAM_H * sy)]
            cv2.rectangle(img, (b[0], b[1]), (b[2], b[3]), (255, 160, 0), 2)
        dev = shot.deviation(r)
        j = shot._joints(r.get('f_fb'))          # noqa: SLF001 —— 同一模块族的内部量
        if j is not None and dev.get('obj'):
            try:
                u, v = geom.project(j, dev['obj'])
                us, vs = geom.to_stream(u, v)
                cv2.circle(img, (int(us * sx), int(vs * sy)), 7, (0, 0, 255), -1)
                ua, va = geom.aim_pixel(closed=False)
                ug, vg = geom.to_stream(ua, va)
                cv2.drawMarker(img, (int(ug * sx), int(vg * sy)), (0, 255, 0),
                               cv2.MARKER_CROSS, 24, 2)
                cv2.line(img, (int(us * sx), int(vs * sy)),
                         (int(ug * sx), int(vg * sy)), (0, 255, 0), 1)
            except ValueError:
                pass
        _label(img, shot.summarize(r), (6, 26))
        stem = os.path.splitext(img_name)[0]
        _imwrite(os.path.join(out, stem + '-ov.jpg'), img)
        _imwrite(os.path.join(out, stem + '-ov-up.jpg'), cv2.rotate(img, cv2.ROTATE_180))
        n += 1
    print('画了 %d 张 -> %s' % (n, out))     # 不用 ⇒ ：Windows 控制台 GBK 编不了
    return 0


def _label(img, text, org):
    import cv2
    # 中文在 OpenCV 里会变问号：只保留 ASCII 部分（结果码/数字够用）
    ascii_text = ''.join(c if ord(c) < 128 else ' ' for c in text)
    cv2.putText(img, ascii_text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, ascii_text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 1, cv2.LINE_AA)


def _imread(path):
    import cv2
    import numpy as np
    try:
        return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    except (OSError, ValueError):
        return None


def _imwrite(path, img):
    import cv2
    import numpy as np
    ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if ok:
        buf.tofile(path)
    return ok


def main():
    ap = argparse.ArgumentParser(description='看预合爪快照')
    ap.add_argument('--dir', default=os.path.expanduser('~/.local/share/arm_shots'))
    ap.add_argument('--overlay', action='store_true', help='把框/投影画到照片上')
    ap.add_argument('--out', default='/tmp/arm_shots_overlay')
    ap.add_argument('--json', action='store_true', help='原样打印每条 json')
    args = ap.parse_args()
    if args.json:
        for r in shot.load_all(args.dir):
            print(json.dumps({k: v for k, v in r.items()}, ensure_ascii=False))
        return 0
    rc = _table(args.dir)
    if args.overlay:
        _overlay(args.dir, args.out)
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
