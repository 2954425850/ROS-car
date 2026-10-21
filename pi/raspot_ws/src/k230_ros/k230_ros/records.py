# -*- coding: utf-8 -*-
"""K230 结果记录的纯函数解析（无 ROS 依赖）。"""


def detections_from_record(rec):
    """记录 → [(class_id, score, cx, cy, w, h, id), ...]（像素）。纯函数，便于测。"""
    W = float(rec.get('w') or 1280)
    H = float(rec.get('h') or 720)
    out = []
    for o in rec.get('objs') or []:
        box = o.get('box')
        if not box or len(box) != 4:
            continue
        l, t, r, b = (float(v) for v in box)
        out.append((str(o.get('cls', '')), float(o.get('score') or 0.0),
                    (l + r) * 0.5 * W, (t + b) * 0.5 * H, (r - l) * W, (b - t) * H,
                    'track' if o.get('src') == 'track' else ''))
    return out
