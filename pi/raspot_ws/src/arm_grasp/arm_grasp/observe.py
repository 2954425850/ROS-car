# -*- coding: utf-8 -*-
"""K230 侧观测：从结果流里挑出"我们要抓的那个框"，以及 8557 锁框客户端。

框是**归一化** `[l, t, r, b]`（左上原点，基准 1280x720 推流画面）—— 与板子 8557 的约定
逐字相同，所以 Pi 转发时**不需要任何坐标换算**（避开"通道/分辨率不同"那个静默坑）。

## 板子侧真身核对（2026-10-01，照 `5.K230/sdcard/k230vision/` 的真实代码）

`objs` 里有两类条目，字段**不一样**：

    检测器结果（vision.py 的 detect()）  {"cls": str, "score": float, "box": [l,t,r,b]}
                                         —— **没有 `src` 键**
    跟踪器结果（app.py 里拼的 t_objs）   {"cls": TRACK_CLASS, "score": float,
                                         "box": [l,t,r,b], "track_id": 1, "src": "track"}

⇒ 判"这是跟踪框"写 `o.get('src') == 'track'`；**不要写 `o['src']`** —— 检测框根本没有
   这个键（会 KeyError）。`track_id` 目前恒为 `1`（跟踪器一次只跟一个目标）。

⚠️ 跟踪器的 `lost` / `ar_dev` / `degenerate` **不会发出来**：app.py 只在
   `not tk["lost"]` 时才 append，漂了/退化的那一拍整条不发。所以下游拿不到这两个字段 ——
   要判框好不好，只能靠 `pick_box` 的连续性，别指望 `b['lost']` / `b['ar_dev']`。

`box` 都是 `results.normalize()` 出来的、已**夹紧到 0~1**：贴边的框不是被丢弃而是被裁平。
跟踪器另有一层 `tracker.init()` 的 `box_window_fits`（拒绝窗口出画的初始框）——
所以从 Pi 看到的框**只会被夹平、不会越界**。
"""
import json
import os
import socket
import time

from . import geom

RESULT_PATH = '/tmp/k230/latest-result.json'


def _box(o):
    return o.get('box')


def iou(a, b):
    """两个归一化 [l,t,r,b] 的 IoU。不相交返回 0.0（不返回负值）。"""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return 0.0 if ua <= 0 else inter / ua


def pick_box(result, want_norm=None, iou_min=0.2):
    """挑框。规则：

    1) 没给 `want_norm`：**优先 `src == 'track'`**（跟踪器给的连续框 —— 检测器在
       场景移动时只有 36.7% 命中），在跟踪框里取面积最大的那个；一个跟踪框都没有
       才退回全部框。
    2) 给了 `want_norm`（人画的框）：先看跟踪框里有没有与它 IoU ≥ `iou_min` 的 ——
       有就**只在跟踪框里挑**（跟踪框连续、抖动小）；一个都没有（人指的可能是
       检测器才认得的那个）→ **退回全部框**再挑。两种情况下都取 IoU 最大者。
       全都不达 `iou_min` ⇒ **返回 None**（"没找到"，不是"随便给一个"）。
    """
    objs = [o for o in (result.get('objs') or []) if _box(o)]
    if not objs:
        return None
    tracks = [o for o in objs if o.get('src') == 'track']
    if want_norm is not None:
        pool = [o for o in tracks if iou(_box(o), want_norm) >= iou_min] or objs
        scored = [(iou(_box(o), want_norm), o) for o in pool]
        scored = [s for s in scored if s[0] >= iou_min]
        if not scored:
            return None
        return max(scored, key=lambda t: t[0])[1]
    pool = tracks or objs
    return max(pool, key=lambda o: (_box(o)[2] - _box(o)[0]) * (_box(o)[3] - _box(o)[1]))


def box_center_ai(box_norm):
    """归一化框中心 → **AI 帧（320x180）像素**。

    归一化的基准是推流画面 1280x720，而 AI 帧 320x180 是它的**均匀 ÷4**（FOV 不变）
    ⇒ 走 `geom.to_ai` 就够。**别在这里手写 ÷4**：那会把"以后有人改了通道/分辨率"
    变成一处静默错误（见 [[calib-match-consumer-channel]]）。
    """
    u = 0.5 * (box_norm[0] + box_norm[2]) * geom.STREAM_W
    v = 0.5 * (box_norm[1] + box_norm[3]) * geom.STREAM_H
    return geom.to_ai(u, v)


def fresh_result(path=RESULT_PATH, max_age_s=1.5):
    """读结果文件并判新鲜度（消费方用 mtime 判，见 collect.read_result）。

    文件不在 / 太旧 / 内容坏了，一律返回 None —— 调用方只判 `is None`。
    """
    try:
        age = time.time() - os.path.getmtime(path)
    except OSError:
        return None
    if age > max_age_s:
        return None
    try:
        with open(path, 'r') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def k230_cmd(host, payload, port=8557, timeout=3.0):
    """8557：一行 JSON 一次连接。返回板子的应答 dict（`{"ok":true}` 或 `{"ok":false,"err":..}`）。

    **任何失败都返回 dict，绝不往上抛** —— 调用方（T9 的 finally）要靠它收尾，
    那里再抛异常就会把"释放跟踪器"这件事一起吞掉。
    """
    s = socket.socket()
    try:
        s.settimeout(timeout)
        s.connect((host, int(port)))
        s.sendall((json.dumps(payload) + '\n').encode())
        buf = b''
        while b'\n' not in buf and len(buf) < 4096:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        if not buf:
            return {'ok': False, 'err': '板子没有应答'}
        return json.loads(buf.split(b'\n')[0].decode())
    except (OSError, ValueError) as e:
        return {'ok': False, 'err': '8557 失败：%s' % e}
    finally:
        try:
            s.close()
        except OSError:
            pass
