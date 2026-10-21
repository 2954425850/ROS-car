# -*- coding: utf-8 -*-
"""从 K230 拉一张画面 + 在画面里找一个"亮的小块"。标定仿射时用。

## ★ 两个踩过的坑，别改回去

1. **`latency` 必须放大**。`k230pi.py` 里 `build_snap_cmd` 用的是 `latency=200`
   + `drop-on-latency=true` —— 在**这块板子**上（WiFi ping 实测 **1762ms**！）
   gst 会把所有帧都判成"太迟"直接丢掉，于是只剩编码器在首个 IDR 之前输出的
   **纯灰占位帧**（实测 `std=0.25`；真实画面 `std≈20`）。
   ⇒ `latency=3000`、**去掉** `drop-on-latency`。VLC 一直能看到画面，就是 gst 这边丢的。

2. **板子 IP 要现查，不能写死**。`k230pi.py` 里 `K230_HOST = "192.168.1.112"`
   是完全失效的旧值；板子走 DHCP，每次重连都换 IP（实测一晚上 `.135→.109→.152`）。
   唯一可靠的来源是结果流文件里的 `_peer`（板子主动连 Pi 时留下的）。
"""
from __future__ import annotations

import io
import json
import os
import subprocess
import tempfile

RESULT_PATH = "/tmp/k230/latest-result.json"
DEFAULT_LATENCY = 3000


def board_host(path=RESULT_PATH):
    """从结果流的 `_peer` 取板子当前 IP。取不到返回 None。"""
    try:
        with open(path, encoding="utf-8") as f:
            peer = json.load(f).get("_peer")
        return peer.rsplit(":", 1)[0] if peer else None
    except Exception:                              # noqa: BLE001
        return None


def _last_jpeg(blob):
    """从"多帧 JPEG 首尾相接"的 blob 里切出最后一个完整的。"""
    end = blob.rfind(b"\xff\xd9")
    if end < 0:
        return None
    start = blob.rfind(b"\xff\xd8", 0, end)
    return blob[start:end + 2] if start >= 0 else None


def snap(host=None, latency=DEFAULT_LATENCY, seconds=20):
    """拉一张 JPEG，返回字节；失败返回 None。"""
    host = host or board_host()
    if not host:
        return None
    fd, path = tempfile.mkstemp(suffix=".bin")
    os.close(fd)
    cmd = ["gst-launch-1.0", "-e", "-q",
           "rtspsrc", "location=rtsp://%s:8554/k230" % host,
           "protocols=udp", "latency=%d" % latency,
           "!", "rtph264depay", "!", "h264parse",
           "!", "video/x-h264,stream-format=byte-stream,alignment=au",
           "!", "avdec_h264", "!", "videoconvert",
           "!", "video/x-raw,format=I420", "!", "jpegenc", "quality=88",
           "!", "filesink", "location=%s" % path]
    # ★ 不能用 `subprocess.run(timeout=...)`：它超时会**抛异常**，
    #   而我们根本没机会去读那个**已经写好**的文件（实测就是这样丢帧的）。
    #   正确做法：让它跑到时间 → SIGINT（`-e` 靠这个发 EOS 把 sink 刷干净）→ 再读。
    import signal
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        p.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=4)
        except subprocess.TimeoutExpired:
            p.kill()
    try:
        blob = open(path, "rb").read()
    except OSError:
        blob = b""
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass
    return _last_jpeg(blob)


def bright_blob(jpg_bytes, bright=150, max_sat=90, min_px=300):
    """在画面里找**最大的"又亮又不怎么彩色"的块**（浅色小球/白纸），
    返回 `(cx, cy, npx)`（归一化）或 None。

    为什么用"亮 + 低饱和"：木头桌面是**暗且偏黄**的（R>G>B、亮度低），
    浅色物体是**亮且接近无彩**的。这个判据对光线不敏感、也不用调阈值框颜色。
    """
    try:
        import numpy as np
        from PIL import Image
    except ImportError:
        return None
    im = Image.open(io.BytesIO(jpg_bytes)).convert("RGB")
    a = np.asarray(im).astype(int)
    mx = a.max(axis=2)
    mn = a.min(axis=2)
    mask = (mx >= bright) & ((mx - mn) <= max_sat)
    if mask.sum() < min_px:
        return None
    # 简易连通块（缩到 1/4 分辨率扫，够用且快）
    small = mask[::4, ::4]
    R, C = small.shape
    seen = small.copy()
    best = None
    for r0 in range(R):
        for c0 in range(C):
            if not seen[r0, c0]:
                continue
            stack = [(r0, c0)]
            seen[r0, c0] = False
            ys, xs = [], []
            while stack:
                r, c = stack.pop()
                ys.append(r); xs.append(c)
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < R and 0 <= cc < C and seen[rr, cc]:
                        seen[rr, cc] = False
                        stack.append((rr, cc))
            if best is None or len(ys) > best[0]:
                best = (len(ys), ys, xs)
    if best is None or best[0] * 16 < min_px:
        return None
    _, ys, xs = best
    h, w = a.shape[:2]
    # 用**外接框中心**而不是像素质心：球的下半部常有阴影、被判定为不够亮，
    # 质心会被往上拉；外接框中心对"部分像素漏判"稳得多。
    cx = (min(xs) + max(xs)) / 2.0 * 4 / w
    cy = (min(ys) + max(ys)) / 2.0 * 4 / h
    return (cx, cy, len(ys) * 16)
