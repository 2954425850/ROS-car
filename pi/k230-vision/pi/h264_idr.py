"""从 H.264 Annex-B 字节流里抠出「最新一张关键帧」（SPS + PPS + IDR），不解码。

给 `k230ctl framed` 用：板子把 720p H.264 推到 Pi 的 8555，这里只扫起始码、
看 NAL 类型，把每个 GOP 开头那张 IDR 连同参数集攒成一个独立可解的小码流。
要照片时只解这一张（~0.2s），平时 CPU 只花在字节扫描上。

NAL 类型（低 5 位）：1 = 非 IDR 片，5 = IDR 片，7 = SPS，8 = PPS，9 = AUD。
"""

from __future__ import annotations

SC = b"\x00\x00\x01"


class IdrExtractor:

    def __init__(self, max_buf: int = 4 * 1024 * 1024):
        self._buf = bytearray()
        self._sps = None
        self._pps = None
        self._idr = []           # 当前这张 IDR 的片
        self.max_buf = max_buf
        self.frames = 0          # 完整取出的关键帧数

    def feed(self, data: bytes) -> list[bytes]:
        """喂字节，返回这次新完成的关键帧码流（每个都是 SPS+PPS+IDR，Annex-B）。"""
        self._buf += data
        out = []
        # 只处理到"最后一个起始码之前"，最后一段可能还没收完
        last = self._buf.rfind(SC)
        if last <= 0:
            if len(self._buf) > self.max_buf:
                self._buf.clear()
            return out
        start = self._buf.find(SC)
        while start != -1 and start < last:
            nxt = self._buf.find(SC, start + 3)
            if nxt == -1 or nxt > last:
                nxt = last
            nal = bytes(self._buf[start + 3:nxt]).rstrip(b"\x00")  # 去掉下一个 4 字节起始码的前导 0
            done = self._nal(nal)
            if done is not None:
                out.append(done)
            start = nxt if nxt < last else -1
        del self._buf[:last]
        return out

    def _nal(self, nal: bytes):
        if not nal:
            return None
        t = nal[0] & 0x1F
        if t == 7:
            self._sps = nal
        elif t == 8:
            self._pps = nal
        elif t == 5:
            self._idr.append(nal)
        if self._idr and t != 5:
            # IDR 后面来了别的 NAL ⇒ 这张 IDR 的片收齐了
            return self._finish()
        return None

    def _finish(self):
        idr, self._idr = self._idr, []
        if self._sps is None or self._pps is None:
            return None
        self.frames += 1
        parts = [self._sps, self._pps] + idr
        return b"".join(b"\x00\x00\x00\x01" + p for p in parts)
