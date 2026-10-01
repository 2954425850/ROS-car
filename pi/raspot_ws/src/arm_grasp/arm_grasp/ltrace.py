# -*- coding: utf-8 -*-
"""按时刻查关节状态：环形缓冲 + 线性插值。

为什么需要：/arm/feedback 25Hz（2026-10-01 起，原 10Hz）、指令 10Hz、K230 的框 6~10Hz，三者不同频；
要在"拍那张图那一刻"的关节状态上做几何，就必须能按时刻查历史。
板子的 ts 只有秒级（平台限制），所以时效基准一律是 **Pi 侧的到达时刻**。
"""
import collections

Sample = collections.namedtuple('Sample', 't cmd fb')


def _ok(fields):
    """这一拍能不能用：长度够，且 p3/p4/p5 都不是 0（0 = 该拍没读到）。"""
    return (fields is not None and len(fields) >= 6
            and all(fields[i] != 0.0 for i in (2, 3, 4)))


class Trace:
    """(时刻, 指令, 回读) 的环形缓冲，按时刻线性插值。"""

    def __init__(self, n=600):
        self.buf = collections.deque(maxlen=n)

    def add(self, t, cmd, fb):
        self.buf.append(Sample(float(t),
                               None if cmd is None else [float(v) for v in cmd],
                               None if fb is None else [float(v) for v in fb]))

    def __len__(self):
        return len(self.buf)

    def span(self):
        if not self.buf:
            return (None, None)
        return (self.buf[0].t, self.buf[-1].t)

    def at(self, t, source='auto'):
        """t 时刻的 6 个 field。越界夹到最近端。

        source: 'cmd' 只信指令；'fb' 只信回读（不可用就抛）；'auto' 每端优先回读，
        该拍回读不可用（含 0）时退回指令。
        """
        if not self.buf:
            raise ValueError('Trace 是空的')
        if source == 'auto':
            def pick(s):
                return s.fb if _ok(s.fb) else s.cmd
        elif source == 'fb':
            def pick(s):
                if not _ok(s.fb):
                    raise ValueError('该拍没有可用回读（p3/p4/p5 有 0）')
                return s.fb
        elif source == 'cmd':
            def pick(s):
                return s.cmd
        else:
            raise ValueError('source 只能是 auto/fb/cmd')

        lo, hi = self.buf[0], self.buf[-1]
        if t <= lo.t or len(self.buf) == 1:
            return list(pick(lo))
        if t >= hi.t:
            return list(pick(hi))
        for i in range(len(self.buf) - 1, 0, -1):
            b, a = self.buf[i], self.buf[i - 1]
            if a.t <= t <= b.t:
                ja, jb = pick(a), pick(b)
                if ja is None or jb is None:
                    raise ValueError('该时刻两侧至少一端没有可用值')
                dt = b.t - a.t
                if dt <= 1e-9:
                    return list(jb)
                r = (t - a.t) / dt
                return [ja[k] + r * (jb[k] - ja[k]) for k in range(6)]
        raise AssertionError('不可达')
