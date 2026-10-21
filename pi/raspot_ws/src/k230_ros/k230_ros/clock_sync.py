# -*- coding: utf-8 -*-
"""K230 板子 ticks_ms ↔ Pi time.monotonic() 的换算（纯 Python，无 ROS 依赖）。

## 为什么需要
板子 `time.time()` 只有秒级精度（app.py 文件头第 3 条），没法拿墙钟对齐。
板子有单调毫秒 `ticks_ms`，Pi 有 `time.monotonic()`，两个时钟**不同源**、只差一个
（缓慢漂移的）常数偏差。估出这个偏差，就能把"板子在 cap_tick 时拍的这帧"
换算成 Pi 单调时刻，再去 ToF / 手臂回读的缓冲里找同一时刻的数据。

## 怎么估：窗口内取最小值
每条结果给一个样本 `rx_mono - tick/1000 = 偏差 + 这条消息的传输延迟`。
延迟 ≥ 0 且 WiFi 下抖动很大（几 ms 到几百 ms），所以**取窗口内最小值**：
最快的那条消息最接近"纯偏差 + 最小延迟"。平均值会被慢包拖偏，不能用。
残余误差 = 最小传输延迟（局域网 TCP，几 ms 量级），对 10Hz 的帧够用。

## 重置
- tick 倒退：板子重启（或 ticks_ms 回绕）⇒ 偏差整个变了，清空窗口重来。
- 新样本比当前最小值小很多（> `jump_s`）：同上，多半是板子重启后 tick 从小值重新数。
"""
from collections import deque

WINDOW_S = 20.0        # 窗口长度：两边晶振漂移 ~50ppm ⇒ 20s 内 ~1ms，可忽略
JUMP_S = 2.0           # 偏差突变超过这么多 ⇒ 认为板子重启，清窗
CAP_MAX_LAG_MS = 2000  # cap_tick 比 tick 早这么多以上 ⇒ 不信（推理不可能这么久）


class TickClock:

    def __init__(self, window_s=WINDOW_S, jump_s=JUMP_S):
        self.window_s = window_s
        self.jump_s = jump_s
        self._win = deque()          # (rx_mono, offset)
        self._last_tick = None
        self.resets = 0

    def feed(self, tick_ms, rx_mono):
        """喂一条：板子发送前的 tick（ms）+ Pi 收到它的单调时刻（s）。"""
        off = rx_mono - tick_ms / 1000.0
        cur = self.offset()
        if self._last_tick is not None and (
                tick_ms < self._last_tick
                or (cur is not None and abs(off - cur) > self.jump_s)):
            self._win.clear()
            self.resets += 1
        self._last_tick = tick_ms
        # 单调队列：只留"比后来者都小"的样本 ⇒ 队首就是窗口最小值
        while self._win and self._win[-1][1] >= off:
            self._win.pop()
        self._win.append((rx_mono, off))
        while self._win and rx_mono - self._win[0][0] > self.window_s:
            self._win.popleft()

    def offset(self):
        """当前偏差（s），没数据返回 None。mono = tick/1000 + offset。"""
        return self._win[0][1] if self._win else None

    def to_mono(self, tick_ms):
        off = self.offset()
        return None if off is None else tick_ms / 1000.0 + off

    def capture_mono(self, rec):
        """一条结果记录（dict，带 tick / cap_tick / _rx_mono）的**拍摄时刻**（Pi 单调秒）。

        返回 (mono, source)：source = 'cap_tick' | 'tick' | 'rx'，说明用的是哪个量。
        老固件没有 cap_tick ⇒ 退到 tick（晚一个推理时长）；连 tick 都没有 ⇒ 用收到时刻。
        """
        rx = rec.get('_rx_mono')
        tick = rec.get('tick')
        if tick is not None and rx is not None:
            self.feed(tick, rx)
        if tick is None or self.offset() is None:
            return rx, 'rx'
        cap = rec.get('cap_tick')
        if cap is not None and 0 <= tick - cap <= CAP_MAX_LAG_MS:
            return self.to_mono(cap), 'cap_tick'
        return self.to_mono(tick), 'tick'
