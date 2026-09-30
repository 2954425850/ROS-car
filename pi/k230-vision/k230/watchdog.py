# /sdcard/k230vision/watchdog.py
"""推流停滞看门狗。

判定依据是"有没有帧真的发出去"，不是"socket 在不在" ——
socket 连着但对端不收也会堵。

## 接口

    wd = Watchdog(stall_ms=3000, on_stall=None)
    wd.tick(p.frames_sent)     # 传**累计**已发帧数（单调不减）
    wd.stalled                 # True/False
    wd.idle_ms                 # 距上次有进展过了多少毫秒
    wd.reset()                 # 手工清零计时

注：计划 Task 3 的 Interfaces 一行写的是 `tick(sent_delta)`，但同一任务的
Step 4 代码和 Step 5 用法（`wd.tick(p.frames_sent)`）都是**累计值**。
这里按**累计值**实现 —— 它无歧义，且调用方本来就有 `frames_sent`。

## 为什么不用"距离上次 tick 的时间"

主循环即使卡在 `send()` 里也会一直 tick。只有 `sent` 这个数字涨了才算有进展，
所以计时基准是 `sent` 的**变化**，不是 tick 的调用频率。
"""
import time


class Watchdog:
    def __init__(self, stall_ms=3000, on_stall=None):
        self.stall_ms = stall_ms
        self.on_stall = on_stall
        self._last_progress = time.ticks_ms()
        self._last_total = 0
        self._fired = False
        self.stalls = 0          # 累计触发次数

    def tick(self, sent_total):
        """喂入累计已发帧数，返回 self.stalled。"""
        if sent_total != self._last_total:
            self._last_total = sent_total
            self._last_progress = time.ticks_ms()
            self._fired = False
        if self.stalled and not self._fired:
            # 只在"进入停滞"的那一次回调，不是每轮都回调
            self._fired = True
            self.stalls += 1
            if self.on_stall is not None:
                try:
                    self.on_stall()
                except Exception as e:      # 回调出错不能拖垮推流
                    print("watchdog: on_stall 回调抛异常", e)
        return self.stalled

    @property
    def stalled(self):
        return time.ticks_diff(time.ticks_ms(), self._last_progress) > self.stall_ms

    @property
    def idle_ms(self):
        return time.ticks_diff(time.ticks_ms(), self._last_progress)

    def reset(self):
        self._last_progress = time.ticks_ms()
        self._fired = False
