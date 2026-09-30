# /sdcard/k230vision/reporter.py
"""结构化结果上报。

## 为什么不排队

结果只有几 KB/s，但网络可能瞬断。排队同样会吃掉 MicroPython 那 3.9 MB 堆。
发不出去就丢弃这一条 —— 结果流是"最新的才有意义"，不是可靠传输。

## 为什么要检查"短写"（与计划草稿的差别）

消息用换行分隔（NDJSON）。TCP 的 `send()` 返回**实际写进去的字节数**，
短写会让接收端把两条消息粘成一条坏行，**之后每一条都错位**。
所以短写按"发不出去"处理：计 dropped、**关掉连接**，绝不重发。
（理由与 NOTES.md 里 H.264「绝不重发半帧」同源：重发已经写进去的前缀，
接收端拿到的就是坏流。这里坏的是行边界。）

## 重连必须节流（2026-10-01 加，**这条是会拖死主循环的**）

调用方的惯用写法是**每轮都调**：

    if not r.ok:
        r.connect()
    r.send(payload)

对端一直在听时这是免费的 —— `ok` 为真，`connect()` 一次都不进（`r_pi`/8556 就是这种）。
**但对端不在时它不是免费的**：一次 `connect()` 要跑满 `send_timeout_s`。
实测（2026-10-01，车上）：
  - 目标主机不可达 → **跑满 0.2 s**；
  - 网关不可达（PUSH 那次） → **跑满 0.5 s**。

主循环目标是 10 Hz（100 ms/轮）。每轮白付 0.2 s ⇒ 实际节拍掉到 **0.6 Hz**
—— 推理、人脸、跟踪全跟着一起慢 16 倍。这个数是在车上量到的，不是推算。

所以 `connect()` 失败后**在 `retry_ms` 内不真去连**，直接返回 False：
绝大多数轮次变成**一次 socket 系统调用都不做**。
"""
import socket
import time


class Reporter:
    def __init__(self, host, port, send_timeout_s=0.2, retry_ms=2000):
        self.host = host
        self.port = port
        self.send_timeout_s = send_timeout_s
        self.retry_ms = retry_ms
        self.sock = None
        self.sent = 0
        self.dropped = 0
        self._last_fail = None     # ticks_ms：上次建连失败的时刻（None = 还没失败过）
        self.connect_fails = 0     # **真的去连**且失败的次数（被节流跳过的不计）

    def connect(self):
        """建连。失败返回 False（不抛异常，调用方每轮都可以直接调）。

        ⚠️ **已经连着时不会重连**，直接返回 True —— 调用方的用法都是
        `if not r.ok: r.connect()`，所以这与原来的行为等价，还省掉一次无谓的
        close/connect。要强制重连就先 `close()`。

        失败后在 `retry_ms` 内**不真的去连**（见文件头"重连必须节流"）。
        """
        if self.sock is not None:
            return True
        now = time.ticks_ms()
        if self._last_fail is not None:
            if time.ticks_diff(now, self._last_fail) < self.retry_ms:
                return False          # 节流窗口内：一次 socket 调用都不做
        self.close()
        try:
            s = socket.socket()
            s.settimeout(self.send_timeout_s)
            s.connect((self.host, self.port))
            self.sock = s
            self._last_fail = None
            return True
        except OSError:
            self.sock = None
            self._last_fail = now
            self.connect_fails += 1
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    @property
    def ok(self):
        return self.sock is not None

    def send(self, payload):
        """payload 不含换行；本函数补上换行做为消息边界。

        **非阻塞语义**：单条消息最多占用 send_timeout_s。发不出去
        （未连接 / 超时 / 短写）就丢弃这一条并返回 False —— 不排队、不重发。
        """
        if not self.ok:
            self.dropped += 1
            return False
        msg = payload + b"\n"
        try:
            n = self.sock.send(msg)
        except OSError:
            self.dropped += 1
            self.close()
            return False
        if n != len(msg):
            # 短写：行边界已不可信，只能断开，不能重发
            self.dropped += 1
            self.close()
            return False
        self.sent += 1
        return True
