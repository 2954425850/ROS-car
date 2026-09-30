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
"""
import socket


class Reporter:
    def __init__(self, host, port, send_timeout_s=0.2):
        self.host = host
        self.port = port
        self.send_timeout_s = send_timeout_s
        self.sock = None
        self.sent = 0
        self.dropped = 0

    def connect(self):
        """建连。失败返回 False（不抛异常，调用方每轮都可以直接调）。"""
        self.close()
        try:
            s = socket.socket()
            s.settimeout(self.send_timeout_s)
            s.connect((self.host, self.port))
            self.sock = s
            return True
        except OSError:
            self.sock = None
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
