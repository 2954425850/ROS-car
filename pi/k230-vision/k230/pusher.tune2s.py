# /sdcard/k230vision/pusher.py
"""裸 H.264 over TCP 推流。

## 丢帧策略（重要，别写错）

H.264 的 P 帧有参考链。**发半帧、或丢单个 NAL，会污染整条流直到下一个 I 帧。**
所以本模块的铁律是：

    一整帧要么整帧发完，要么整帧不发。

实现：阻塞发送 + 每帧总超时。若某帧超时，说明 socket 已经堵了，
**立刻断开重连**（而不是继续硬塞）。重连后**没有"请求 IDR"这条路**，
只能等 GOP=25 的下一个 IDR 自愈（本平台无请求 IDR 的接口，见「恢复边界」一节）。
配合 GLOBAL GOP=25（1 秒），任何损伤 1 秒内恢复。

## 断线期间也必须继续 GetStream（Task 3 加，重要）

接收端消失时**不能只是不 pump()**：VENC 的输出队列（`SetOutBufs(chn, 8, ...)`
只有 8 个缓冲）会被填满，MPP 侧没有可用的输出缓冲。所以第二条铁律是：

    连接断了也要把码流取出来 Release 掉（丢弃），保持 MPP 流动。

`pump()` 因此在**未连接时也走 GetStream->ReleaseStream**，把这帧计入
`frames_dropped`。这就是"要么立刻发走，要么丢弃"，与 GLOBAL 约束一致：绝不排队。

## 为什么不缓冲

MicroPython 堆只有约 3.9 MB。网络一堵就排队 = 内存爆 = 板子崩。
宁掉帧，不排队。

## 恢复边界（Task 3 查明，别越界）

本模块**只做 TCP 层重连，不重建 MPP 管线**。同一 MicroPython 会话内拆掉管线
再建第二条**必报** `MediaManager link failed(9)`（连 cleanup 完整也一样，见 NOTES.md），
重建管线只有 `machine.reset()` 一条路。所以：

- 接收端消失/回来 -> 只重连 socket，**预期可行**，本模块负责；
- 推流失败就"重建管线" -> 本平台**不存在**这条路，不要写。

重连后靠 `gop_len=25`（30fps 下 <1 s）等下一个 IDR 让新接收端出画。
本平台 **Encoder 没有请求 IDR 的接口**（`dir(Encoder)` 只有 SetOutBufs /
Create / Start / Stop / Destroy / GetStream / ReleaseStream / SendFrame），
无法主动请求关键帧，只能等 GOP。

## 为什么不把超时判据改软（2026-09-17 决策：**否决**，别再提）

`_send_frame` 里"单帧 0.5 s send 超时 -> 立刻断线重连"看起来像个可以优化的地方
（"链路只是慢，何必断？改成连续 N 帧超时才判 LOST 岂不更好"）。
**这条优化在本平台是被认真否决过的**，理由如下，请勿再提：

1. **超时后重试同一帧会污染字节流。** TCP `send()` 超时后**无法知道它到底写进去了
   多少字节**（可能 0，也可能部分已进内核缓冲并**终将送达对端**）。重试同一帧
   = 把已经写进去的前 k 个字节再发一遍，接收端拿到的流就坏了，
   而且**要坏到下一个 IDR 才可能恢复**。所以唯一安全的动作就是**断线重连** ——
   这正是现在的行为。
2. **"连续 N 帧超时"等于把"硬塞"写进代码。** 留着一条已经堵住的连接只会**推迟**
   恢复，与设计约束（"宁掉帧，不排队"、"发现 socket 堵了就断开重连"）正好相反。
3. **0.5 s 不是"轻微卡顿"的阈值，是个很宽的阈值。** 单帧约 8.5 KB，0.5 s 发完
   相当于 **17 KB/s ≈ 136 kbps**。要触发超时，链路吞吐得掉到 136 kbps 以下 ——
   那是**严重劣化**，不是抖动。实测那两次端到端只有 0.161 / 1.155 Mbps 时
   打印的 `PUSH LOST` 是**准确报告**，不是误报。
4. **代价可接受。** 一次（哪怕是被误判的）断线最多损失
   `回退(<=1 s) + 等 GOP(<=0.83 s) ≈ 1.8 s` 的画面。

（同一决策在 NOTES.md「Task 3」一节有更长的记录。）
"""
import socket
import uctypes
from media.vencoder import StreamData


class Pusher:
    # 2026-09-17 用户决定：0.5 -> 2.0 s。
    # 实测 30 分钟长稳里发生 ~57 次真实断连（平均 33 s 一次），每次都要等下一个 IDR
    # 才能恢复（gop=25 约 0.83 s）。链路抖动是断连的诱因之一，把单帧预算放宽到 2.0 s
    # 可以吸收抖动、减少假断连。策略不变：整帧发不完仍然断开重连、绝不重发半帧。
    # 注意：预算放大意味着「帧真的卡住」的发现晚 ~1.5 s；而对端进程死掉会立刻回 RST，
    # 本来就不靠这个超时发现。
    def __init__(self, host, port, enc, chn, frame_timeout_s=2.0):
        self.host = host
        self.port = port
        self.enc = enc
        self.chn = chn
        self.frame_timeout_s = frame_timeout_s
        self.sock = None
        self.frames_sent = 0
        self.frames_dropped = 0
        self.bytes_sent = 0
        self.lost_events = 0     # 断线次数（只在"刚发现连接坏了"时 +1，不是每轮 +1）
        self.connects = 0        # connect() 成功次数（含首次）

    # ---- 连接管理 ----
    def connect(self):
        self.close()
        try:
            s = socket.socket()
            s.settimeout(self.frame_timeout_s)
            s.connect((self.host, self.port))
            self.sock = s
            self.connects += 1
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

    # ---- 发一帧 ----
    def _send_frame(self, stream_data):
        """整帧发送。任何一份没发完就抛 OSError，由调用方断线重连。"""
        for i in range(stream_data.pack_cnt):
            buf = uctypes.bytearray_at(stream_data.data[i], stream_data.data_size[i])
            n = self.sock.send(buf)
            if n != len(buf):
                raise OSError("partial send: %d/%d" % (n, len(buf)))
            self.bytes_sent += n

    def pump(self):
        """取一帧编码码流并处理掉（发走，或丢弃）。

        返回：0 送出一帧；-1 当前没有码流；-2 未连接 / 连接刚故障

        **注意：未连接时也会 GetStream->Release**（丢弃）。否则接收端离开的
        这几秒里 VENC 的 8 个输出缓冲会被填满，把 MPP 管线顶住。
        """
        stream_data = StreamData()
        ret = self.enc.GetStream(self.chn, stream_data, timeout=100)
        if ret != 0:
            return -1

        rc = 0
        try:
            if not self.ok:
                # 断线期间：取出来立刻放掉，保持 MPP 流动（宁可丢弃，绝不排队）
                self.frames_dropped += 1
                rc = -2
            else:
                self._send_frame(stream_data)
                self.frames_sent += 1
        except OSError:
            # 帧已经发残缺，TCP 流不可恢复 -> 断线重连
            self.frames_dropped += 1
            self.lost_events += 1
            self.close()
            rc = -2
        finally:
            self.enc.ReleaseStream(self.chn, stream_data)
        return rc
