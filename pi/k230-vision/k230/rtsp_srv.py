# /sdcard/k230vision/rtsp_srv.py
"""把某个编码通道挂到原生 RTSP 服务器上（给局域网拉流）。

用 `multimedia.rtsp_server()` + `rtspserver_sendvideodata_byphyaddr`（按物理地址发，
零拷贝）。这是官方 `02-Media/rtsp_server.py` 和 `/sdcard/libs/WBCRtsp.py` 都验证过的路径。

## 为什么不用 _thread（与官方例程的唯一区别）

官方例程和 WBCRtsp 都用 `_thread.start_new_thread` 开推流线程。本项目**要单线程**：
MicroPython 的 `_thread` 与 MPP 的配合在本平台没验证过（设计文档 Task 7 有说明）。
所以这里由主循环调 `pump()` 推进。

**`rtspserver_sendvideodata_byphyaddr` 从主循环调用是否与官方线程版等效 —— 待实测。**
（Task 4 的板子在写这一版时已卡死，此项**未验证**。）

## 与官方例程的其它差异

- 官方用 `rtspserver_sendvideodata`（把数据拷成 `bytes` 再发，多一次拷贝）；
  这里用 `..._byphyaddr`（直接给物理地址），签名取自 WBCRtsp：
  `rtspserver_sendvideodata_byphyaddr(session, phy_addr, data_size, timeout_ms)`。
- `width` / `height` 只作记录用途：`rtspserver_createsession` 不接受尺寸，
  SPS/PPS 是随码流走的，服务端不需要提前知道分辨率。
"""
import multimedia as mm
from media.vencoder import StreamData


class RtspOut:
    def __init__(self, enc, chn, port=8554, session="k230",
                 width=1280, height=720):
        self.enc = enc
        self.chn = chn
        self.port = port
        self.session = session
        # 仅记录用途，见文件头
        self.width = (width + 15) & ~15
        self.height = height
        self.srv = None
        self.running = False
        self.frames = 0
        self.packs = 0
        self.bytes_sent = 0
        self.empty = 0          # 调用 pump 但没取到码流的次数

    def start(self):
        self.srv = mm.rtsp_server()
        self.srv.rtspserver_init(self.port)
        self.srv.rtspserver_createsession(
            self.session, mm.multi_media_type.media_h264, False
        )
        self.srv.rtspserver_start()
        self.running = True

    def get_url(self):
        return self.srv.rtspserver_getrtspurl(self.session)

    def pump(self):
        """取一帧并交给 RTSP 服务器。

        返回：0 送出一帧；-1 当前没有码流（或未启动）。

        **未连接/没有客户端时也要继续 GetStream->Release**，否则 VENC 的
        8 个输出缓冲会被填满，把 MPP 管线顶住（同 Task 3 在 pusher.py 里的教训）。
        这里的 GetStream 用非阻塞 timeout=0，取不到就返回 -1，帧留在队列里给下一轮。
        """
        if not self.running:
            return -1
        sd = StreamData()
        ret = self.enc.GetStream(self.chn, sd, timeout=0)
        if ret != 0:
            self.empty += 1
            return -1
        try:
            for i in range(sd.pack_cnt):
                self.srv.rtspserver_sendvideodata_byphyaddr(
                    self.session, sd.phy_addr[i], sd.data_size[i], 1000
                )
                self.bytes_sent += sd.data_size[i]
                self.packs += 1
            self.frames += 1
        finally:
            self.enc.ReleaseStream(self.chn, sd)
        return 0

    def stop(self):
        if not self.running:
            return
        self.running = False
        try:
            self.srv.rtspserver_stop()
            self.srv.rtspserver_deinit()
        except Exception as e:
            print("rtsp.stop EXC", e)
        self.srv = None
