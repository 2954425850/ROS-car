# /sdcard/k230vision/cam.py
"""sensor + 多个硬件编码通道（Task 4 多通道版）。

一路 sensor 可以同时绑到多个 VENC 通道（VENC 共 4 个，`VENC_MAX_CHN_NUMS=4`）。
chn0 给云端推流，chn1 给局域网 RTSP。

## 接口

    cam = Camera(width, height)
    cam.add_channel(chn, bitrate_kbps=2048)   # 必须在 start() 之前
    cam.start()
    cam.encoder(chn)                          # 交给 Pusher / RtspOut
    cam.stop()

## 为什么用 MediaManager.link 而不是 SendFrame

MediaManager.link 把 sensor 直接绑到编码器（零拷贝），是 rtsp_server.py 例程的做法，
CPU 开销最低。SendFrame 模式需要先把帧搬到 Python 侧，多一次拷贝。

## 多通道：**每个通道各 new 一个 Encoder()**（Task 4 实测结论）

2026-09-17 实测（每次前 machine.reset()，探针 t_venc_probe2.py）：

    MODE=A 两个 Encoder() 实例，各挂一个通道
           -> chn0 102 帧/892658 B、chn1 102 帧/908977 B，双双出流，cleanup 干净
    MODE=B 一个 Encoder() 实例挂两个通道
           -> 两个通道也能出流（各 93 帧），但 **cleanup 卡死**（sensor.stop()/Stop
              之后主线程不再响应，REPL 也不回，最后只能断电）

所以本文件用 MODE=A 的写法。**不要图省事改成单个 Encoder() 带多个通道。**

（另：`/sdcard/examples` 全部 344 个 .py 里**没有任何一个**同时用两个 VENC 通道；
`video_encoder.py` 里出现两次 `Encoder()` 只是两个独立函数，都用 VENC_CHN_ID_0。）

## 帧率：靠 Sensor(fps=) + dst_frame_rate 两处配合（2026-09-17 干净板实测）

gc2093 硬件档位只有 30/60/90（`list_mode()`），所以 25fps 要「先选 30 档，再让 VENC 丢到 25」：

1. `Sensor(fps=SENSOR_FPS)` —— **唯一有效的帧率开关**，走**构造函数**。
   ⚠️ `Sensor(fps=25)` 不被支持但**会被静默忽略**，回落到 60fps 档（实测 53.6fps），
   而 `src_frame_rate=25` 当除数会把码率放大到 4.2 Mbps —— 静默陷阱。
2. `ChnAttrStr` 的 `src_frame_rate` / `dst_frame_rate` / `gop_len` —— 默认 **30/30/30**，
   不显式赋值就形同虚设。`src_frame_rate` 是码率预算的**除数**。

**⚠️ `dst_frame_rate` 在本 build 上不丢帧**，输出帧率就等于 sensor 档位，拿不到 25fps。

## 码率：`bit_rate` 是每帧预算，不是硬上限

    总码率 ≈ bit_rate × 实际输出帧率 / src_frame_rate

## request_idr 是显式 no-op（Task 4 实测）

`dir(Encoder)` 只有 SetOutBufs / Create / Start / Stop / Destroy / GetStream /
ReleaseStream / SendFrame —— **没有请求 IDR 的接口**。所以丢帧/重连后只能等
`gop_len=25` 的下一个 IDR 自愈（30fps 下 < 1 s）。

**这里保留 `request_idr(chn)` 只是为了让调用方（app.py）有个统一的名字，
它什么都不做。** 不要以为调了它就会立刻出关键帧。

## 第三个通道 chn2：给 KPU 的 AI 帧（Task 7 加）

`add_ai_channel(w, h)` 登记一路 **CAM_CHN_ID_2 / RGBP888** 的 sensor 输出，
供 `Detector` 取帧（`sensor.snapshot(chn=CAM_CHN_ID_2)`）。

- 这正是官方 `libs/PipeLine.py` 的 `get_frame()` 内部唯一做的事；我们不用 `PipeLine`
  是因为它 `create()` 时会 `sensor.reset()` 并把 **chn0 独占给 Display**，
  而 chn0 我们要留给推流。`CAM_CHN_ID_MAX = 3`，chn0/1/2 正好用满。
- **`w/h` 必须等于模型输入分辨率**（yolov8n_320 → 320x320）：官方
  `ObjectDetectionApp` 重写了 `preprocess()`（`return [nn.from_numpy(input_np)]`），
  ai2d 那套预处理根本没被调用，改尺寸会让推理结果错乱。
- 三通道共存的顺序照抄已验证的 `t_vision.py`：**所有 `set_framesize`/`set_pixformat`
  都在 VENC `Create`/`link`/`Start` 之前**。见 `start()` 里的注释。
"""
import config
from media.sensor import *
from media.media import *
from media.vencoder import *


class Camera:
    def __init__(self, width=1280, height=720):
        self.width = ALIGN_UP(width, 16)
        self.height = height
        self.sensor = None
        self._chans = {}          # chn -> {"enc": Encoder, "link": ..., "bitrate": int}
        self._started = False
        self._ai = None           # (w, h) 或 None：chn2 给 KPU 的 AI 帧

    # ---- 配置 ----
    def add_channel(self, chn, bitrate_kbps=2048):
        """登记一个编码通道。必须在 start() 之前调用。"""
        if self._started:
            raise RuntimeError("add_channel 必须在 start() 之前调用")
        self._chans[chn] = {"enc": None, "link": None, "bitrate": bitrate_kbps}

    def add_ai_channel(self, width, height):
        """登记 chn2（RGBP888）给 KPU 做 AI 帧。必须在 start() 之前调用。

        width/height **必须等于模型输入分辨率**（见文件头）。
        """
        if self._started:
            raise RuntimeError("add_ai_channel 必须在 start() 之前调用")
        self._ai = (ALIGN_UP(width, 16), height)

    @property
    def chans(self):
        return sorted(self._chans.keys())

    # ---- 生命周期 ----
    def start(self):
        if self._started:
            raise RuntimeError("Camera 已经 start 过了")
        if not self._chans:
            raise RuntimeError("没有通道，先 add_channel(chn)")

        # 帧率只能走构造函数；SENSOR_FPS 必须是 sensor 真有的档位（30/60/90）
        self.sensor = Sensor(fps=config.SENSOR_FPS)
        self.sensor.reset()
        self.sensor.set_framesize(width=self.width, height=self.height, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)

        # chn2 -> KPU 的 AI 帧。**位置有意放在这里**：与已验证的 t_vision.py 一致，
        # 所有 set_framesize/set_pixformat 都在 VENC Create/link/Start 之前完成。
        if self._ai is not None:
            self.sensor.set_framesize(w=self._ai[0], h=self._ai[1], chn=CAM_CHN_ID_2)
            self.sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
            print("CHN2 OK %dx%d RGBP888 (AI)" % self._ai)

        # 每个通道各一个 Encoder() 实例（见文件头 MODE=A 结论）
        for chn, c in self._chans.items():
            enc = Encoder()
            enc.SetOutBufs(chn, 8, self.width, self.height)
            attr = ChnAttrStr(
                enc.PAYLOAD_TYPE_H264,
                enc.H264_PROFILE_MAIN,
                self.width,
                self.height,
                bit_rate=c["bitrate"],
            )
            # 默认 30/30/30，必须显式赋值，否则 config.FPS/GOP 形同虚设。
            # src 必须等于 sensor 真实档位：它既是丢帧判据，也是码率预算的除数。
            attr.src_frame_rate = config.SENSOR_FPS
            attr.dst_frame_rate = config.FPS
            attr.gop_len = config.GOP
            print("VENC chn=%s src=%s dst=%s gop=%s bitrate=%s kbps"
                  % (chn, config.SENSOR_FPS, config.FPS, config.GOP, c["bitrate"]))
            enc.Create(chn, attr)
            c["enc"] = enc

        src = self.sensor.bind_info()["src"]
        for chn, c in self._chans.items():
            c["link"] = MediaManager.link(src, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, chn))
            print("LINK chn=%s OK" % chn)

        for chn, c in self._chans.items():
            c["enc"].Start(chn)
            print("START chn=%s OK" % chn)

        self.sensor.run()
        self._started = True

    def encoder(self, chn):
        return self._chans[chn]["enc"]

    def request_idr(self, chn):
        """**显式 no-op**：本平台 Encoder 没有请求 IDR 的接口（见文件头）。

        保留这个函数名是为了让调用方有统一的写法；调用它不会产生任何效果，
        真正的自愈手段是等 gop_len=25 的下一个 IDR。
        """
        return None

    def stop(self):
        """完整清理。顺序与 Task 2 验证过的干净路径一致：
        sensor.stop() -> del link -> encoder.Stop/Destroy。
        """
        if self.sensor:
            try:
                self.sensor.stop()
            except Exception as e:
                print("cam.stop sensor EXC", e)
        for chn, c in self._chans.items():
            if c["link"] is not None:
                try:
                    del c["link"]
                except Exception as e:
                    print("cam.stop link(%s) EXC" % chn, e)
                c["link"] = None
            if c["enc"] is not None:
                try:
                    c["enc"].Stop(chn)
                except Exception as e:
                    print("cam.stop Stop(%s) EXC" % chn, e)
                try:
                    c["enc"].Destroy(chn)
                except Exception as e:
                    print("cam.stop Destroy(%s) EXC" % chn, e)
                c["enc"] = None
        self._started = False
