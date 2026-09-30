# /sdcard/k230vision/t_capture.py
"""抓帧落盘做目视核对（v3）。

两件事：
  1. AI 帧（chn2, 320x320 RGBP888）—— 证明框套在物体上
  2. 推流帧（chn0, 1280x720 YUV420SP）—— 证明**两路视野一致**，
     即「归一化坐标 × 前端显示尺寸」能正确叠加

⚠️ **绝对不要对 720p 帧调 `to_rgb888()`**：RGB888 需要 1280*720*3 = 2.7 MB 连续内存，
在 3.9 MB 的 MicroPython 堆上会直接把板子/串口搞死（实测过一次）。
只 dump 裸字节，转换放到 Pi 侧做。
"""
import sys
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
from media.sensor import *
from media.media import *
from media.vencoder import *
import image

OUT = "/sdcard/k230vision/"
WARMUP = 25

det = None
sensor = None
encs = {}
links = {}

try:
    sensor = Sensor(fps=config.SENSOR_FPS)
    sensor.reset()
    sensor.set_framesize(width=config.WIDTH, height=config.HEIGHT,
                         alignment=12, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    sensor.set_framesize(width=config.WIDTH, height=config.HEIGHT,
                         alignment=12, chn=CAM_CHN_ID_1)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)
    sensor.set_framesize(w=config.AI_WIDTH, h=config.AI_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)

    src = sensor.bind_info()["src"]
    for chn in (config.PUSH_CHN, config.RTSP_CHN):
        enc = Encoder()
        enc.SetOutBufs(chn, 8, config.WIDTH, config.HEIGHT)
        attr = ChnAttrStr(enc.PAYLOAD_TYPE_H264, enc.H264_PROFILE_MAIN,
                          config.WIDTH, config.HEIGHT,
                          bit_rate=config.BITRATE_KBPS)
        attr.src_frame_rate = config.SENSOR_FPS
        attr.dst_frame_rate = config.FPS
        attr.gop_len = config.GOP
        enc.Create(chn, attr)
        encs[chn] = enc
        links[chn] = MediaManager.link(src, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, chn))
        enc.Start(chn)
    print("PIPELINE OK")

    from vision import Detector
    det = Detector(kmodel_path=config.KMODEL_PATH, labels=config.LABELS,
                   model_input_size=config.MODEL_INPUT_SIZE,
                   max_boxes_num=config.MAX_BOXES_NUM,
                   confidence_threshold=config.CONF_THRESHOLD,
                   nms_threshold=config.NMS_THRESHOLD,
                   rgb888p_size=[config.AI_WIDTH, config.AI_HEIGHT])
    det.config_preprocess()
    sensor.run()

    objs = []
    for i in range(WARMUP):
        im = sensor.snapshot(chn=CAM_CHN_ID_2)
        objs = det.detect(im.to_numpy_ref())
        del im
        gc.collect()
    print("OBJS", objs)
    print("MEM_BEFORE_DUMP %d" % gc.mem_free())

    # ---- 1. AI 帧裸像素 ----
    ai = sensor.snapshot(chn=CAM_CHN_ID_2)
    with open(OUT + "cap_ai.raw", "wb") as f:
        f.write(ai.to_numpy_ref())
    print("DUMP_OK cap_ai.raw")
    del ai
    gc.collect()

    # ---- 2. 推流帧（chn0, YUV420SP）----
    # YUV420SP 不支持 to_numpy_ref（ValueError: image format not support），
    # 且 720p 的 RGB888 转换需要 2.7 MB 连续内存会把板子搞死，所以只试这两种小格式。
    v0 = sensor.snapshot(chn=CAM_CHN_ID_0)
    print("V0 size=%s fmt=%s" % (v0.width(), v0.height()))
    for meth, path in (("to_grayscale", "cap_v0_gray.raw"),
                       ("to_rgb565", "cap_v0_565.raw")):
        try:
            c = getattr(v0, meth)()
            print("CONV_OK %s mem=%d" % (meth, gc.mem_free()))
            with open(OUT + path, "wb") as f:
                f.write(c.to_numpy_ref())
            print("DUMP_OK %s" % path)
            del c
            gc.collect()
        except Exception as e:
            print("CONV_EXC %s: %s" % (meth, e))
        gc.collect()
    del v0
    gc.collect()

    import os
    print("FILES", sorted(f for f in os.listdir(OUT) if f.startswith("cap_")))

except BaseException as e:
    import sys as _s
    _s.print_exception(e)
finally:
    print("CLEANUP_START")
    if det is not None:
        try:
            det.deinit()
        except Exception as e:
            print("cleanup det EXC", e)
    if sensor is not None:
        try:
            sensor.stop()
        except Exception as e:
            print("cleanup sensor EXC", e)
    for chn, lk in links.items():
        try:
            del lk
        except Exception as e:
            print("cleanup link EXC", e)
    for chn, enc in encs.items():
        try:
            enc.Stop(chn)
        except Exception as e:
            print("cleanup Stop(%s) EXC" % chn, e)
        try:
            enc.Destroy(chn)
        except Exception as e:
            print("cleanup Destroy(%s) EXC" % chn, e)
    gc.collect()
    print("CLEANUP_DONE MEM %d" % gc.mem_free())
