# /sdcard/k230vision/t_fov2.py
"""核对 AI 通道(chn2) 的取景方式：**整幅缩放** 还是 **居中裁剪**？

## 为什么这个测试能回答问题

- 若 chn2 是「把整幅 sensor 画面缩放到目标尺寸」（各向异性）：把目标从 320x320 改成
  320x180（16:9，与推流同比例）后，**同一个物体在归一化坐标里的位置和范围不变**
  （只是不再被纵向拉伸）。
- 若 chn2 是「按目标宽高比居中裁剪」：320x320 时只用到画面中间 56% 宽，
  改成 320x180 会用满全宽 —— 物体的归一化位置/范围**会明显改变**。

物体位置不变 ⇒ AI 帧与推流帧**同视场** ⇒ 归一化坐标乘前端显示尺寸可以直接叠框。
"""

import sys
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
from media.sensor import *
from media.media import *
from media.vencoder import *

OUT = "/sdcard/k230vision/"
AIW, AIH = 320, 180   # 16:9，与推流同比例

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
    # 关键：chn2 换成 16:9
    sensor.set_framesize(w=AIW, h=AIH, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    print("CHN2 OK %dx%d RGBP888" % (AIW, AIH))

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

    sensor.run()
    # 丢弃前几帧
    for i in range(20):
        im = sensor.snapshot(chn=CAM_CHN_ID_2)
        del im
        gc.collect()

    im = sensor.snapshot(chn=CAM_CHN_ID_2)
    print("FRAME %dx%d" % (im.width(), im.height()))
    # 这一帧的偏红区域重心（RGBP888 planar，直接自己算，不用 KPU）
    arr = im.to_numpy_ref()
    n = AIW * AIH
    with open(OUT + "cap_ai_169.raw", "wb") as f:
        f.write(arr)
    print("DUMP_OK cap_ai_169.raw bytes=%d" % (n * 3))
    del arr
    del im
    gc.collect()
    print("MEM %d" % gc.mem_free())

except BaseException as e:
    import sys as _s
    _s.print_exception(e)
finally:
    print("CLEANUP_START")
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
