# /sdcard/k230vision/t_fov3.py
"""Task 6 附加验证：**chn0（720p 推流帧）与 chn2（AI 帧）是否同一视场**。

## 为什么不能用 to_rgb888 / to_grayscale / get_pixel

本 build 实测这三种写法都把板子/串口搞死过（NOTES.md，共 3 次）。
本脚本走**第四条路**：`image.Image.bytearray()` ——
直接拿到完整图像缓冲区（实测 8x8 RGB888 → 192 B，16x16 YUV420 → 384 B），
**不做任何格式转换、不逐像素读**，取一帧**原样**写文件。
解码与分析全部搬到 Pi 上做（Pi 没有内存压力）。本路径实测**没有把板子搞死**。

## v2 修正：**必须先预热**

v1 在 `sensor.run()` 之后**立刻**dump chn0，拿到的是**曝光还没收敛**的第一帧：
Pi 侧解出来 Y 通道 `mean=22.7 / max=46`（整帧近乎全黑），
而 chn2 那帧 luma `mean=104`。两帧亮度差一个数量级，
"偏红区域"的 bbox 因此不可比（见 NOTES.md 里那条"必须等链路/自动曝光收敛"的教训）。
v2 在 dump 之前对**两条通道各自**预热 `WARM0 / WARM2` 帧，并打印 Y 稀疏均值自检。

## 只做「取一帧 + 写文件」

- chn0 = YUV420SP 1280x720（≈1.38 MB）→ `snapshot(chn=0).bytearray()`
- chn2 = RGBP888 320x320（≈0.3 MB）→ `snapshot(chn=2).to_numpy_ref()`（Task 5 已验安全）

**本脚本不建 VENC**：通道的取景几何由 `sensor.set_framesize` 决定，
与有没有挂编码器无关（chn0+chn2 挂 VENC 共存已由 t_vision.py 验过）。

一个 MicroPython 会话只能建一条管线 ⇒ 跑前必须 `machine.reset()`。本脚本带 `finally` 清理。
"""
import sys
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
from media.sensor import *
from media.media import *

OUT = "/sdcard/k230vision/"
WARM0 = 30      # chn0 预热帧数（等自动曝光收敛）
WARM2 = 15


def fmt_of(im):
    try:
        return im.format
    except Exception as e:
        return "?%s" % e


def sparse_mean(ba, n=921600, step=4096):
    """Y 平面的稀疏均值（只用几百次索引，不申请内存）。"""
    s = 0
    c = 0
    i = 0
    while i < n:
        s += ba[i]
        c += 1
        i += step
    return s / float(c)


sensor = None

try:
    sensor = Sensor(fps=config.SENSOR_FPS)
    sensor.reset()
    sensor.set_framesize(width=config.WIDTH, height=config.HEIGHT,
                         alignment=12, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
    sensor.set_framesize(w=config.AI_WIDTH, h=config.AI_HEIGHT, chn=CAM_CHN_ID_2)
    sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
    print("CHN0 OK %dx%d YUV420SP" % (config.WIDTH, config.HEIGHT))
    print("CHN2 OK %dx%d RGBP888" % (config.AI_WIDTH, config.AI_HEIGHT))

    sensor.run()
    print("RUN OK")

    # ---------- 预热：两条通道都要（等自动曝光/白平衡收敛）----------
    for i in range(WARM0):
        im = sensor.snapshot(chn=CAM_CHN_ID_0)
        del im
        gc.collect()
    for i in range(WARM2):
        im = sensor.snapshot(chn=CAM_CHN_ID_2)
        del im
        gc.collect()
    print("WARMUP OK chn0=%d chn2=%d" % (WARM0, WARM2))

    # ---------- 1) chn0 一帧原样 dump ----------
    im0 = sensor.snapshot(chn=CAM_CHN_ID_0)
    if im0 is None:
        print("CHN0 SNAPSHOT None")
    else:
        print("CHN0 FRAME %dx%d fmt=%s" % (im0.width(), im0.height(), fmt_of(im0)))
        ba = im0.bytearray()
        print("CHN0 BYTEARRAY type=%s len=%d" % (type(ba), len(ba)))
        print("CHN0 Y_SPARSE_MEAN %.1f" % sparse_mean(ba))
        with open(OUT + "raw_chn0.bin", "wb") as f:
            f.write(ba)
        print("CHN0 DUMP_OK bytes=%d" % len(ba))
        del ba
        del im0
        gc.collect()
        print("MEM_AFTER_CHN0 %d" % gc.mem_free())

    # ---------- 2) chn2 一帧原样 dump ----------
    im2 = sensor.snapshot(chn=CAM_CHN_ID_2)
    if im2 is None:
        print("CHN2 SNAPSHOT None")
    else:
        print("CHN2 FRAME %dx%d fmt=%s" % (im2.width(), im2.height(), fmt_of(im2)))
        arr = im2.to_numpy_ref()
        with open(OUT + "raw_chn2.bin", "wb") as f:
            f.write(arr)
        print("CHN2 DUMP_OK written=%d" % (config.AI_WIDTH * config.AI_HEIGHT * 3))
        del arr
        del im2
        gc.collect()

    print("MEM_END %d" % gc.mem_free())

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
    gc.collect()
    print("CLEANUP_DONE MEM %d" % gc.mem_free())
