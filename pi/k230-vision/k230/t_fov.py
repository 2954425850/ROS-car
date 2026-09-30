# /sdcard/k230vision/t_fov.py
"""核对 **AI 帧(chn2) 与 推流帧(chn0) 的视野是否一致**。

为什么需要：云端前端是「把归一化坐标乘上自己的显示尺寸」来叠框的。
如果 chn2 是**裁剪**出来的（而不是整幅缩放），归一化坐标在视频上就会**错位**。

怎么测：用 `get_pixel` **稀疏采样** chn0 的 720p 帧（不申请大块内存 ——
之前对 720p 做 to_rgb888 / to_grayscale 都把板子/串口搞死过），
算「偏红像素」的重心，和检测框中心比。两者接近 ⇒ 两路视野一致。

只采样 32x18 = 576 个点，零大分配。
"""
import sys
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
from media.sensor import *
from media.media import *
from media.vencoder import *

WARMUP = 25
GW, GH = 32, 18

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
    if not objs:
        print("NO_DETECTION_ABORT")
    else:
        box = objs[0]["box"]
        cx = (box[0] + box[2]) / 2.0
        cy = (box[1] + box[3]) / 2.0
        print("BOX_CENTER_NORM %.4f %.4f" % (cx, cy))

        v0 = sensor.snapshot(chn=CAM_CHN_ID_0)
        W = v0.width()
        H = v0.height()
        print("V0 %dx%d" % (W, H))

        # 第一遍：先看几个像素长什么样，确定 get_pixel 的返回类型
        print("PIXSAMPLE", [v0.get_pixel(W // 2, H // 2), v0.get_pixel(10, 10)])

        sx = 0.0
        sy = 0.0
        sw = 0.0
        nred = 0
        grid = []
        for j in range(GH):
            y = int((j + 0.5) * H / GH)
            line = []
            for i in range(GW):
                x = int((i + 0.5) * W / GW)
                p = v0.get_pixel(x, y)
                if isinstance(p, tuple):
                    r, g, b = int(p[0]), int(p[1]), int(p[2])
                else:
                    v = int(p)
                    r = (v >> 16) & 0xFF
                    g = (v >> 8) & 0xFF
                    b = v & 0xFF
                red = max(0, r - max(g, b))
                line.append(red)
                if red > 25:
                    nx = (i + 0.5) / GW
                    ny = (j + 0.5) / GH
                    sx += nx * red
                    sy += ny * red
                    sw += red
                    nred += 1
            grid.append(line)
        print("RED_PIXELS %d / %d" % (nred, GW * GH))
        for line in grid:
            print("MAP " + "".join("#" if v > 60 else ("+" if v > 25 else ".") for v in line))
        if sw > 0:
            print("RED_CENTROID_NORM %.4f %.4f" % (sx / sw, sy / sw))
            print("DELTA_FROM_BOX_CENTER %.4f %.4f" % (sx / sw - cx, sy / sw - cy))
        del v0
        gc.collect()

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
