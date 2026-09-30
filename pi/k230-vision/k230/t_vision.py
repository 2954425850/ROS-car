# /sdcard/k230vision/t_vision.py
"""Task 5 验证：KPU 推理，并**同时**证明两路 VENC 还在出流。

## 推理帧来源（本任务唯一的研究问题）

**结论写在文件头，完整证据见 NOTES.md**：
AI 的帧不来自 PipeLine 的 `get_frame()`，而是**直接来自 sensor 的第三个输出通道
`CAM_CHN_ID_2`（RGBP888）**，用 `sensor.snapshot(chn=CAM_CHN_ID_2)` 取。
官方 `PipeLine.get_frame()` 内部做的**就是这一件事**：

    self.cur_frame = self.sensor.snapshot(chn=CAM_CHN_ID_2)
    return self.cur_frame.to_numpy_ref()

所以**不需要 PipeLine**：它只是个把 sensor + Display + OSD 打包起来的壳，
而 Display 在我们的方案里是多余的（headless，框交给云端前端）。
我们已经有自己验通的多通道 `cam.py`（chn0 + chn1 两路 VENC），
只要**再配一个 chn2 给 AI** 即可 —— 三者同源同 sensor、分辨率各异。

## 测试协议

一个 MicroPython 会话只能建一条管线，所以跑之前必须 `machine.reset()`。
本脚本带 `finally` 完整清理。

## 用法

    timeout 150 ~/.local/bin/mpremote connect /dev/ttyACM0 exec \
        "exec(open('/sdcard/k230vision/t_vision.py').read())"
"""
import sys
import time
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
from media.sensor import *
from media.media import *
from media.vencoder import *

DUR = 12.0          # 采样秒数


def main():
    det = None
    sensor = None
    encs = {}
    links = {}
    try:
        # ---------- 1. sensor + 三路通道 ----------
        sensor = Sensor(fps=config.SENSOR_FPS)
        sensor.reset()
        print("SENSOR OK fps=%s" % config.SENSOR_FPS)

        # chn0 -> VENC0（推流），与 cam.py 一致
        sensor.set_framesize(width=config.WIDTH, height=config.HEIGHT,
                             alignment=12, chn=CAM_CHN_ID_0)
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_0)
        print("CHN0 OK %dx%d YUV420SP" % (config.WIDTH, config.HEIGHT))

        # chn1 -> VENC1（RTSP）
        sensor.set_framesize(width=config.WIDTH, height=config.HEIGHT,
                             alignment=12, chn=CAM_CHN_ID_1)
        sensor.set_pixformat(Sensor.YUV420SP, chn=CAM_CHN_ID_1)
        print("CHN1 OK %dx%d YUV420SP" % (config.WIDTH, config.HEIGHT))

        # chn2 -> AI（RGBP888），写法与官方 PipeLine.create() 里那两行一致
        sensor.set_framesize(w=config.AI_WIDTH, h=config.AI_HEIGHT, chn=CAM_CHN_ID_2)
        sensor.set_pixformat(Sensor.RGBP888, chn=CAM_CHN_ID_2)
        print("CHN2 OK %dx%d RGBP888" % (config.AI_WIDTH, config.AI_HEIGHT))

        # ---------- 2. 两路 VENC（照抄 cam.py 的 MODE=A 写法）----------
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
            links[chn] = MediaManager.link(
                src, (VIDEO_ENCODE_MOD_ID, VENC_DEV_ID, chn))
            print("LINK chn=%s OK" % chn)

        for chn, enc in encs.items():
            enc.Start(chn)
            print("START chn=%s OK" % chn)

        # ---------- 3. 推理器 ----------
        from vision import Detector
        det = Detector(
            kmodel_path=config.KMODEL_PATH,
            labels=config.LABELS,
            model_input_size=config.MODEL_INPUT_SIZE,
            max_boxes_num=config.MAX_BOXES_NUM,
            confidence_threshold=config.CONF_THRESHOLD,
            nms_threshold=config.NMS_THRESHOLD,
            rgb888p_size=[config.AI_WIDTH, config.AI_HEIGHT],
        )
        det.config_preprocess()
        print("DETECTOR OK kmodel=%s" % config.KMODEL_PATH)
        print("KPU inputs=%d outputs=%d"
              % (det.get_kmodel_inputs_num(), det.get_kmodel_outputs_num()))

        # ---------- 4. 启流 + 采样 ----------
        sensor.run()
        print("RUN OK")

        sd = StreamData()
        t0 = time.time()
        n = 0
        venc_frames = 0
        hits = 0
        last_objs = None
        while time.time() - t0 < DUR:
            img = sensor.snapshot(chn=CAM_CHN_ID_2)
            if img is None:
                print("SNAPSHOT None")
                continue
            objs = det.detect(img.to_numpy_ref())
            n += 1
            # 头两帧把 postprocess 的**原始**返回打出来，核对字段顺序
            if n <= 2:
                raw = det.last_dets
                print("RAW n=%d type=%s len=%s" % (n, type(raw), len(raw) if raw else 0))
                for k in range(len(raw)):
                    rk = raw[k]
                    print("  RAW[%d] len=%d first3=%s" % (k, len(rk), rk[:3]))
            if objs:
                hits += 1
                last_objs = objs
            if n <= 3 or (objs and hits <= 8):
                print("FRAME %d objs: %s" % (n, objs))
            # 同一时刻把 chn0 的码流取出来 —— 证明 VENC 没有被 AI 饿死
            if encs[config.PUSH_CHN].GetStream(config.PUSH_CHN, sd, timeout=0) == 0:
                venc_frames += 1
                encs[config.PUSH_CHN].ReleaseStream(config.PUSH_CHN, sd)
            del img
            gc.collect()
        del sd

        el = time.time() - t0
        print("STATS infer_frames=%d venc_chn0_frames=%d obj_frames=%d el=%.2f"
              % (n, venc_frames, hits, el))
        print("INFER_FPS %.2f" % (n / el))
        print("VENC_FPS %.2f" % (venc_frames / el))
        print("LAST_OBJS %s" % (last_objs,))
        print("MEM %d" % gc.mem_free())

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
                print("cleanup link(%s) EXC" % chn, e)
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


main()
