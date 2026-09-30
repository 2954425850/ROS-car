# /sdcard/k230vision/app.py
"""主循环：WiFi -> 摄像头(3 通道) -> 推流 + RTSP + 推理 + 结果上报。

## 单线程编排

MicroPython 的 _thread 在这块板子上与 MPP 的配合没验证过，**不要用多线程**。
所有"取一帧"的动作都是非阻塞或短超时的（pusher 100 ms / rtsp 0），
在一个循环里轮流推进 —— Task 4 已证单线程 `pump()` 与官方 _thread 版等效。

## 四处按实测修正的地方（照抄计划原版会踩）

1. **不用 `from libs.PipeLine import PipeLine`**，计划里那句是多余的。
   AI 帧来自 sensor 的**第三个通道**：`sensor.snapshot(chn=CAM_CHN_ID_2)`
   （RGBP888）。`PipeLine` 只是把 sensor+Display+OSD 打包的壳，它 `reset()` 后
   **独占 chn0 给 Display**，而 chn0 我们要给推流。`CAM_CHN_ID_MAX = 3`，正好用满。
2. **AI 帧分辨率必须等于模型输入分辨率（320x320）**：官方 `ObjectDetectionApp`
   重写了 `preprocess()`（`return [nn.from_numpy(input_np)]`），ai2d 预处理
   根本没被调用，改尺寸会让推理结果错乱。
3. **`time.time()` 在本平台只有整数秒**，计划里 `now = time.time();
   if now - last >= 0.1` 这种节流**是坏的**（实测一个目标 10 Hz 的循环跑成了
   24.55 Hz，差值恒为 0，节流退化成空操作）。本文件所有定速/计时一律用
   `ticks_ms` / `ticks_diff`（写法照已验证的 t_report.py）。
   注意 `ts` 字段 `int(time.time()*1000)` 因此**只有秒级精度**，这是平台限制。
4. **`cam.request_idr()` 是显式 no-op**（本平台 Encoder 没有请求 IDR 的接口），
   保留调用但不产生效果；重连后的恢复靠 `gop_len=25`（0.83 s）等下一个 IDR。
   （原计划里的 `_last_objs` 占位符早已由真检测结果取代；
   **2026-09-19 起连"沿用上一帧"这个做法本身也去掉了** ——
   连续性改由跟踪器负责，陈旧坐标不再冒充新数据。见下"跟踪器"。）

## 与计划的两处有意偏离（都写明理由）

- **推理与上报合并到同一个 10 Hz 节拍。** 计划把它们写成两个各自 `>= 0.1` 的块，
  周期相同却独立推进，会互相漂移（某条 report 可能带上一轮或下一轮的 frame 号）。
  合并后每条消息恰好对应一次推理，`frame` 连续无跳号（与已验证的 t_report.py 同形）。
- **`wd.tick()` 每轮都调。** 计划只在 `rc == 0` 时调；那样在完全断流
  （`rc` 一直 -1/-2）时看门狗**永远不会触发**，恰好丢掉了它唯一该管的场景。
  为了让停滞告警不刷屏，用 `on_stall` 回调（每个停滞期只打一行）。

## Detector 为什么在 Camera 之前建

计划把 `Detector` 建在 sensor 之后。已验证的 `t_vision.py`/`t_report.py` 里
`sensor.run()` 是**最后**一步，而 `cam.start()` 内部就会 `sensor.run()`；
`sensor.run()` 之后再建 Detector 属于未验证路径。`AIBase.__init__` 只做
`kpu.load_kmodel()`，不碰 sensor，所以把 Detector 提到 `Camera` 之前建是安全的。
（本文件已实测：见 NOTES.md「Task 7」。）
**`Tracker` 同理，也建在 `Camera` 之前。**

## 跟踪器：补上检测器缺的那一半

检测器每帧独立判断、**没有记忆**，所以**对运动中的目标掉帧严重**：
实测目标静止时命中 **97.8%**，目标/场景在动时只有 **36.7%**
（分数全都贴在 `conf=0.30` 门限上，余量极薄）。

`Tracker`（NanoTrack，**类别无关**）接住那 63%：给一个框就把它连续跟下去，
不关心框里是什么。两者分工：

    Detector  认出"是什么"、给出一个框   —— 准，但会跳、会丢
    Tracker   跟住"就是它"             —— 连续，但不知道那是什么

**框的来源目前是检测器**，所以只覆盖检测器认识的类（`config.TRACK_CLASS`）。
检测器**不认识**的物体（桃、辣椒）需要人给一次框，那条路还没做。

**判漂移用 `ar_dev`（长宽比偏离）+ `iou`（与检测框的交并比），不用 score** ——
实测：跟到背景上时 score 一样是 0.999。详见
`docs/plans/2026-09-19-tracker-integration.md`。

## 外部目标框入口：检测器不认识的物体唯一的入口

检测器只知道那 80 类。**桃、辣椒这类东西它永远给不出框**，所以跟踪器再好也没用 ——
没有框可跟。`targets.py` 提供那个入口：外面发一行 JSON 进来指定"跟这个"。

    监听 8557（config.TARGET_PORT），**一次命令一个连接**，收一行 JSON 回一行 JSON：

    {"pt": [u, v]}                  # 点 —— 归一化中心点，自动开一个方框
    {"pt": [u, v], "size": 0.12}    # size = 方框边长占**画面宽度**的比例
    {"box": [l, t, r, b]}           # 归一化框，与 results.normalize 同约定
    {"cmd": "stop"}                 # 停止跟踪

坐标一律**归一化到推流画面** —— 发命令的一方**不需要知道 AI 帧多大、有没有 letterbox**，
那是 K230 自己的事。

**任一条外部命令都会关掉"检测器自动锁定"**（`det_auto = False`）：人来指定就是命令，
不能被下一帧的检测器覆盖掉。目前**没有"切回检测器驱动"的命令**，要回去只能重启。

## 人脸：谁在场

和检测器同源同一帧跑（`FaceRecognizer.analyze()`，约 **17ms/张脸**），
结果走上报里**单独的 `faces` 数组**，不混进 `objs`。

**每条人脸的关键字段是 `who` 和 `ok`，它们的组合有不同含义，下游要区别对待：**

    ok=True  且 who="id1"   -> 认出来了
    ok=True  且 who=None    -> 脸是好的，但库里没有这个人
    ok=False 且 who=None    -> **这张脸不完整/太靠边，身份不可信**（不是"不认识"）

**`ok=False` 时不查库** —— 拿一张被切掉半边的脸去比对，只会得到一个低分，
而那个低分**分不清是"不是这个人"还是"这张脸没拍好"**。本项目原则是
**认不出好过认错人**，所以宁可报"不可用"。

特征库在 `/sdcard/k230vision/facedb/<名字>.bin`（每人多帧，比对取最大）——
**多帧是必需的**：实测单参考会让 63.3% 的帧认不出自己，多帧降到 0%。

## 注册不是一次仪式：**边用边长**

种子注册只要**一个很短的时间窗**（`config.FACE_SEED_SECONDS`，之后靠增量长）。
运行中只要满足条件就往库里补一帧，所以"注册要多久"这件事不再重要。

**入库的门限（`FACE_ADMIT = 0.90`）比识别门限（0.78）高得多**，因为
**增量入库会自我强化错误**：把一个误认的陌生人写进谁的库，以后就更像他了。
四道限制缺一不可：

    1. 分数 >= FACE_ADMIT（远高于识别门限）
    2. 脸过了质量门（`ok`）
    3. 画面里**只有一张脸**
    4. 与库里已有的太像就不加（在 `FaceDB.append()` 里）

改动**每隔 `FACE_SAVE_EVERY_MS` 落盘一次**，退出前再刷一次 —— 别每帧写 SD。

⚠️ 门限 0.78 是**暂定**的：实测"同一人/不同人"的分离窗口只有 0.047 宽，
且"不同人"那组是在相机取景修好之前测的。细节见
`docs/plans/2026-09-19-face-pipeline.md`。

## 恢复边界（不要越界）

`pusher.py` **只做 TCP 层重连，不重建 MPP 管线**：同一 MicroPython 会话内拆掉管线
再建第二条**必报** `MediaManager link failed(9)`，重建只有 `machine.reset()` 一条路。
所以这里重连失败就退避重试，**绝不写"推流失败就重建管线"**。

## 用法

    timeout 1900 ~/.local/bin/mpremote connect /dev/ttyACM0 exec \
        "exec(open('/sdcard/k230vision/app.py').read())"
"""
import os
import sys
import time
import gc

sys.path.insert(0, "/sdcard/k230vision")

import config
import netup
from media.sensor import CAM_CHN_ID_2
from cam import Camera
from pusher import Pusher
from rtsp_srv import RtspOut
from results import encode, normalize
from reporter import Reporter
from vision import Detector
from tracker import Tracker
from targets import (TargetRx, template_crop_side as tside,
                     pick_body_for_face)
from faces import FaceRecognizer, FaceDB
from watchdog import Watchdog

INFER_HZ = 10.0                 # 推理 + 上报节拍（计划两块都是 0.1 s）
STATS_MS = 30000                # 30 s 一行 STATS，给长稳测试留证据


def main():
    netup.init(config.WIFI_SSID, config.WIFI_PASS)
    while not netup.connect():
        print("WIFI FAILED, retry")
        time.sleep(2)
    print("WIFI OK %s" % netup.ip())

    # ---- KPU：在 Camera 之前建（只 load kmodel，不碰 sensor，见文件头）----
    det = Detector(kmodel_path=config.KMODEL_PATH,
                   labels=config.LABELS,
                   model_input_size=config.MODEL_INPUT_SIZE,
                   max_boxes_num=config.MAX_BOXES_NUM,
                   confidence_threshold=config.CONF_THRESHOLD,
                   nms_threshold=config.NMS_THRESHOLD,
                   rgb888p_size=[config.AI_WIDTH, config.AI_HEIGHT])
    det.config_preprocess()
    print("DETECTOR OK kmodel=%s" % config.KMODEL_PATH)

    trk = Tracker(rgb888p_size=[config.AI_WIDTH, config.AI_HEIGHT],
                  thresh=config.TRACK_THRESH,
                  ar_tol=config.TRACK_AR_TOL,
                  iou_min=config.TRACK_IOU_MIN)
    print("TRACKER OK class=%s ar_tol=%s iou_min=%s"
          % (config.TRACK_CLASS, config.TRACK_AR_TOL, config.TRACK_IOU_MIN))

    # ---- 模板几何一致性断言（2026-09-19 加）----
    # `targets.py` 里那份 `template_crop_side()` 是**抄**厂商 TrackCropApp 的式子
    # （targets 是纯函数模块、不能 import 硬件，所以只能抄）。抄错 = 校验放行越界框 = 死机。
    # 这里拿**真实例**的 get_padding_crop_param() 对一次，把"两份式子漂移"变成启动就能看见。
    _drift = 0
    for _w, _h in ((38.4, 38.4), (16.0, 9.0), (64.0, 54.0), (90.0, 90.0), (6.4, 6.4)):
        trk._crop.center_xy_wh = [1.0, 1.0, _w, _h]   # 只借用公式；此时尚未 init，不参与裁剪
        _real = trk._crop.get_padding_crop_param()[8]
        _mine = tside(_w, _h)
        if _real != _mine:
            _drift += 1
            print("WARN template-geometry DRIFT %.1fx%.1f: targets=%s TrackCropApp=%s"
                  % (_w, _h, _mine, _real))
    if _drift:
        print("GEOM MISMATCH %d/5 —— targets.py 的模板几何已与厂商代码脱节，越界校验不可信"
              % _drift)
    else:
        print("GEOM OK 5/5 (targets.template_crop_side == TrackCropApp)")
    trk._crop.center_xy_wh = [1.0, 1.0, 1.0, 1.0]     # 复原成构造时的初值

    # 外部目标框入口（必须在 netup 之后 —— 它要 bind 端口）
    rx = TargetRx(config.TARGET_PORT, config.AI_WIDTH, config.AI_HEIGHT,
                  default_size=config.TARGET_SIZE)
    print("TARGETRX OK port=%d  ({\"pt\":[u,v]} / {\"box\":[l,t,r,b]} / "
          "{\"cmd\":\"stop\"|\"auto\"|\"follow\",\"who\":\"id1\"})"
          % config.TARGET_PORT)

    # ---- 人脸：检测/对齐/特征/活体 + 特征库。**建在 Camera 之前**（同 Detector/Tracker）----
    # ⚠️ 加载失败**不能拖垮整个 app** —— 禁掉人脸、其余照常。
    # （见相机端设计文档 §7「模型加载失败不能拖垮整个 app」）
    frec = None
    fdb = None
    if config.FACE_ENABLE:
        try:
            frec = FaceRecognizer(config.AI_WIDTH, config.AI_HEIGHT, need_liveness=True)
            fdb = FaceDB(config.FACE_DB_DIR, threshold=config.FACE_THRESHOLD,
                         db_max=config.FACE_DB_MAX)
            print("FACE OK 库=%d 人 / %d 条  识别门限=%.2f 入库门限=%.2f 每人上限=%d"
                  % (fdb.count(), fdb.total_vecs(), fdb.threshold,
                     config.FACE_ADMIT, fdb.db_max))
        except BaseException as e:
            print("FACE DISABLED (%s: %s)" % (type(e).__name__, e))
            frec = None
            fdb = None
    else:
        print("FACE disabled (config.FACE_ENABLE=False)")

    # ---- 摄像头：chn0 推流 / chn1 RTSP / chn2 AI ----
    cam = Camera(config.WIDTH, config.HEIGHT)
    cam.add_channel(config.PUSH_CHN, config.BITRATE_KBPS)
    cam.add_channel(config.RTSP_CHN, config.BITRATE_KBPS)
    cam.add_ai_channel(config.AI_WIDTH, config.AI_HEIGHT)
    cam.start()
    print("CAM OK chans=%s ai=%sx%s" % (cam.chans, config.AI_WIDTH, config.AI_HEIGHT))

    pusher = Pusher(config.PUSH_HOST, config.PUSH_PORT,
                    cam.encoder(config.PUSH_CHN), config.PUSH_CHN)
    print("PUSH connect=%s %s:%s" % (pusher.connect(),
                                     config.PUSH_HOST, config.PUSH_PORT))

    rtsp = RtspOut(cam.encoder(config.RTSP_CHN), config.RTSP_CHN,
                   config.RTSP_PORT, config.RTSP_SESSION,
                   config.WIDTH, config.HEIGHT)
    rtsp.start()
    print("RTSP %s" % rtsp.get_url())

    r_pi = Reporter(config.RESULT_HOST, config.RESULT_PORT)
    print("REPORTER connect=%s %s:%s" % (r_pi.connect(),
                                         config.RESULT_HOST, config.RESULT_PORT))

    # 云端未就绪（CLOUD_RESULT_HOST 为空串）=> 不启用，**本地路径不因此失败**
    r_cloud = None
    if config.CLOUD_RESULT_HOST:
        r_cloud = Reporter(config.CLOUD_RESULT_HOST, config.CLOUD_RESULT_PORT)
        print("CLOUD connect=%s %s:%s" % (r_cloud.connect(),
                                          config.CLOUD_RESULT_HOST,
                                          config.CLOUD_RESULT_PORT))
    else:
        print("CLOUD disabled (CLOUD_RESULT_HOST empty) - local path unaffected")

    wd = Watchdog(config.PUSH_STALL_WARN_MS)
    # 用回调而不是每轮 print：停滞期每轮打一行会把 30 min 的日志刷爆
    wd.on_stall = lambda: print("WARN push stalled idle_ms=%d sent=%d"
                                % (wd.idle_ms, pusher.frames_sent))

    t_start = time.ticks_ms()
    nxt = t_start
    t_stats = t_start
    period = int(1000.0 / INFER_HZ)
    frame_no = 0
    obj_frames = 0
    sample = None
    # 跟踪器计数
    trk_frames = 0          # 锁定着的帧数
    trk_locks = 0
    trk_releases = 0
    trk_iou = -1.0          # 最近一次与检测框的 IoU（没有检测框时保持旧值）
    det_auto = True         # 是否允许"检测器自动锁定"；任一外部命令都会把它关掉
    follow_who = None       # 正在等"这个人的正脸"（follow 命令受理后置位）
    follow_deadline = 0     # 等它等到什么时候（超时就放弃，不动 det_auto）
    # 人脸计数
    face_seen = 0           # 检测到的脸（按张累计）
    face_ok = 0             # 过了质量门的
    face_known = 0          # 认出是谁的
    face_added = 0          # **增量入库**加进去的特征条数（"边用边长"）
    face_err = 0            # 人脸环节报错的次数（只打第一条，免得刷屏）
    t_face_save = time.ticks_ms()   # 上次把库落盘的时刻

    try:
        while True:
            # ★ os.exitpoint()：**让它能被 Ctrl-C 打断。**
            #
            # MicroPython 只在 exitpoint 处处理挂起的键盘中断。不调它，主循环就是
            # "不可打断"的 —— 而本文件会由 `/sdcard/main.py` 上电自启，那就意味着
            # **raw REPL 再也进不去，只能拔卡**（2026-09-17 踩过，不是猜的：
            # NOTES.md 里"自启 app.py 后 raw REPL 进不去"记的就是这个）。
            #
            # 调了它之后：Ctrl-C（或 CanMV IDE 的停止按钮）→ 抛 KeyboardInterrupt
            # → 下面 except 打印 + finally 清理 → 回到 REPL。
            # **开机自启和"随时能救回来"这件事就不冲突了。**
            os.exitpoint()

            # 1) 推流：每轮尽量取，不空转
            rc = pusher.pump()
            if rc == -2:
                # 断线：退避后只重连 socket。**不要在这里重建 MPP 管线**（见文件头）
                time.sleep_ms(config.RECONNECT_BACKOFF_MS)
                if pusher.connect():
                    print("PUSH RECONNECTED connects=%d" % pusher.connects)
                    cam.request_idr(config.PUSH_CHN)   # 显式 no-op，见文件头第 4 条
            # 看门狗每轮都喂（计划只在 rc==0 时喂，那样完全断流时它永不触发）
            wd.tick(pusher.frames_sent)

            # 2) RTSP 通道（非阻塞；没有客户端时也要 GetStream->Release，
            #    否则 VENC 的 8 个输出缓冲会被填满把 MPP 顶住）
            rtsp.pump()

            # 2b) 外部目标框入口（非阻塞；只有人来命令时才有活）
            rx.poll()

            # 3) 10 Hz：推理 + 上报（同一节拍，见文件头"偏离"）
            now = time.ticks_ms()
            if time.ticks_diff(now, nxt) >= 0:
                nxt = now + period
                img = cam.sensor.snapshot(chn=CAM_CHN_ID_2)
                if img is not None:
                    arr = img.to_numpy_ref()
                    objs = det.detect(arr)
                    frame_no += 1
                    if objs:
                        obj_frames += 1
                        if sample is None:
                            sample = objs

                    # ---- 外部指定的目标（点屏/语音/云端）：优先级高于检测器 ----
                    # 人来指定就是命令。**任一条外部命令都关掉"检测器自动锁定"**
                    # （det_auto=False），否则：
                    #   - 外部说"停"，下一帧检测器又把同一个类锁回来 → "停"等于没生效
                    #   - 外部指定了苹果，跟漂了却自动锁到橘子上 → 目标被偷偷换掉
                    # 想回到检测器驱动，重启即可（**还没有"切回来"的命令**，见文档 §5）。
                    kind, px = rx.take()
                    if kind == "stop":
                        if trk.locked:
                            trk.release()
                            trk_releases += 1
                        det_auto = False
                        follow_who = None
                        print("TARGET stop -> det_auto OFF")
                    elif kind == "follow":
                        # ⚠️ 这里**先不动 det_auto** —— 要等真锁上了才关。
                        # 人不在画面里时不该把板子自带的自动锁定废掉。
                        follow_who = px
                        follow_deadline = time.ticks_add(now, config.FOLLOW_TIMEOUT_MS)
                        print("TARGET follow who=%s 已受理（等人脸）" % px)
                    elif kind == "auto":
                        det_auto = True
                        follow_who = None
                        print("TARGET auto -> det_auto ON（回到检测器驱动）")
                    elif kind == "set":
                        det_auto = False
                        follow_who = None
                        if trk.init(arr, px):
                            trk_locks += 1
                            print("TARGET set px=(%d,%d,%d,%d) -> det_auto OFF"
                                  % (px[0], px[1], px[2], px[3]))
                        else:
                            print("TARGET set REJECTED px=%s  (%s)"
                                  % (px, trk.last_reject))

                    # ---- 跟踪器：给锁定目标补上帧间连续性 ----
                    ref = None
                    for o in objs:
                        if o["cls"] == config.TRACK_CLASS:
                            ref = o["box"]
                            break
                    t_objs = []
                    if not trk.locked:
                        if ref is not None and det_auto:
                            l, t, r, b = ref
                            if trk.init(arr, (l * config.AI_WIDTH, t * config.AI_HEIGHT,
                                              (r - l) * config.AI_WIDTH,
                                              (b - t) * config.AI_HEIGHT)):
                                trk_locks += 1
                    else:
                        tk = trk.update(arr, ref_box=ref)
                        if tk is not None:
                            if tk["lost"]:
                                # 漂了就松手，等检测器再给框 —— **不要带着坏框往下走**
                                trk.release()
                                trk_releases += 1
                            else:
                                t_objs.append({
                                    "cls": config.TRACK_CLASS,
                                    "score": tk["score"],
                                    "box": tk["box"],
                                    "track_id": 1,
                                    "src": "track",
                                })
                                if tk["iou"] is not None:
                                    trk_iou = tk["iou"]
                    # ---- 人脸：谁在场 ----
                    # 与检测器同源同一帧。约 17ms/张脸，10 Hz 下有富余。
                    fobjs = []
                    if frec is not None:
                        try:
                            f4 = arr.reshape((1, 3, config.AI_HEIGHT, config.AI_WIDTH))
                            got = frec.analyze(f4)
                            del f4
                            n_faces = len(got)
                            for f in got:
                                bx = f["box"]
                                who = None
                                match = None
                                # ⭐ **质量门：脸不完整就不做身份判定。**
                                # 报"这张脸不可用"好过给出一个低分匹配 ——
                                # 本项目原则是**认不出好过认错人**。
                                if f["ok"] and fdb is not None and fdb.count() > 0:
                                    nm, sc, known = fdb.match(f["feat"])
                                    match = round(sc, 4)
                                    if known:
                                        who = nm
                                    # ---- 增量入库（"边用边长"）----
                                    # 注册不是一次仪式：认得出的人每次出现都可能补一帧。
                                    # **三道限制，少一道都会把库搞坏：**
                                    #   1. 分数 >= FACE_ADMIT（**远高于**识别门限 0.78）
                                    #      —— 增量入库会自我强化错误，只有非常有把握的才准进
                                    #   2. 脸要过质量门（就是外面的 f["ok"]）
                                    #   3. 画面里**只有一张脸**（多张时不知道是谁的）
                                    # 第 4 道在 append() 里：与库里已有的太像就不加。
                                    if (who is not None
                                            and sc >= config.FACE_ADMIT
                                            and n_faces == 1):
                                        if fdb.append(who, f["feat"]):
                                            face_added += 1
                                            if face_added <= 3 or face_added % 10 == 0:
                                                print("FACE +1 -> %s (score=%.3f, 库=%d 条)"
                                                      % (who, sc, fdb.total_vecs()))
                                fobjs.append({
                                    "cls": "face",
                                    "score": f["score"] if "score" in f else None,
                                    "box": normalize(
                                        [bx[0], bx[1], bx[0] + bx[2], bx[1] + bx[3]],
                                        config.AI_WIDTH, config.AI_HEIGHT),
                                    "who": who,
                                    "match": match,
                                    "live": None if f["live"] is None else round(f["live"], 4),
                                    "ok": f["ok"],
                                    "src": "face",
                                })
                                face_seen += 1
                                if f["ok"]:
                                    face_ok += 1
                                if who is not None:
                                    face_known += 1
                        except BaseException as e:
                            if face_err == 0:
                                import sys as _s2
                                _s2.print_exception(e)
                                print("FACE ERR (之后不再逐条打)")
                            face_err += 1

                    # 增量入库后**隔一段时间**落盘 —— 别每帧写 SD（写坏卡、也拖慢循环）
                    if fdb is not None and time.ticks_diff(now, t_face_save) >= config.FACE_SAVE_EVERY_MS:
                        n_saved = fdb.flush()
                        t_face_save = now
                        if n_saved:
                            print("FACE DB flushed %d 人 / %d 条"
                                  % (n_saved, fdb.total_vecs()))

                    # ---- 身份跟随：脸 -> 身体框 -> 锁定 ----
                    # `faces` 和 `objs` 是**两个独立数组**，板子不会自动把
                    # "张三的脸"和"张三的身体"绑起来 —— 那一步在这里做。
                    # 必须放在人脸算完之后（同帧），否则拿到的是上一帧的脸。
                    if follow_who is not None:
                        fhit = None
                        for f in fobjs:
                            if f["who"] == follow_who and f["ok"]:
                                fhit = f
                                break
                        if fhit is None:
                            if time.ticks_diff(now, follow_deadline) >= 0:
                                print("TARGET follow %s 超时（%d ms 内没等到正脸）"
                                      % (follow_who, config.FOLLOW_TIMEOUT_MS))
                                follow_who = None
                            # 没超时就继续等 —— 人可能还没转过身来
                        else:
                            body = pick_body_for_face(fhit["box"], objs)
                            if body is None:
                                # 这一帧身体还没检出（检测器对运动目标有 63% 掉帧）。
                                # **不放弃**，下一帧再看；真正放弃交给上面的超时。
                                if time.ticks_diff(now, follow_deadline) >= 0:
                                    print("TARGET follow %s：一直只有脸、没有 person 框，放弃"
                                          % follow_who)
                                    follow_who = None
                            else:
                                l, t, r, b = body
                                if trk.init(arr, (l * config.AI_WIDTH,
                                                  t * config.AI_HEIGHT,
                                                  (r - l) * config.AI_WIDTH,
                                                  (b - t) * config.AI_HEIGHT)):
                                    det_auto = False
                                    trk_locks += 1
                                    print("TARGET follow %s -> 锁到身体框 "
                                          "[%.3f,%.3f,%.3f,%.3f] match=%s -> det_auto OFF"
                                          % (follow_who, l, t, r, b, fhit["match"]))
                                    follow_who = None
                                else:
                                    print("TARGET follow %s：身体框被拒（%s）"
                                          % (follow_who, trk.last_reject))
                                    follow_who = None

                    del arr
                    del img
                    if trk.locked:
                        trk_frames += 1

                    # 上报 = **本帧真实的检测结果** + 跟踪器的连续目标。
                    #
                    # ⚠️ 这里**不再沿用上一帧的 last_objs**。原来的写法会把陈旧坐标当新数据
                    # 发出去（检测器对运动目标有 63% 的帧给不出框，下游看到的是目标"闪断"）。
                    # **连续性现在交给跟踪器**，不再靠"保持上一帧"——那是两回事，不能混。
                    #
                    # w/h 用**推流**分辨率：前端拿归一化坐标 × 自己的显示尺寸
                    payload = encode(objs + t_objs, config.WIDTH, config.HEIGHT,
                                     frame_no, int(time.time() * 1000),
                                     faces=fobjs)
                    if not r_pi.ok:
                        r_pi.connect()
                    r_pi.send(payload)
                    if r_cloud is not None:
                        if not r_cloud.ok:
                            r_cloud.connect()
                        r_cloud.send(payload)
                else:
                    print("SNAPSHOT None")
                gc.collect()

            # 4) 30 s 一行 STATS
            if time.ticks_diff(now, t_stats) >= STATS_MS:
                t_stats = now
                print("STATS t=%ds frame=%d obj=%d sent=%d dropped=%d lost=%d "
                      "conns=%d rtsp=%d rtsp_empty=%d stall=%d report=%d/%d "
                      "trk=%d lock=%d rel=%d iou=%.2f tgt=%d/%d auto=%d "
                      "face=%d ok=%d known=%d add=%d err=%d mem=%d"
                      % (time.ticks_diff(now, t_start) // 1000, frame_no, obj_frames,
                         pusher.frames_sent, pusher.frames_dropped,
                         pusher.lost_events, pusher.connects,
                         rtsp.frames, rtsp.empty, wd.stalls,
                         r_pi.sent, r_pi.dropped,
                         trk_frames, trk_locks, trk_releases, trk_iou,
                         rx.recv_ok, rx.recv_bad, 1 if det_auto else 0,
                         face_seen, face_ok, face_known, face_added, face_err,
                         gc.mem_free()))

    except BaseException as e:
        import sys as _s
        _s.print_exception(e)
    finally:
        print("APP CLEANUP_START")
        try:
            pusher.close()
        except Exception as e:
            print("app: pusher.close EXC", e)
        try:
            rtsp.stop()
        except Exception as e:
            print("app: rtsp.stop EXC", e)
        try:
            rx.close()
        except Exception as e:
            print("app: rx.close EXC", e)
        try:
            if fdb is not None:
                n = fdb.flush()      # 退出前把增量入库的改动落盘
                print("app: FACE DB flushed %d 人" % n)
        except Exception as e:
            print("app: fdb.flush EXC", e)
        try:
            cam.stop()
        except Exception as e:
            print("app: cam.stop EXC", e)
        try:
            det.deinit()
        except Exception as e:
            print("app: det.deinit EXC", e)
        gc.collect()
        print("APP CLEANUP_DONE MEM %d" % gc.mem_free())


if __name__ == "__main__":
    main()
