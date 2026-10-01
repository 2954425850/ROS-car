# /sdcard/k230vision/faces.py
"""人脸：检测 -> 对齐 -> 特征 -> 与特征库比对。**每人注册多帧。**

## 为什么每人要多帧（实测依据）

单参考样本覆盖不了姿态范围。实测（同一个人、9 秒内自然的头部转动）：

    cos(首帧)  = 0.494 ~ 0.909   均值 0.728     -> 例程 score 0.747 ~ 0.955，门限 0.75
    cos(上一帧) = 0.486 ~ 0.972   均值 0.899     -> 例程 score 0.743 ~ 0.986

**最低值 0.7428 已经掉到门限 0.75 底下了** —— 约 1/29 的帧会被判成"不认识"。
存多帧、比对时取**最大**相似度，就能把这个覆盖住。

## 打分的尺度（照抄例程 `face_recognition_lite.py`）

    特征两边都做 L2 归一化 -> np.dot(a,b) 就是余弦
    score = cos/2 + 0.5          # 把 [-1,1] 映射到 [0,1]
    门限 face_recognition_threshold = 0.75   =>  等价于 cos >= 0.5

⚠️ **必须归一化**：实测 `|feat| = 11 ~ 15`，不是单位长度。例程的
`database_search()` 里是归一化过的（`feature /= np.linalg.norm(feature)`），
照抄它的**完整**逻辑，别只抄那一行 dot。

## 特征维度

`face_recognition_mobile.kmodel` 输出 **512 维**（实测）。
例程注释里写 `feature_num = 128`，**是错的**。

## 存储

    /sdcard/k230vision/facedb/<名字>.bin     # M x 512 个 float32（**已归一化**）
    /sdcard/k230vision/facedb/meta.txt       # 每行: <名字> <帧数>

一个名字一个文件，文件里是 M 帧的特征首尾相接。名字就是身份字符串。
"""
import gc
import os
import sys
import time

sys.path.insert(0, "/sdcard/k230vision/vendor")

import ulab.numpy as np

import face_registration_lite as frl
import face_liveness_rgb as flr

KM = "/sdcard/k230vision/vendor/kmodel/"
ANCHORS_PATH = "/sdcard/k230vision/vendor/prior_data_320.bin"
DET_IN = [320, 320]
REG_IN = [112, 112]
LIVE_IN = [112, 112]
FEAT_DIM = 512
DB_DIR = "/sdcard/k230vision/facedb"


def l2norm(v):
    """把 ulab 一维数组 L2 归一化，返回**新的 ulab 数组**。

    ## 为什么全程用 ulab 数组，不用 Python 浮点列表

    实测（`t_ulab.py`，512 个 float）：

        **Python 列表**  20532 字节   （每个浮点是一个独立对象，~40 字节）
        **ulab 数组**     2048 字节   （float32 原始缓冲）

    **差 10 倍。** 每人 80 条特征时：**1.64 MB vs 160 KB** ——
    前者一块 4 MB 的堆装不下两个人。**这就是"只能记住几个人"的全部原因。**

    顺带：比对也从"纯 Python 循环 80×512 次乘法"变成一次 `np.dot`（C 层）。
    """
    n = float(np.linalg.norm(v))
    if n <= 0.0:
        return v
    return v / n


def _dot(a, b):
    """两个已归一化的一维 ulab 数组的点积 = 余弦。"""
    return float(np.dot(a, b))


def score_from_cos(c):
    """余弦 -> 例程的 0~1 分。"""
    return c / 2.0 + 0.5


def quality(item, W, H, min_face_h=40, min_face_w=30, margin=2):
    """判断一张脸**够不够格拿去注册**。返回 (ok, 原因)。

    为什么需要它：实测第 1 位注册时脸框一直是 `y=0`、高到 173/180 —— **脸被上边缘切了**，
    那批帧把"同一人"的最低分拖到 0.7848（第 2 位是 0.9189）。
    **是注册质量在拖低同一人的下限，把它推向不同人的上限。**

    检查三类（从实测症状反推）：
      1. **5 个关键点**都要在画面内留出 margin —— 对齐矩阵靠它们算，
         有关键点出画 = 对齐必错。**比只检查框更本质。**
      2. 框整体在画面内留出 margin —— 脸被切就会这样
      3. 脸别太小（细节不够）、也别太大（必然贴边）
    """
    b = item["box"]
    x, y, w, h = b[0], b[1], b[2], b[3]
    if x < margin or y < margin or x + w > W - margin or y + h > H - margin:
        return (False, "脸框贴边(被切)")
    if w < min_face_w or h < min_face_h:
        return (False, "脸太小 w=%.0f h=%.0f" % (w, h))
    lm = item.get("landm")
    if lm:
        for i in range(len(lm)):
            v = lm[i]
            if i % 2 == 0:
                if v < margin or v > W - margin:
                    return (False, "关键点出画(横)")
            else:
                if v < margin or v > H - margin:
                    return (False, "关键点出画(纵)")
    return (True, "")


def capture_gallery(rec, sensor, chn, W, H, seconds=5.0, m_target=24, min_keep=3):
    """在**固定时间窗**内采一批合格特征，最后均匀抽稀到 m_target 条。

    ## 为什么要按时间收口

    原来的实现是"采到 M 帧为止（最多试 3M 次）"，**耗时取决于被丢弃多少帧** ——
    实测丢弃率 62% 时花了 **13 秒**。注册耗时必须是**可预期的**，所以改成按时间收口：
    到点就停，采到多少算多少。

    ## 为什么结束时要抽稀

    窗内可能收到 50+ 帧，但它们只隔几十毫秒、姿态几乎一样。
    **均匀抽稀让留下的帧铺满整个时间窗** —— 覆盖的姿态范围比"取前 24 帧"大得多，
    而"多帧"的全部价值就在于覆盖姿态。

    返回 (feats, n_try, reasons)：合格特征列表、试了多少次、各原因的丢弃次数。
    """
    t0 = time.ticks_ms()
    budget = int(seconds * 1000)
    feats = []
    reasons = {}
    n_try = 0
    while time.ticks_diff(time.ticks_ms(), t0) < budget:
        im = sensor.snapshot(chn=chn)
        if im is None:
            continue
        arr = im.to_numpy_ref()
        f4 = arr.reshape((1, 3, H, W))
        del arr
        del im
        faces = rec.analyze(f4)
        del f4
        n_try += 1
        if not faces:
            reasons["无脸"] = reasons.get("无脸", 0) + 1
            gc.collect()
            continue
        # 取面积最大的那张脸（离镜头最近 = 要注册的人）
        b = faces[0]
        for f in faces:
            if f["box"][2] * f["box"][3] > b["box"][2] * b["box"][3]:
                b = f
        if not b["ok"]:
            reasons[b["why"]] = reasons.get(b["why"], 0) + 1
        else:
            feats.append(b["feat"])
        gc.collect()

    n_good = len(feats)
    if n_good > m_target:
        # 均匀抽稀：把整个时间窗铺满
        kept = []
        for i in range(m_target):
            kept.append(feats[(i * n_good) // m_target])
        feats = kept
    return (feats, n_try, reasons)


class FaceRecognizer:
    """检测 + 对齐 + 特征 + 活体。不做比对（那是 FaceDB 的事）。"""

    def __init__(self, frame_w, frame_h, need_liveness=True, debug_mode=0):
        self.W = frame_w
        self.H = frame_h
        anchors = np.fromfile(ANCHORS_PATH, dtype=np.float).reshape((4200, 4))
        self.det = frl.FaceDetApp(KM + "face_detection_320.kmodel",
                                  model_input_size=DET_IN, anchors=anchors,
                                  confidence_threshold=0.5, nms_threshold=0.2,
                                  rgb888p_size=[frame_w, frame_h])
        self.reg = frl.FaceRegistrationApp(KM + "face_recognition_mobile.kmodel",
                                           model_input_size=REG_IN,
                                           rgb888p_size=[frame_w, frame_h])
        self.det.config_preprocess(input_image_size=[frame_w, frame_h])
        self.live = None
        if need_liveness:
            self.live = flr.FaceLivenessApp(KM + "face_liveness_rgb.kmodel",
                                            model_input_size=LIVE_IN,
                                            rgb888p_size=[frame_w, frame_h])

    def analyze(self, frame4):
        """frame4: (1,3,H,W) 的 numpy（调用方负责 reshape）。

        返回一个列表，每张脸一个 dict：
            {"box": [x,y,w,h] 像素, "feat": [512] 已归一化,
             "live": float 或 None, "is_live": bool 或 None}
        """
        out = []
        res = self.det.run(frame4)
        if not res:
            return out
        boxes, landms = res
        for i in range(len(boxes)):
            b = boxes[i]
            self.reg.config_preprocess(landms[i], input_image_size=[self.W, self.H])
            feat = self.reg.run(frame4)
            # ⚠️ 全程 ulab 数组，**不转成 Python 浮点列表** —— 差 10 倍内存（见 l2norm）
            item = {
                "box": [float(b[0]), float(b[1]), float(b[2]), float(b[3])],
                "landm": [float(landms[i][j]) for j in range(len(landms[i]))],
                "feat": l2norm(feat.flatten()),
                "live": None,
                "is_live": None,
            }
            if self.live is not None:
                self.live.config_preprocess(b, input_image_size=[self.W, self.H])
                lv = self.live.run(frame4)
                try:
                    vals = [float(v) for v in lv.flatten()]
                except BaseException:
                    vals = [float(v) for v in lv]
                if len(vals) >= 2:
                    # 实测真脸时 [小, 大] -> 第 2 个是"活体"
                    item["live"] = vals[1]
                    item["is_live"] = vals[1] >= 0.5
            # ⭐ 自带质量门：调用方不必记得检查。
            # 脸不完整时 ok=False —— **这种帧不该拿去做身份判定**
            # （报"这张脸不可用"好过给出一个低分匹配；本项目原则是认不出好过认错）。
            item["ok"], item["why"] = quality(item, self.W, self.H)
            out.append(item)
        return out


class FaceDB:
    """特征库。一个名字一个文件，文件里是 M 帧**已归一化**的特征。

    ## 为什么库里存的是 ulab 2 维数组，不是 Python 浮点列表

    实测（`t_ulab.py`）：512 个 float 的 **Python 列表 = 20532 字节**，
    **ulab 数组 = 2048 字节**，**差 10 倍**（Python 浮点是独立对象，~40 字节/个）。
    每人 80 条时是 **1.64 MB vs 160 KB** —— 前者一块 4 MB 的堆装不下两个人。
    **这就是"只能记住几个人"的全部原因，换掉存法就没了。**

    顺带比对也从"纯 Python 循环 80x512 次乘法"变成一次 `np.dot`。
    """

    def __init__(self, db_dir=DB_DIR, threshold=0.78, db_max=80):
        self.db_max = db_max          # 每人最多存多少条（超了丢最早的）
        self._dirty = False           # 有增量没落盘
        # ⚠️ 门限 0.78 是**暂定的**，依据见下（实测两组分布）：
        #
        #   不同人（真实测到）  0.6541 ~ 0.7384   均值 0.6968
        #   同一人（id1 过滤后）0.7856 ~ 0.9642
        #   同一人（id2 过滤前）0.9189 ~ 0.9747
        #
        # 分离窗口 = [0.7384, 0.7856]，**只有 0.047 宽**。
        # 本项目的原则是"**认错人比认不出严重得多**"，所以**往窗口上端靠**取 0.78：
        # 代价是更多"不认识"，换来冒名顶替的空间更小。
        #
        # **但真正撑窄这个窗口的是取景**：实测注册时 62% 的帧因为脸贴边/被切而不可用。
        # 把相机摆好会同时抬高同一人的下限、也改善查询帧质量。
        # 这个数字不是终值 —— 需要重测。
        self.dir = db_dir
        self.threshold = threshold
        self.names = []          # [(name, ulab 2维数组 (n, FEAT_DIM)), ...]
        self._ensure_dir()
        self.reload()

    def _ensure_dir(self):
        if self.dir not in os.listdir("/sdcard/k230vision"):
            try:
                os.mkdir(self.dir)
            except BaseException:
                pass

    def _path(self, name):
        return self.dir + "/" + name + ".bin"

    @staticmethod
    def _stack(vecs):
        """一串一维 ulab 数组 -> 一个 (n, FEAT_DIM) 的二维数组。"""
        blob = bytearray()
        for v in vecs:
            blob += v.tobytes()
        return np.frombuffer(bytes(blob), dtype=np.float).reshape((len(vecs), FEAT_DIM))

    def reload(self):
        self.names = []
        try:
            files = os.listdir(self.dir)
        except BaseException:
            return
        for fn in files:
            if not fn.endswith(".bin"):
                continue
            name = fn[:-4]
            try:
                data = open(self._path(name), "rb").read()
            except BaseException:
                continue
            v = np.frombuffer(data, dtype=np.float)
            n = len(v) // FEAT_DIM
            if n <= 0:
                continue
            # 直接 reshape 成二维，**不经过 Python 浮点列表**
            arr = v[:n * FEAT_DIM].reshape((n, FEAT_DIM))
            if n > self.db_max:
                # 文件可能是**在旧上限下**写的（比如上限从 80 改成 40 之后）——
                # 载入时按当前上限截到**最新的** db_max 条，否则上限形同虚设、
                # 内存也白占（实测每条 2KB，多 40 条就是 80KB）。
                kept = []
                for i in range(n - self.db_max, n):
                    kept.append(arr[i])
                arr = self._stack(kept)
            self.names.append((name, arr))

    def count(self):
        return len(self.names)

    def total_vecs(self):
        n = 0
        for _name, arr in self.names:
            n += arr.shape[0]
        return n

    def add(self, name, feats):
        """把一批**已归一化**的特征加到某个名字下（已存在则覆盖）。"""
        arr = self._stack(feats)
        with open(self._path(name), "wb") as fh:
            fh.write(arr.tobytes())
        self.reload()

    def remove(self, name):
        try:
            os.remove(self._path(name))
        except BaseException:
            pass
        self.reload()

    def append(self, name, feat, dup_cos=0.99):
        """给某个名字**增量**加一条特征（"边用边长"）。返回 True 表示真的加了。

        三道限制，少一道都会把库搞坏：

        1. **与库里已有的太像就不加**（余弦 > `dup_cos`）—— 多帧的价值在
           **覆盖不同姿态**，不是堆一堆几乎一样的帧白占位置。
        2. 超过 `db_max` 就丢**最早**的（旧姿态让位给新姿态）。
        3. 只改内存，**落盘靠 `flush()`** —— 别每帧写 SD。

        ⚠️ **调用方的责任**：只有**非常有把握**的帧才该进来。把误认的陌生人
        写进谁的库，以后就更像他 —— **增量入库会自我强化错误**，这是它唯一的真危险。
        """
        for idx in range(len(self.names)):
            nm, arr = self.names[idx]
            if nm != name:
                continue
            if arr.shape[0] > 0:
                if float(np.max(np.dot(arr, feat))) > dup_cos:
                    return False
            rows = arr.shape[0]
            start = 1 if rows >= self.db_max else 0     # 满了就丢最早那一条
            kept = []
            for i in range(start, rows):
                kept.append(arr[i])
            kept.append(feat)
            self.names[idx] = (nm, self._stack(kept))
            self._dirty = True
            return True
        return False

    def flush(self):
        """把内存里的库落盘。返回写了几个人。没改动就什么都不做。"""
        if not self._dirty:
            return 0
        n = 0
        for name, arr in self.names:
            with open(self._path(name), "wb") as fh:
                fh.write(arr.tobytes())
            n += 1
        self._dirty = False
        return n

    def match(self, feat):
        """拿一个**已归一化**的特征去查库。

        返回 (name, score, is_known)：
          name     —— 最像的那个名字（谁也不像时仍是分数最高的那个，便于诊断）
          score    —— 例程尺度的分数（cos/2+0.5），全体里最大的
          is_known —— score >= threshold
        """
        best_name = None
        best = -1.0
        for name, arr in self.names:
            if arr.shape[0] == 0:
                continue
            # 一次 np.dot 拿到该人全部帧的余弦（C 层），比 Python 循环快一个量级
            m = float(np.max(np.dot(arr, feat)))
            if m > best:
                best = m
                best_name = name
        if best_name is None:
            return (None, 0.0, False)
        sc = score_from_cos(best)
        return (best_name, sc, sc >= self.threshold)
