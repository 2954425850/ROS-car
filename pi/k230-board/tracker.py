# /sdcard/k230vision/tracker.py
"""单目标跟踪封装（NanoTrack）。

**类别无关**：给一个框，跟住框里的东西 —— 全程不知道也不需要知道那是什么。
这是"检测器不认识的物体"（桃、辣椒……）唯一的出路。

## 四条来自实测的硬约束

1. **初始框必须紧贴目标。** 模板里混进背景，跟踪器就会去跟背景 ——
   而且 score 一样是 0.999。**这是调用方的义务**（YOLO 的框天然是紧的，正好合适）。
   见 `docs/plans/2026-09-19-tracker-nanotrack-result.md` §3.1。

2. **score 不能用来判断丢跟。** 实测：跟背景跟得最"稳"的那两跑，score 全程 0.999。
   **score 高 ≠ 跟得对。**

3. **判漂移要用"长宽比偏离"，不是"面积膨胀"。** 面积会因目标真实地变大而变大；
   长宽比对尺度不敏感。实测数据：

   | 场景 | 初始框 | 之后 | AR 偏离 | 面积倍数 |
   |---|---|---|---|---|
   | `t_track3` 漂到背景 | 30x30 (AR 1.00) | 87x25 (AR 3.5) | **+2.50** | 2.4 |
   | `t_track5` 漂到背景 | 32x32 (1.00) | 87x40 (AR 2.2) | **+1.20** | 3.4 |
   | `t_track6` **跟住** | 30x26 (1.15) | 稳定在 AR 1.0~1.2 | **< 0.2** | < 1.6 |
   | `t_track7` **误报那次** | 34x34 (1.00) | 面积涨 4.36 倍，**AR 仍 1.05** | **≈ 0.05** | **4.36** |

   最后一行是关键：那一次目标是真的变大了（被拿起靠近镜头），**面积翻 4 倍多但没漂**。
   所以本类**不再输出 `grow`** —— 一个会误导人的字段一定会被误用。

4. **不调 deinit。** `AIBase.deinit()` 内部会 `nn.shrink_memory_pool()`，
   在多模型常驻时会弄坏别的模型（相机端设计文档 §3）。

## 双重判据

- **`ar_dev`**：帧间随时可得（不依赖任何外部输入）
- **`iou`**：调用方若**同一帧**上有该目标的**可信参考框**（比如检测器的框），传进来，
  Tracker 会算两者的 IoU。**检测器的框提供了外部校准**，比自证可靠。

`lost` = `ar_dev > ar_tol` **或** （有 iou 且 `iou < iou_min`）。
**两个阈值都是暂定的**，需要更多实测来定；调用方**不要仅凭 `lost` 就丢弃目标**。

## 坐标约定

对外一律用**归一化 [l, t, r, b]**，与 `results.normalize`、检测器输出同形，
所以 `results.encode` 不用改，下游也分不出（也不该分出）框是谁给的。
"""
import sys

sys.path.insert(0, "/sdcard/k230vision/vendor")

from nanotracker import TrackCropApp, TrackSrcApp, TrackerApp

from results import normalize
from targets import box_window_fits

KM = "/sdcard/k230vision/vendor/kmodel/"
CROP_IN = [127, 127]
SRC_IN = [255, 255]


def _iou(a, b):
    """两个 [l, t, r, b] 的交并比。"""
    ix = min(a[2], b[2]) - max(a[0], b[0])
    iy = min(a[3], b[3]) - max(a[1], b[1])
    if ix <= 0 or iy <= 0:
        return 0.0
    inter = ix * iy
    ua = ((a[2] - a[0]) * (a[3] - a[1])
          + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / ua if ua > 0 else 0.0


class Tracker:
    """一次只跟一个目标。要换目标就再调一次 init()。"""

    def __init__(self, rgb888p_size, thresh=0.1, ar_tol=0.5, iou_min=0.3,
                 debug_mode=0):
        self.W = rgb888p_size[0]
        self.H = rgb888p_size[1]
        self.thresh = thresh
        self.ar_tol = ar_tol
        self.iou_min = iou_min

        self._locked = False
        self._template = None
        self._cwh = None
        self._init_ar = 1.0
        self._last = None
        self.last_reject = None      # 上一次 init() 被拒的原因（成功时清空）

        # 三个模型在构造时一次建好、常驻。不要频繁 new/del ——
        # "全部模型常驻"是这套架构的前提（实测 13 个 / 105MB 共存无压力）。
        ratio = float(SRC_IN[0]) / float(CROP_IN[0])
        self._crop = TrackCropApp(
            KM + "cropped_test127.kmodel", model_input_size=CROP_IN,
            ratio_src_crop=0.0, center_xy_wh=[1.0, 1.0, 1.0, 1.0],
            rgb888p_size=[self.W, self.H], display_size=[self.W, self.H],
            debug_mode=debug_mode)
        self._src = TrackSrcApp(
            KM + "nanotrack_backbone_sim.kmodel", model_input_size=SRC_IN,
            ratio_src_crop=ratio, rgb888p_size=[self.W, self.H],
            display_size=[self.W, self.H], debug_mode=debug_mode)
        self._head = TrackerApp(
            KM + "nanotracker_head_calib_k230.kmodel", crop_input_size=CROP_IN,
            thresh=thresh, rgb888p_size=[self.W, self.H],
            display_size=[self.W, self.H], debug_mode=debug_mode)

    # ---- 对外 ----
    @property
    def locked(self):
        return self._locked

    def init(self, frame, box_px):
        """用一帧 + 一个**紧**框初始化。

        frame  : AI 帧的 numpy（to_numpy_ref()，(3,H,W)）
        box_px : (x, y, w, h) 像素，AI 帧坐标系 —— 左上角 + 宽高
        返回 True 表示锁定成功。

        **返回 False 的两种情况**（原因写在 `self.last_reject`）：
        - 框 < 4px
        - 框没有整个落在画面内（见 init 里的注释）
        调用方拿到 False 就当"没锁上"，**不要**自己再往下走。
        """
        x, y, w, h = [float(v) for v in box_px]
        if w < 4 or h < 4:
            self.last_reject = "box too small (%.1fx%.1f)" % (w, h)
            return False
        cx = x + w / 2.0
        cy = y + h / 2.0

        # ⚠️ 目标框必须**整个**在画面内。
        # 历史：`config_preprocess()` 会在 (cx,cy) 上居中裁一个 s_z 见方的大正方形
        # （s_z ≈ 2*边长）。以前方框一出画，起点就是负数，`ai2d.crop()` 会把整个
        # MicroPython VM 弄死（2026-09-19 实测 {"pt":[0.05,0.05]} → crop 起点 -22 → 断电）。
        # **现在那条路已经补了 pad**（TrackCropApp 照抄 TrackSrcApp 补灰边），
        # 所以裁剪窗口出画不再是问题 —— 但**目标本身还是得完整可见**：
        # 灰边只是"目标周围的背景"，补掉无所谓；拿半个身子当模板是跟不住的。
        # **这里是唯一的收口点**：外部命令（app.py 的 rx.take()）和检测器自动锁定
        # （app.py 拿检测框那条）都从 init() 进来，所以拦在这里两条路都安全。
        if not box_window_fits(x, y, w, h, self.W, self.H):
            self.last_reject = ("box would leave the %dx%d frame: %.1fx%.1f at "
                                "(%.1f,%.1f)" % (self.W, self.H, w, h, x, y))
            return False
        self.last_reject = None

        # TrackCropApp 的 crop 参数是从实例属性 center_xy_wh 算出来的，
        # 所以在调用 config_preprocess() 之前把它换成我们的框即可。
        self._crop.center_xy_wh = [cx, cy, w, h]
        self._crop.config_preprocess()

        self._template = self._crop.run(frame)
        self._cwh = [cx, cy, w, h]
        self._init_ar = w / h
        self._locked = True
        self._last = None
        return True

    def release(self):
        """放弃当前目标（不销毁模型 —— 见文件头第 4 条）。"""
        self._locked = False
        self._template = None
        self._cwh = None
        self._last = None

    def update(self, frame, ref_box=None):
        """跟一帧。未锁定时返回 None。

        frame   : AI 帧的 numpy
        ref_box : 可选。**同一帧**上该目标的**可信参考框**（归一化 [l,t,r,b]，
                  与检测器输出同形）。有此输入时 `iou` 才有值 —— 这是外部校准，
                  比跟踪器自证可靠得多。

        返回（坐标已归一化）：
            {"box": [l,t,r,b], "score": float,
             "ar_dev": float,      # |当前AR / 初始AR - 1|，尺度不敏感
             "iou": float 或 None,  # 与 ref_box 的交并比（无 ref 时为 None）
             "lost": bool,
             "degenerate": bool}   # True = head 这帧给不出框（跟丢到底了），
                                   # 此时 score=-1、ar_dev=999，box 是**上一次**的位置
        """
        if not self._locked:
            return None

        self._src.config_preprocess(self._cwh)
        src_out = self._src.run(frame)
        det = self._head.run(self._template, src_out, self._cwh)

        # ⚠️ 2026-09-19 实测：目标跟到接近画面边缘之后，`nanotracker_postprocess`
        # 会判为不可靠而**只吐一个元素**，下面 det[1] 直接抛 IndexError ——
        # **在 app.py 的主循环里那会把整个 app 打死**（当天的实测脚本就是这么结束的）。
        # 所以这里降级成"丢了"，交给上层按 lost 走，**绝不往上抛**。
        # box 沿用上一次的位置：上层拿到 lost 会 release，不会把它当有效坐标用。
        if det is None or len(det) < 2:
            cx, cy, w, h = self._cwh
            out = {
                "box": normalize([cx - w / 2.0, cy - h / 2.0,
                                  cx + w / 2.0, cy + h / 2.0], self.W, self.H),
                "score": -1.0,
                "ar_dev": 999.0,
                "iou": None,
                "lost": True,
                "degenerate": True,   # 新字段：这次不是"漂了"，是 head 根本给不出框
            }
            self._last = out
            return out

        tb = det[0]
        cwh = det[1]
        self._cwh = cwh

        cx = float(cwh[0])
        cy = float(cwh[1])
        w = float(cwh[2])
        h = float(cwh[3])
        try:
            score = float(tb[4])
        except BaseException:
            score = -1.0

        box = normalize([cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0],
                        self.W, self.H)

        ar_dev = abs((w / max(1e-6, h)) / max(1e-6, self._init_ar) - 1.0)

        iou = None
        if ref_box is not None:
            iou = _iou(box, ref_box)

        lost = (ar_dev > self.ar_tol) or (iou is not None and iou < self.iou_min)

        out = {
            "box": box,
            "score": round(score, 3),
            "ar_dev": round(ar_dev, 3),
            "iou": None if iou is None else round(iou, 3),
            "lost": lost,
            "degenerate": False,
        }
        self._last = out
        return out
