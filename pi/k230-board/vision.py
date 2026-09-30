# /sdcard/k230vision/vision.py
"""KPU 推理封装。

**照抄官方 `/sdcard/examples/05-AI-Demo/object_detect_yolov8n.py` 的 `ObjectDetectionApp`**
（AIBase 模式）。后处理 / 标签表 / 阈值**原样搬运，不做任何改动**。

改动（相对官方例程）：

1. **删掉了 `preprocess()` 的重写**（见下"三条必须记住的机制"第 1 条）——
   官方那版会让 ai2d letterbox 变成死代码，是"画面纵向拉长 1.78 倍、橘子变竖蛋"的根因；
2. `draw_result` 留空 —— 框由云端前端叠加，不烧进码流；
3. 结果转成**归一化坐标**（走 `results.normalize`）；
4. 新增 `detect(img)`，返回 `[{"cls", "score", "box"}]`。

## 为什么是 COCO YOLOv8n 而不是人脸 / 人体模型

验收时镜头前放的是**苹果**。人脸模型（`face_detection_320.kmodel`）和人体模型
（`person_detect_yolov5n.kmodel`）都认不出苹果，等于无法核对坐标正确性。
COCO 80 类里 **`apple` 是合法类别（index 47）**，所以用 `object_detect_yolov8n.py`
这套（模型 `yolov8n_320.kmodel`）。计划草稿原本写的是照抄 `ai_rtsp.py` 的人脸类，
此处按「必须能检出镜头前那个物体」的原则改抄同一仓库里的 COCO 检测例程，
AIBase 骨架与三处改动的要求完全一致。

## 三条必须记住的机制

1. **`preprocess()` 保持 AIBase 的默认实现**（`return [self.ai2d.run(input_np)]`），
   ai2d 的 letterbox **必须真的执行**。官方例程把它重写成了 `[nn.from_numpy(input_np)]`
   （原样喂 sensor 帧），让 `config_preprocess()` 里配的 pad/resize 成了死代码 ——
   于是 AI 帧只能与模型输入同尺寸，`cam.py` 就会把 16:9 的画面硬塞进 1:1，
   **整幅画面纵向拉长 1.78 倍**（实测：橘子是球，却渲染成竖着的蛋）。
   **AI 帧 320x180（16:9）+ ai2d letterbox → 模型 320x320。**
   注意 `letterbox_pad_param` 把灰边全堆在**底部**（top=0、left=0），别改成居中 ——
   `aidemo.yolov8_det_postprocess` 按同一约定解读，改成居中会让框系统性偏移。
2. **后处理返回的框是「显示分辨率」坐标，且格式是 `(x, y, w, h)`**：
   官方 `draw_result` 里写的是 `x, y, w, h = dets[0][i]` 再 `draw_rectangle(x,y,w,h)`，
   而 `yolov8_det_postprocess` 的第 4 个参数就是 display_size。
   本文件把 `display_size` 设成 **AI 帧尺寸**，于是框直接落在 AI 帧像素坐标系里，
   归一化时除以 AI 帧尺寸即可 —— 不需要任何手工缩放。
3. **后处理返回的是三元组** `(boxes, cls_ids, scores)`，不是字典列表：
   `dets[0]` = 框列表，`dets[1]` = 类别 id 列表，`dets[2]` = 分数列表。
"""
from media.sensor import *      # ALIGN_UP 定义在这里（cam.py 也是这么拿到的）
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.Utils import *
import nncase_runtime as nn
import ulab.numpy as np
import image
import aidemo

from results import normalize


class Detector(AIBase):
    def __init__(self, kmodel_path, labels, model_input_size, max_boxes_num,
                 confidence_threshold=0.3, nms_threshold=0.4,
                 rgb888p_size=[320, 320], debug_mode=0):
        super().__init__(kmodel_path, model_input_size, rgb888p_size, debug_mode)
        self.kmodel_path = kmodel_path
        self.labels = labels
        # 模型输入分辨率
        self.model_input_size = model_input_size
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.max_boxes_num = max_boxes_num
        # sensor 给到 AI 的图像分辨率（宽度 16 对齐）
        self.rgb888p_size = [ALIGN_UP(rgb888p_size[0], 16), rgb888p_size[1]]
        # 后处理输出坐标系 = AI 帧坐标系（见文件头第 2 条）
        self.display_size = [self.rgb888p_size[0], self.rgb888p_size[1]]
        self.debug_mode = debug_mode
        self.color_four = get_colors(len(self.labels))
        self.x_factor = float(self.rgb888p_size[0]) / self.model_input_size[0]
        self.y_factor = float(self.rgb888p_size[1]) / self.model_input_size[1]
        self.ai2d = Ai2d(debug_mode)
        self.ai2d.set_ai2d_dtype(nn.ai2d_format.NCHW_FMT, nn.ai2d_format.NCHW_FMT,
                                 np.uint8, np.uint8)

    # 配置预处理。**这条路径是活的**：preprocess() 没被重写，ai2d 会真的执行，
    # 所以这里的 pad(灰边)+resize 就是 letterbox 本体。
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # ⚠️ 这里**故意不重写** preprocess()，保持 AIBase 的默认实现：
    #     return [self.ai2d.run(input_np)]
    #
    # 官方 05-AI-Demo/object_detect_yolov8n.py 把 preprocess 重写成了
    # `[nn.from_numpy(input_np)]`（原样喂 sensor 帧）。那条路径会让 config_preprocess()
    # 里配的 ai2d pad/resize **完全不执行**，于是 AI 帧必须与模型输入同尺寸，
    # 而 cam.py 就会把 16:9 的画面硬塞进 1:1 —— 纵向拉长 1.78 倍
    # （实测苹果/橘子这类球形物体变成竖椭圆，而竖向物体反而更"像"，所以症状是
    #  "有的检得对、有的完全不行"，不是"整体都差"）。
    #
    # 要动这里，先回读设计文档 §6 与 t_geom.py 的判据（那个判据能红）。

    # 后处理（照抄官方）
    def postprocess(self, results):
        with ScopedTiming("postprocess", self.debug_mode > 0):
            new_result = results[0][0].transpose()
            det_res = aidemo.yolov8_det_postprocess(
                new_result.copy(),
                [self.rgb888p_size[1], self.rgb888p_size[0]],
                [self.model_input_size[1], self.model_input_size[0]],
                [self.display_size[1], self.display_size[0]],
                len(self.labels), self.confidence_threshold,
                self.nms_threshold, self.max_boxes_num)
            return det_res

    def detect(self, img):
        """img: sensor 的 AI 通道（CAM_CHN_ID_2, RGBP888, 320x320）帧。

        返回 [{"cls": str, "score": float, "box": [l,t,r,b]}]，box **归一化**到 0~1。
        """
        dets = self.run(img)
        # 原文留档，供 t_vision.py 核对 postprocess 的字段顺序（不影响生产逻辑）
        self.last_dets = dets
        out = []
        if not dets or len(dets) < 3:
            return out
        boxes, cls_ids, scores = dets[0], dets[1], dets[2]
        w, h = self.display_size[0], self.display_size[1]
        for i in range(len(boxes)):
            # 官方 draw_result 的用法：x, y, w, h = dets[0][i]（是宽高，不是右下角）
            x, y, bw, bh = boxes[i]
            x = float(x); y = float(y); bw = float(bw); bh = float(bh)
            cls_id = int(cls_ids[i])
            out.append({
                "cls": self.labels[cls_id] if cls_id < len(self.labels) else str(cls_id),
                "score": round(float(scores[i]), 3),
                "box": normalize([x, y, x + bw, y + bh], w, h),
            })
        return out

    def draw_result(self, pl, dets):
        """本项目不用 —— 框交给云端前端叠加，不烧进码流。

        保留空实现是为了满足 AIBase 的接口形状（官方例程在主循环里会调它）。
        """
        pass
