# /sdcard/k230vision/vision.py
"""KPU 推理封装。

**照抄官方 `/sdcard/examples/05-AI-Demo/object_detect_yolov8n.py` 的 `ObjectDetectionApp`**
（AIBase 模式）。前处理 / 后处理 / 标签表 / 阈值**全部原样搬运，不做任何改动**
—— 这是本任务里唯一不该动脑的地方。

只做三处改动：

1. `draw_result` 留空 —— 框由云端前端叠加，不烧进码流；
2. 结果转成**归一化坐标**（走 `results.normalize`）；
3. 新增 `detect(img)`，返回 `[{"cls", "score", "box"}]`。

## 为什么是 COCO YOLOv8n 而不是人脸 / 人体模型

验收时镜头前放的是**苹果**。人脸模型（`face_detection_320.kmodel`）和人体模型
（`person_detect_yolov5n.kmodel`）都认不出苹果，等于无法核对坐标正确性。
COCO 80 类里 **`apple` 是合法类别（index 47）**，所以用 `object_detect_yolov8n.py`
这套（模型 `yolov8n_320.kmodel`）。计划草稿原本写的是照抄 `ai_rtsp.py` 的人脸类，
此处按「必须能检出镜头前那个物体」的原则改抄同一仓库里的 COCO 检测例程，
AIBase 骨架与三处改动的要求完全一致。

## 三条必须记住的机制（都来自官方例程）

1. **`preprocess()` 被重写了**：`return [nn.from_numpy(input_np)]`
   —— sensor 帧**原样**喂给模型，`config_preprocess` 里配的 ai2d pad/resize
   在这条路径上**根本没被调用**。所以 **AI 帧分辨率必须等于模型输入分辨率**
   （320x320），否则 `nn.from_numpy` 的 shape 与模型输入对不上。
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

    # 配置预处理（照抄官方；注意本路径下 preprocess 被重写，ai2d 实际未被调用）
    def config_preprocess(self, input_image_size=None):
        with ScopedTiming("set preprocess config", self.debug_mode > 0):
            ai2d_input_size = input_image_size if input_image_size else self.rgb888p_size
            top, bottom, left, right, self.scale = letterbox_pad_param(
                self.rgb888p_size, self.model_input_size)
            self.ai2d.pad([0, 0, 0, 0, top, bottom, left, right], 0, [128, 128, 128])
            self.ai2d.resize(nn.interp_method.tf_bilinear, nn.interp_mode.half_pixel)
            self.ai2d.build([1, 3, ai2d_input_size[1], ai2d_input_size[0]],
                            [1, 3, self.model_input_size[1], self.model_input_size[0]])

    # 预处理被重写：原样把 sensor 帧转成 tensor（照抄官方）
    def preprocess(self, input_np):
        with ScopedTiming("preprocess", self.debug_mode > 0):
            return [nn.from_numpy(input_np)]

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
