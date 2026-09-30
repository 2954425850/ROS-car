# /sdcard/k230vision/results.py
"""结构化结果的编码。

## 为什么坐标要归一化

推理分辨率（640x640 的模型输入 / 640x360 的 AI 帧）和推流分辨率（720p）不同，
且以后任一方都可能改。发归一化坐标后，前端只需乘上自己的显示尺寸，两边彻底解耦。

## 约定（设计文档 §4.2）

    {"ts": <epoch_ms>, "frame": <n>, "w": <推流宽>, "h": <推流高>,
     "objs": [{"cls": "person", "score": 0.91, "box": [l, t, r, b]}, ...]}

`box` 是**归一化**的 [left, top, right, bottom]，四个数都裁剪到 0.0~1.0。
"""
import ujson


def encode(objs, width, height, frame_no, ts_ms):
    """打包成 JSON bytes，直接可以往 socket 里写。"""
    return ujson.dumps({
        "ts": ts_ms,
        "frame": frame_no,
        "w": width,
        "h": height,
        "objs": objs,
    }).encode()


def normalize(box_px, width, height):
    """像素框 [x1, y1, x2, y2] -> 归一化 [l, t, r, b]，裁剪到 0~1。

    越界的框（NMS 后仍可能探出画面）在这里夹紧，前端不必再判。
    """
    l, t, r, b = box_px
    f = lambda v, m: 0.0 if v < 0 else (1.0 if v > m else v / m)
    return [round(f(l, width), 4), round(f(t, height), 4),
            round(f(r, width), 4), round(f(b, height), 4)]
