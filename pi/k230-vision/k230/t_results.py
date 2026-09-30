# /sdcard/k230vision/t_results.py
"""results.py 的纯逻辑验证（不需要模型，不需要管线）。"""
import sys
sys.path.insert(0, "/sdcard/k230vision")
from results import normalize, encode

print("N1", normalize([0, 0, 640, 360], 1280, 720))          # 期望 [0.0,0.0,0.5,0.5]
print("N2", normalize([-10, -10, 2000, 2000], 1280, 720))    # 期望裁剪到 [0.0,0.0,1.0,1.0]
print("N3", normalize([160, 90, 640, 360], 640, 360))        # 期望 [0.25,0.25,1.0,1.0]
print("N4", normalize([0.0, 0.0, 0.0, 0.0], 1280, 720))      # 期望 [0.0,0.0,0.0,0.0]
print("E1", encode([{"cls": "person", "score": 0.9, "box": [0.1, 0.2, 0.3, 0.4]}],
                   1280, 720, 1, 1789631000123))
print("E2", encode([], 1280, 720, 2, 1789631000456))
