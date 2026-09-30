# -*- coding: utf-8 -*-
"""抓一帧看看现在相机对着哪（不记录样本）。

    look.py            # 抓帧 + 颜色找盖 + 存一张转正的图

只做三件事：确认画面里有没有瓶盖、把它圈出来、把图存成能看的尺寸。
图在 /tmp/k230/manual/look-up.jpg（已按「相机倒置」转 180°）。
"""
import math
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')

from arm_grasp.collect import blue_blob_px, capture_frame, k230_host_from_result

RAW = '/tmp/k230/manual/look.jpg'
UP = '/tmp/k230/manual/look-up.jpg'


def main():
    host = k230_host_from_result()
    if not capture_frame(RAW, host):
        print('❌ 拍照失败')
        return 1
    blob = blue_blob_px(RAW)
    from PIL import Image, ImageDraw
    im = Image.open(RAW).rotate(180)
    d = ImageDraw.Draw(im)
    if blob is None:
        print('瓶盖：**不在画面里**')
    else:
        u, v = blob[0], blob[1]
        print('瓶盖：在画面 (%.0f, %.0f)，面积 %d px' % (u, v, blob[2]))
        u2, v2 = im.size[0] - u, im.size[1] - v      # 原图坐标 -> 转正后的坐标
        r = math.sqrt(blob[2] / math.pi)             # ★ 等效半径，别用固定值
        d.ellipse([u2 - r, v2 - r, u2 + r, v2 + r], outline=(255, 0, 0), width=6)
        d.text((12, 12), 'blob %.0f,%.0f  area %d' % (u, v, blob[2]), fill=(255, 0, 0))
    # 十字线标画面中心，方便判断偏了多少
    cx, cy = im.size[0] // 2, im.size[1] // 2
    d.line([cx - 40, cy, cx + 40, cy], fill=(0, 255, 0), width=3)
    d.line([cx, cy - 40, cx, cy + 40], fill=(0, 255, 0), width=3)
    im.resize((640, 360)).save(UP, quality=90)
    print('图：%s' % UP)
    return 0


sys.exit(main())
