# -*- coding: utf-8 -*-
"""检验 K230 出来的像素是不是【理想针孔】。

    imgpipe.py [--out /tmp/k230/manual/pipe.jpg]

原理（不需要标定、不需要姿态、不需要距离）：
  理想针孔下，三维空间里
    · 直线 -> 直线
    · 一个平面上的任意图形 -> 单应变换（homography）的结果
  所以：拿一个**平面直边矩形**（A4 纸/书本/手机），
     ① 把四个角映射到理想矩形 -> 定出单应矩阵 H
     ② 把【整条轮廓】用 H 变换回去
     ③ 量它偏离理想矩形边界多远  -> 非零就是畸变/非针孔
"""
import argparse, math, os, sys
import numpy as np, cv2

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
from arm_grasp.collect import capture_frame, k230_host_from_result

RECT = (210.0, 297.0)          # A4，mm


def biggest_quad(gray):
    out = []
    for name, mask in (('亮', cv2.threshold(gray, 0, 255,
                                           cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]),
                       ('暗', cv2.threshold(gray, 0, 255,
                                           cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1])):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = [c for c in cnts if cv2.contourArea(c) > 0.04 * gray.size]
        if not cnts:
            continue
        c = max(cnts, key=cv2.contourArea)
        peri = cv2.arcLength(c, True)
        ap = cv2.approxPolyDP(c, 0.02 * peri, True)
        out.append((name, c, ap))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/k230/manual/pipe.jpg')
    args = ap.parse_args()
    host = k230_host_from_result()
    if not capture_frame(args.out, host):
        print('拍照失败'); return 1
    img = cv2.imread(args.out)
    if img is None:
        print('读图失败'); return 1
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    H_img, W_img = gray.shape
    print('图像 %dx%d' % (W_img, H_img))

    cands = biggest_quad(gray)
    if not cands:
        print('❌ 没找到足够大的连通区域 —— 换个对比更明显的物体（比如深色书本）'); return 1

    for name, c, ap4 in cands:
        print('\n=== 候选：%s 区域，面积 %.1f%%，多边形顶点数 %d ==='
              % (name, 100.0 * cv2.contourArea(c) / gray.size, len(ap4)))
        if len(ap4) != 4:
            print('    顶点不是 4 个 —— 换物体或换个角度'); continue
        # 四个角按角度排序
        pts = ap4.reshape(-1, 2).astype(np.float64)
        ctr = pts.mean(axis=0)
        pts = pts[np.argsort(np.arctan2(pts[:, 1] - ctr[1], pts[:, 0] - ctr[0]))]
        # 边长匹配：长方形长边/短边
        d = [np.linalg.norm(pts[(i+1) % 4] - pts[i]) for i in range(4)]
        if (d[0] + d[2]) > (d[1] + d[3]):
            corners = pts                                  # pts 顺序: 短长短长
            dst = np.array([[0, 0], [RECT[0], 0], [RECT[0], RECT[1]], [0, RECT[1]]], np.float64)
        else:
            corners = np.roll(pts, 1, axis=0)
            dst = np.array([[0, 0], [RECT[0], 0], [RECT[0], RECT[1]], [0, RECT[1]]], np.float64)
        Hm = cv2.getPerspectiveTransform(corners.astype(np.float32), dst.astype(np.float32))

        # ① 每条边的直线度
        print('    ① 四条边的「直线度」（把该边像素拟成直线，量最大偏离）:')
        worst_str = 0.0
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            sel = c.reshape(-1, 2).astype(np.float64)
            d_ab = b - a; n = np.linalg.norm(d_ab)
            t = np.clip(((sel - a) @ d_ab) / (n * n), 0, 1)
            proj = a + t[:, None] * d_ab
            dist = np.linalg.norm(sel - proj, axis=1)
            on = dist < 3.0
            if on.sum() < 10:
                continue
            P = sel[on]
            vx, vy, x0, y0 = cv2.fitLine(P.astype(np.float32), cv2.DIST_L2, 0, 0.01, 0.01).ravel()
            dev = np.abs((P[:, 0] - x0) * vy - (P[:, 1] - y0) * vx)
            print('       边%d: n=%4d  最大偏离 %.3f px   平均 %.3f px'
                  % (i, on.sum(), dev.max(), dev.mean()))
            worst_str = max(worst_str, dev.max())

        # ② 整条轮廓的单应残差
        P = c.reshape(-1, 2).astype(np.float64)
        q = cv2.perspectiveTransform(P.reshape(-1, 1, 2).astype(np.float32), Hm).reshape(-1, 2)
        W, Hh = RECT
        dx = np.minimum(np.abs(q[:, 0]), np.abs(q[:, 0] - W))
        dy = np.minimum(np.abs(q[:, 1]), np.abs(q[:, 1] - Hh))
        d_edge = np.minimum(dx, dy)
        print('    ② 整条轮廓的单应残差: 最大 %.2f mm (%.2f px)  平均 %.2f mm'
              % (d_edge.max(), d_edge.max() * corners[1][0] / RECT[0] / max(1e-9, 1),
                 d_edge.mean()))
        print('       -> 理想针孔应为 0（除噪声）。非零且随位置变大 = 畸变')
        print('    ★ 判据：直线度最大偏离 < 1px 且单应残差 < 0.5mm -> 基本是理想针孔')
    return 0


sys.exit(main())
