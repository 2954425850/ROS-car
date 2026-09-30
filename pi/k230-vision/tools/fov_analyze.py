#!/usr/bin/env python3
"""Task 6 附加验证（Pi 侧）：chn0(720p YUV420SP) 与 chn2(AI RGBP888) 是否**同一视场**。

输入（板子原样 dump，板上未做任何转换）:
    raw_chn0.bin   1280x720 YUV420SP
    raw_chn2.bin   320x320  RGBP888 planar R|G|B

输出:
    fov3_chn0.png            chn0 解码图
    fov3_chn0_nv21.png       备用解码（若 NV12 判错时可目视比对）
    fov3_chn2.png            chn2 解码图
    fov3_side_by_side.png    两图并排 + 各自算出的目标归一化框
    fov3_overlay.png         把 chn2 的框按归一化坐标画到 chn0 图上（对齐检验）

判定（两条独立证据）:
  A. 结构相关性：把 chn2 **各向异性拉伸到 1280x720** 后与 chn0 灰度求归一化相关，
     并在 ±128 像素（1280x720 坐标系）内做位移搜索。
     同视场 ⇒ 相关高、最佳位移 ≈ (0,0)；居中裁剪 ⇒ 相关显著低 / 位移不为 0。
     负对照：故意按"垂直放大 1.78 倍"错配一次，看相关系数是否明显掉下去
     （证明这个指标本身有区分力）。
  B. 目标归一化 bbox 对比：两图各自找"最红目标"（或最强竖直边缘），
     比较归一化位置与比例。

用法: python3 fov_analyze.py
"""
import os
import sys
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
F0 = os.path.join(ROOT, "raw_chn0.bin")
F2 = os.path.join(ROOT, "raw_chn2.bin")

W0, H0 = 1280, 720
W2, H2 = 320, 320


def decode_nv(buf, w, h, nv12=True):
    """YUV420SP (NV12/NV21) -> BGR。处理可能的行 stride 补齐。"""
    total = len(buf)
    stride = None
    if total == w * h * 3 // 2:
        stride = w
    else:
        s = total / (h * 1.5)
        if abs(s - round(s)) < 1e-6:
            stride = int(round(s))
    if stride is None:
        raise ValueError("无法从长度 %d 推出 %dx%d YUV420SP 的 stride" % (total, w, h))
    y = np.frombuffer(buf, np.uint8, count=h * stride).reshape(h, stride)[:, :w]
    uv = np.frombuffer(buf, np.uint8, count=(h // 2) * stride,
                       offset=h * stride).reshape(h // 2, stride)[:, :w]
    nv = np.vstack([y, uv]).copy()
    code = cv2.COLOR_YUV2BGR_NV12 if nv12 else cv2.COLOR_YUV2BGR_NV21
    return cv2.cvtColor(nv, code), stride


def largest_red_bbox(bgr):
    """返回 (norm_bbox=[l,t,r,b], coverage, mask)。找不到返回 (None, 0, mask)。"""
    b = bgr[:, :, 0].astype(np.int16)
    g = bgr[:, :, 1].astype(np.int16)
    r = bgr[:, :, 2].astype(np.int16)
    red = r - np.maximum(g, b)
    m = (red > 30).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None, 0.0, m
    i = 1 + int(np.argmax(stats[1:, 4]))
    x, y, w, h, area = stats[i]
    H, W = bgr.shape[:2]
    return ([x / W, y / H, (x + w) / W, (y + h) / H], area / float(W * H),
            (lab == i).astype(np.uint8))


def gray_of(bgr):
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)


def corr(a, b):
    a = a.astype(np.float32)
    b = b.astype(np.float32)
    a = (a - a.mean()) / (a.std() + 1e-6)
    b = (b - b.mean()) / (b.std() + 1e-6)
    return float((a * b).mean())


def best_shift(g0, g2, pad_px):
    """在小图上用 matchTemplate 做位移搜索。返回 (dx, dy, corr)，单位 = 小图像素。"""
    ta = cv2.resize(g0, (320, 180), interpolation=cv2.INTER_AREA)
    tb = cv2.resize(g2, (320, 180), interpolation=cv2.INTER_AREA)
    big = cv2.copyMakeBorder(ta, pad_px, pad_px, pad_px, pad_px, cv2.BORDER_REPLICATE)
    res = cv2.matchTemplate(big.astype(np.float32), tb.astype(np.float32),
                            cv2.TM_CCOEFF_NORMED)
    _, mx, _, loc = cv2.minMaxLoc(res)
    return loc[0] - pad_px, loc[1] - pad_px, float(mx)


def strongest_vertical_edge(gray):
    """最强竖直边缘的归一化 x（版式参照物用）。"""
    gx = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)).sum(axis=0)
    gx = cv2.GaussianBlur(gx.reshape(1, -1), (1, 31), 0).ravel()
    return float(np.argmax(gx)) / gray.shape[1]


def main():
    b0 = open(F0, "rb").read()
    b2 = open(F2, "rb").read()
    print("RAW chn0 bytes=%d  chn2 bytes=%d" % (len(b0), len(b2)))

    img0, stride = decode_nv(b0, W0, H0, nv12=True)
    img0n21, _ = decode_nv(b0, W0, H0, nv12=False)
    print("chn0 stride=%d (w=%d)  期望 YUV420SP 1280x720 = %d B"
          % (stride, W0, W0 * H0 * 3 // 2))

    n2 = W2 * H2
    if len(b2) != n2 * 3:
        raise ValueError("raw_chn2.bin 长度 %d 不是 320x320x3" % len(b2))
    arr = np.frombuffer(b2, np.uint8)
    # Task 5 实测：RGBP888 是 planar R|G|B
    img2 = np.dstack([arr[2 * n2:3 * n2].reshape(H2, W2),
                      arr[n2:2 * n2].reshape(H2, W2),
                      arr[0:n2].reshape(H2, W2)]).copy()

    cv2.imwrite(os.path.join(ROOT, "fov3_chn0.png"), img0)
    cv2.imwrite(os.path.join(ROOT, "fov3_chn0_nv21.png"), img0n21)
    cv2.imwrite(os.path.join(ROOT, "fov3_chn2.png"), img2)

    # ---- 证据 A：结构相关性 ----
    g0 = gray_of(img0)
    img2_stretch = cv2.resize(img2, (W0, H0), interpolation=cv2.INTER_LINEAR)
    g2s = gray_of(img2_stretch)
    print("EVID_A_CORR_ZERO_SHIFT %.4f" % corr(g0, g2s))
    # 小图 320x180 上 pad=32 ⇒ 等效 1280x720 上 ±128 px
    dx, dy, mx = best_shift(g0, g2s, 32)
    print("EVID_A_BEST_SHIFT dx=%d dy=%d corr=%.4f  (小图320x180; ±128px @720p)"
          % (dx, dy, mx))
    # 负对照：把 chn2 中央 1/1.78 的行当成全幅（即"垂直放大 1.78 倍"的错配）
    c0 = int(H2 * (1 - 1 / 1.78) / 2)
    c1 = H2 - c0
    gctrl = gray_of(cv2.resize(img2[c0:c1, :], (W0, H0)))
    print("EVID_A_CONTROL_WRONG_ZOOM_CORR %.4f" % corr(g0, gctrl))

    # ---- 证据 B：目标归一化 bbox ----
    bbox0, cov0, _ = largest_red_bbox(img0)
    bbox2, cov2, _ = largest_red_bbox(img2)
    print("EVID_B_CHN0_RED_BBOX %s coverage=%.4f" % (bbox0, cov0))
    print("EVID_B_CHN2_RED_BBOX %s coverage=%.4f" % (bbox2, cov2))
    if bbox0 and bbox2:
        print("EVID_B_DELTA (chn2 - chn0) [%.4f %.4f %.4f %.4f]"
              % tuple(bbox2[i] - bbox0[i] for i in range(4)))

    e0 = strongest_vertical_edge(g0)
    e2 = strongest_vertical_edge(gray_of(img2))
    print("EVID_B_VERT_EDGE_NORM_X chn0=%.4f chn2=%.4f delta=%+.4f"
          % (e0, e2, e2 - e0))

    # ---- 出图 ----
    def draw(img, bbox, color, label):
        out = img.copy()
        H, W = out.shape[:2]
        if bbox:
            x1, y1, x2, y2 = int(bbox[0] * W), int(bbox[1] * H), \
                             int(bbox[2] * W), int(bbox[3] * H)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 3)
            cv2.putText(out, label, (max(0, x1), max(22, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        return out

    a = draw(img0, bbox0, (0, 255, 0), "chn0 own")
    b = draw(cv2.resize(img2, (W0, H0)), bbox2, (0, 255, 255), "chn2 own (stretched)")
    cv2.imwrite(os.path.join(ROOT, "fov3_side_by_side.png"), np.vstack([a, b]))

    ov = draw(img0, bbox2, (0, 0, 255), "chn2 box -> chn0 coords")
    if bbox0:
        H, W = ov.shape[:2]
        cv2.rectangle(ov, (int(bbox0[0] * W), int(bbox0[1] * H)),
                      (int(bbox0[2] * W), int(bbox0[3] * H)), (0, 255, 0), 2)
    cv2.imwrite(os.path.join(ROOT, "fov3_overlay.png"), ov)
    print("PNG_WRITTEN fov3_chn0.png fov3_chn0_nv21.png fov3_chn2.png "
          "fov3_side_by_side.png fov3_overlay.png")


if __name__ == "__main__":
    sys.exit(main())
