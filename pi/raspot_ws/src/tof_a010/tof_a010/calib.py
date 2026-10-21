# -*- coding: utf-8 -*-
"""标定：**尺度**（mm/计数）与 **ToF↔相机映射**。

纯计算，没有 IO —— 采样靠 `tof_probe --live` 记，这里只负责解。

## 两个样本集，别混

1. **尺度样本** `{(count, true_mm)}`：平面目标放在**尺量已知距离**处。
   解 `mm = a*count + b`。`a` 就是 `unit_mm_per_count`。
   ★ 验收判据：**只有斜率 `a` 与配置差 > 5% 才报红**（把 UNIT 写错 1 ⇒ 斜率错 ~25%，
     立刻红）。**截距 `b` 不判合格** —— 它是"尺子零点贴在哪"，不是误差。理由见 `scale_report`。
2. **映射样本** `{(box_cx, box_cy, tof_cx, tof_cy)}`：同一个目标摆在画面 9 个位置，
   记录「框中心（推流归一化）」与「ToF 块图里目标的落点（ToF 归一化）」。

## 为什么映射必须是仿射

腕上安装 ⇒ ToF 和相机**一起动**，两者之间的相对位姿是**常数** ⇒
图像上的对应关系是一个**固定的仿射**，与手臂姿态无关。
所以标一次就够，不存在"每拍重算"的问题。

⚠️ **残余视差**：两者不共轴，基线约 2~3cm，40cm 处约 5~7% 视场（几个 ToF 像素），
方向随目标偏离光轴而变化 —— 仿射吸收不掉这一项。好在目标尺寸远大于它，
且取的是**区内最小**，所以容忍。若标定后落点仍稳定偏在背景上，
先怀疑视差，而不是噪声。
"""
from __future__ import annotations

import math


# --------------------------------------------------------------------------
# 一元/二元最小二乘（手写，省一个 numpy 依赖，且这些维度不值得上矩阵库）
# --------------------------------------------------------------------------

def fit_line(xs, ys):
    """`y = a*x + b` 最小二乘。返回 `(a, b, rms)`。"""
    n = len(xs)
    if n < 2:
        raise ValueError("至少要 2 个点")
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if abs(sxx) < 1e-12:
        raise ValueError("所有 x 都一样，解不出来（目标没动过？）")
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx
    b = my - a * mx
    rms = math.sqrt(sum((y - (a * x + b)) ** 2 for x, y in zip(xs, ys)) / n)
    return a, b, rms


def fit_plane(xs, ys, zs):
    """`z = a*x + b*y + c` 最小二乘（3x3 正规方程）。返回 `(a, b, c, rms)`。"""
    n = len(xs)
    if n < 3:
        raise ValueError("至少要 3 个点")
    sx = sum(xs); sy = sum(ys); sz = sum(zs)
    sxx = sum(x * x for x in xs); syy = sum(y * y for y in ys)
    sxy = sum(x * y for x, y in zip(xs, ys))
    sxz = sum(x * z for x, z in zip(xs, zs))
    syz = sum(y * z for y, z in zip(ys, zs))
    m = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, float(n)]]
    rhs = [sxz, syz, sz]
    a, b, c = _solve3(m, rhs)
    rms = math.sqrt(sum((z - (a * x + b * y + c)) ** 2
                        for x, y, z in zip(xs, ys, zs)) / n)
    return a, b, c, rms


def _solve3(m, rhs):
    """3x3 高斯消元（带部分主元）。"""
    import copy
    a = copy.deepcopy(m)
    b = list(rhs)
    for col in range(3):
        piv = max(range(col, 3), key=lambda r: abs(a[r][col]))
        if abs(a[piv][col]) < 1e-12:
            raise ValueError("样本退化（点共线/只动了一个方向），解不出来")
        a[col], a[piv] = a[piv], a[col]
        b[col], b[piv] = b[piv], b[col]
        for r in range(col + 1, 3):
            f = a[r][col] / a[col][col]
            for c2 in range(col, 3):
                a[r][c2] -= f * a[col][c2]
            b[r] -= f * b[col]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        s = b[r] - sum(a[r][c2] * x[c2] for c2 in range(r + 1, 3))
        x[r] = s / a[r][r]
    return x


# --------------------------------------------------------------------------
# 映射
# --------------------------------------------------------------------------

def fit_map_4(samples):
    """4 参：`x' = ax*x + bx`，`y' = ay*y + by`。x/y 各自独立，两个一元回归。"""
    bx = [s["box_cx"] for s in samples]
    by = [s["box_cy"] for s in samples]
    tx = [s["tof_cx"] for s in samples]
    ty = [s["tof_cy"] for s in samples]
    ax, bxc, rx = fit_line(bx, tx)
    ay, byc, ry = fit_line(by, ty)
    return {"ax": ax, "bx": bxc, "ay": ay, "by": byc}, max(rx, ry)


def fit_map_6(samples):
    """6 参：多出旋转/错切。两块板子装歪了几度时用它。"""
    bx = [s["box_cx"] for s in samples]
    by = [s["box_cy"] for s in samples]
    tx = [s["tof_cx"] for s in samples]
    ty = [s["tof_cy"] for s in samples]
    a11, a12, a13, rx = fit_plane(bx, by, tx)
    a21, a22, a23, ry = fit_plane(bx, by, ty)
    return ({"a11": a11, "a12": a12, "a13": a13,
             "a21": a21, "a22": a22, "a23": a23}, max(rx, ry))


def residuals(map_cfg, samples):
    """逐个样本的像素残差（ToF 归一化 → 乘 100 就是 ToF 像素）。"""
    from . import a010
    out = []
    for s in samples:
        px, py = a010.map_point(map_cfg, s["box_cx"], s["box_cy"])
        out.append((math.hypot(px - s["tof_cx"], py - s["tof_cy"]), px, py))
    return out


def map_report(samples, accept_px=10.0):
    """打两份拟合的结果与残差，并给出**建议写进 config 的那一段**。

    判据：**留出集**（这里用全体，样本少）最大残差 < `accept_px` ToF 像素
    （100 像素 = 整个视场，10 像素 ≈ 视场的 10%，够抓取）。
    """
    lines = ["样本 %d 个" % len(samples)]
    best = None
    for name, fn in (("4 参", fit_map_4), ("6 参", fit_map_6)):
        try:
            cfg, rms = fn(samples)
        except ValueError as e:
            lines.append("  %s 拟合失败：%s" % (name, e))
            continue
        rs = [r for r, _px, _py in residuals(cfg, samples)]
        worst = max(rs) * 100.0
        lines.append("  %-4s  rms=%.1f ToF像素   最差=%.1f ToF像素"
                     % (name, rms * 100, worst))
        if best is None or worst < best[2]:
            best = (name, cfg, worst)
    if best is None:
        return "\n".join(lines) + "\n  ✗ 两种都拟合失败"

    name, cfg, worst = best
    verdict = "✅ 过" if worst < accept_px else "❌ 不过（目标 %s 个 ToF 像素内）" % accept_px
    lines.append("  ⇒ 建议用 **%s**：%s   最差 %.1f ToF像素" % (name, verdict, worst))
    lines.append("")
    lines.append("  # 写进 config/tof.yaml：")
    lines.append("  map:")
    for k in sorted(cfg):
        lines.append("    %s: %.6f" % (k, cfg[k]))
    lines.append("  # （4 参时上面会是 ax/bx/ay/by；6 参是 a11..a23）")
    return "\n".join(lines)


# --------------------------------------------------------------------------
# 尺度
# --------------------------------------------------------------------------

def scale_report(samples, unit_cfg, accept_rel=0.05):
    """`samples = [{"count": c, "true_mm": m}, ...]` → 文本。

    ## 判据只有**斜率**，截距不进判据

    ★ 这条**必须能红**：把 `unit_mm_per_count` 改错 1，斜率会跟着错 ~25%，
      立刻超 5% 的判据。标定脚本如果永远绿，它就没有存在意义。

    ⚠️ **截距不判合格**，因为它的物理含义是"尺子零点贴在哪"：
      尺子贴在外壳前脸、而传感器缩在里面 ~21mm 时，截距就是 +21mm ——
      **这是正常的，不是误差**（2026-10-14 实测：rms 1.5mm、斜率 3.951、
      截距 21.3mm，正是外壳到镜片的那段壳）。
      拿它去判红，只会逼人去补一个**本来就不该补**的偏置，
      而那 21mm 会直接搬进抓取落点。
    """
    xs = [float(s["count"]) for s in samples]
    ys = [float(s["true_mm"]) for s in samples]
    a, b, rms = fit_line(xs, ys)
    rel = abs(a - unit_cfg) / unit_cfg
    worst = max(abs(y - (a * x + b)) for x, y in zip(xs, ys))
    ok = rel <= accept_rel
    return "\n".join([
        "尺度样本 %d 个" % len(samples),
        "  拟合  mm = %.4f * count + %.2f    rms=%.2fmm  最差=%.2fmm" % (a, b, rms, worst),
        "  配置  unit_mm_per_count = %.4f" % unit_cfg,
        "  斜率相对差 %.1f%%（**判据 <= %.0f%%**，只有它算合格与否）"
        % (rel * 100, accept_rel * 100),
        "  ⇒ %s" % ("✅ 过（刻度没问题）" if ok
                    else "❌ 不过 —— 刻度错了，去改 config 的 unit_mm_per_count"),
        "",
        "  截距 %+.2f mm —— **不进判据**，它是「尺子零点贴在哪」的体现：" % b,
        "    · 尺子贴在**外壳前脸**、传感器缩在里面 ⇒ 截距就是那段壳（约 20mm 量级），正常",
        "    · 只有当尺子**确实量到传感器平面**、截距还明显非零，才是模块的固定偏置",
        "    · 两者的影响差一个抓取落点，**不要凭猜去补**（有 A/B 再定）",
        "",
        "  # 写进 config/tof.yaml（斜率一般不用改，4.0 就是这个模块的标称值）：",
        "  unit_mm_per_count: %.4f" % a,
    ])
