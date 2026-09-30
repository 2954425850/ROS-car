# -*- coding: utf-8 -*-
"""把 `geom_probe --mode p4 --out` 攒下来的多条样本合起来看 —— **不需要尺子**。

    python3 tools/p4_spread.py docs/p4.json
    python3 tools/p4_spread.py --selftest          # 不需要硬件

模型：每条样本给 (光心 C_i, 深度 t_i)，管线算出的点 P_i = C_i + t_i·u_i
（u_i 由 P_i 与 C_i 反出，与管线自身一致）。瓶盖没动 ⇒ 真值只有一个点 P*。

未知量取 **(P*, δ)**，关系是线性的：
    P_i + δ·u_i = P*          δ = **真深度 − 报告深度**（报告偏大 ⇒ δ 为负）

* **δ** ≈ 0 ⇒ 深度对。δ = −2cm ⇒ 报告的深度整体偏大 2cm。
* **残差散布**：扣掉 δ 之后算出的点离 P* 还有多远 —— "链子一致不一致"的判据。

★★ **成败取决于姿态覆盖**：若各条视线**几乎平行**，δ 与 P* 分不开，
本脚本会算出「方向张角」并在太小时**明确警告这个数据集测不出深度偏差**。
摆姿态时**远近、俯仰都要拉开**，否则就是在做"运气测试"。
"""
import argparse
import json
import math
import random
import sys

sys.path.insert(0, '/home/cy/raspot_ws/src/arm_grasp')
from arm_grasp.lm import _solve


def analyse(recs):
    N = len(recs)
    R, U = [], []
    for r in recs:
        c, p, d = r['C'], r['target'], r['D_m']
        v = [p[k] - c[k] for k in range(3)]
        n = math.sqrt(sum(x * x for x in v))
        if n < 1e-9:
            raise ValueError('某条样本的 target 与 C 重合')
        u = [x / n for x in v]
        U.append(u)
        R.append([c[k] + d * u[k] for k in range(3)])
    su = [sum(u[k] for u in U) for k in range(3)]
    A = [[0.0] * 4 for _ in range(4)]
    b = [0.0] * 4
    for k in range(3):
        A[k][k] = float(N)
        A[k][3] = -su[k]
        A[3][k] = -su[k]
    A[3][3] = float(N)
    for i in range(N):
        for k in range(3):
            b[k] += R[i][k]
        b[3] += -sum(U[i][k] * R[i][k] for k in range(3))
    x = _solve([row[:] for row in A], b)
    Pst, delta = x[:3], x[3]
    res = []
    for i in range(N):
        e = [R[i][k] - Pst[k] + delta * U[i][k] for k in range(3)]
        res.append(math.sqrt(sum(t * t for t in e)))
    mean = [sum(p[k] for p in R) / N for k in range(3)]
    raw = [math.dist(p, mean) for p in R]
    mu = [sum(u[k] for u in U) / N for k in range(3)]
    nm = math.sqrt(sum(t * t for t in mu)) or 1.0
    mu = [t / nm for t in mu]
    angs = [math.degrees(math.acos(max(-1, min(1, sum(U[i][k] * mu[k] for k in range(3))))))
            for i in range(N)]
    # ★ 张角用**最大两两夹角**。原来用「偏离均值的最大值」，它只是跨度的一半，
    #   会把本来够开的数据误报成"测不出"（2026-09-29 实际踩到：跨度约 22 度被报成 11.2 度）。
    pair = []
    for i in range(N):
        for j in range(i + 1, N):
            pair.append(math.degrees(math.acos(max(-1, min(1, sum(
                U[i][k] * U[j][k] for k in range(3)))))))
    return {'N': N, 'delta': delta, 'P': Pst,
            'res_rms': math.sqrt(sum(e * e for e in res) / N), 'res_max': max(res),
            'raw_rms': math.sqrt(sum(e * e for e in raw) / N), 'raw_max': max(raw),
            'ang_spread': max(pair) if pair else 0.0,
            'ang_dev_max': max(angs), 'angs': angs, 'depths': [r['D_m'] for r in recs]}


def _mk(C, P_true, bias, rnd=None, dir_sigma=0.0):
    """bias = **报告深度相对真值的偏差**（report = true + bias）。
    rnd/dir_sigma：给**视线方向**加抖动（模拟像素噪声 → 方向噪声）。"""
    v = [P_true[k] - C[k] for k in range(3)]
    t_true = math.sqrt(sum(x * x for x in v))
    u = [x / t_true for x in v]
    if rnd is not None and dir_sigma > 0.0:
        u = [u[k] + rnd.gauss(0.0, dir_sigma) for k in range(3)]
        n = math.sqrt(sum(x * x for x in u))
        u = [x / n for x in u]
    t_rep = t_true + bias
    P = [C[k] + t_rep * u[k] for k in range(3)]
    return {'target': P, 'C': list(C), 'D_m': t_rep, 'Z_axial_m': t_rep,
            'mode': 'p4', 'alpha': 0.0}


def _selftest():
    """★ 关键是**在有噪声的条件下**比较两种姿态覆盖。

    无噪声时 δ 精确可辨（跟张角无关）；真实数据一定有方向噪声
    （P3 量到的方向精度 ≈ 0.1° ≈ 12cm 处 2mm），那时**张角决定 δ 还能不能分开**。
    """
    P_true = (0.05, -0.22, 0.016)
    SIG = 0.002                 # 方向噪声 0.002 rad ≈ 0.11° ≈ 12cm 处 2mm
    BIAS = 0.02                 # 报告深度偏大 2cm

    def run(Cs, seed):
        rnd = random.Random(seed)
        return analyse([_mk(C, P_true, BIAS, rnd, SIG) for C in Cs])

    rnd0 = random.Random(3)
    wide = [(0.06 + rnd0.uniform(-.02, .02), rnd0.uniform(-.03, .03),
             0.07 + rnd0.uniform(0, .10)) for _ in range(3)]
    wide += [(-0.06, 0.10, 0.12), (0.20, -0.30, 0.25), (0.05, 0.18, 0.05)]
    narrow = [(0.09 + rnd0.uniform(-.005, .005), rnd0.uniform(-.005, .005), 0.10)
              for _ in range(6)]

    errsW, errsN = [], []
    for seed in range(8):
        for Cs, errs in ((wide, errsW), (narrow, errsN)):
            a = run(Cs, seed)
            errs.append(abs(a['delta'] + BIAS))      # δ 的复原误差（米）
    mW = sum(errsW) / len(errsW) * 1000
    mN = sum(errsN) / len(errsN) * 1000
    ang = run(wide, 0)['ang_spread']
    angN = run(narrow, 0)['ang_spread']
    print('  视线张角 %5.1f° 时：δ 复原误差 %.2f mm   (报告深度真偏 20mm)'
          % (ang, mW))
    print('  视线张角 %5.1f° 时：δ 复原误差 %.2f mm  <-- 几乎量不出来'
          % (angN, mN))
    assert mW < 5.0, mW                       # 张得开 -> δ 量得准
    assert mN > 3.0 * mW, (mW, mN)            # 平行 -> 明显更差（这就是"测不出"）
    a0 = run(wide, 0)
    print('  张角 %.1f° 那组：残差 rms %.2f mm（噪声级）' % (ang, a0['res_rms'] * 1000))
    assert a0['res_rms'] * 1000 < 5.0, a0
    print('自检通过：张得开 δ 准（%.2fmm）；几乎平行时差 %.0f 倍 ⇒ 「再测两次」若姿态不拉开就是白测。'
          % (mW, mN / mW))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('path', nargs='?')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('--min-angle-deg', type=float, default=15.0)
    ap.add_argument('--res-tol-mm', type=float, default=10.0)
    ap.add_argument('--delta-tol-mm', type=float, default=10.0)
    ap.add_argument('--drop', default='',
                    help='排掉已知坏的样本序号（1 起，逗号分隔），如 5')
    args = ap.parse_args(argv)
    if args.selftest:
        _selftest()
        return 0
    if not args.path:
        ap.error('给一个 p4 的 json（或 --selftest）')
    raw = json.load(open(args.path, encoding='utf-8'))
    recs = [r for r in (raw if isinstance(raw, list) else [raw])
            if r.get('mode') == 'p4' and 'target' in r and 'C' in r]
    all_n = len(recs)
    if args.drop:
        drop = {int(x) - 1 for x in args.drop.replace(',', ' ').split()}
        recs = [r for i, r in enumerate(recs) if i not in drop]
        print('（按 --drop %s 排掉了 %d 条，剩 %d 条）' % (args.drop, all_n - len(recs), len(recs)))
    if len(recs) < 3:
        print('只有 %d 条可用的 p4 样本 —— 至少 3 条（建议 6 条）才有意义。' % len(recs))
        return 2
    a = analyse(recs)
    print('样本 %d 条' % a['N'])
    for r, d, ang in zip(recs, a['depths'], a['angs']):
        print('   (%7.2f,%7.2f,%7.2f) cm  深度 %5.2f cm  α %6.1f°  视线偏离均值 %4.1f°'
              % (r['target'][0] * 100, r['target'][1] * 100, r['target'][2] * 100,
                 d * 100, r['alpha'], ang))
    print()
    print('最佳公共点 P* = (%.2f, %.2f, %.2f) cm' % tuple(x * 100 for x in a['P']))
    print()
    if a['ang_spread'] < args.min_angle_deg:
        print('⚠️  **方向张角只有 %.1f°**（< %.0f°）—— 视线几乎平行，δ 与 P* 分不开。'
              % (a['ang_spread'], args.min_angle_deg))
        print('    这个数据集**测不出深度偏差**；把远近/俯仰拉开再采。')
    print('★ δ（真深度 − 报告深度）= %+.2f cm   ->  %s'
          % (a['delta'] * 100,
             '✅ 深度对' if abs(a['delta']) * 1000 < args.delta_tol_mm
             else '❌ 深度系统性偏（报告值偏%+.1f cm）' % (a['delta'] * 100)))
    print('   （最大两两夹角 %.1f°，偏离均值最大 %.1f°）'
          % (a['ang_spread'], a['ang_dev_max']))
    print('★ 残差散布（扣掉 δ 后）rms %.2f mm / 最大 %.2f mm   ->  %s'
          % (a['res_rms'] * 1000, a['res_max'] * 1000,
             '✅ 是同一个点，链子一致' if a['res_rms'] * 1000 < args.res_tol_mm
             else '❌ 散开了，链子有系统性问题'))
    print('   （不扣 δ 的原始散布 rms %.2f mm，供对照）' % (a['raw_rms'] * 1000))
    return 0


if __name__ == '__main__':
    sys.exit(main())
