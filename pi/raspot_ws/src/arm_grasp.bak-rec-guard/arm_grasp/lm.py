# -*- coding: utf-8 -*-
"""极简 Levenberg-Marquardt（数值雅可比）。避免在 Pi 上装 scipy。"""


def least_squares(residual, x0, max_iter=300, tol=1e-12, lam0=1e-3):
    """返回 (x, cost, iters)。residual(x) -> list[float]。"""
    n = len(x0)
    x = list(x0)

    def cost_at(v):
        r = residual(v)
        return sum(c * c for c in r)

    fx = residual(x)
    c = cost_at(x)
    lam = lam0
    iters = 0
    for iters in range(1, max_iter + 1):
        # 数值雅可比 J[i][j] = d r_i / d x_j
        J = []
        for j in range(n):
            h = 1e-6 * max(1.0, abs(x[j]))
            xp = list(x)
            xp[j] += h
            rp = residual(xp)
            J.append([(rp[i] - fx[i]) / h for i in range(len(fx))])
        # 正规方程 (J^T J + lam I) dx = -J^T f
        m = len(fx)
        JTJ = [[sum(J[a][i] * J[b][i] for i in range(m)) for b in range(n)]
               for a in range(n)]
        JTf = [sum(J[a][i] * fx[i] for i in range(m)) for a in range(n)]
        for a in range(n):
            JTJ[a][a] += lam * (1.0 + JTJ[a][a])
        try:
            dx = _solve(JTJ, [-v for v in JTf])
        except ZeroDivisionError:
            lam *= 10.0
            continue
        xn = [x[i] + dx[i] for i in range(n)]
        cn = cost_at(xn)
        if cn < c:
            if abs(c - cn) < tol * max(1.0, c):
                x, c, fx = xn, cn, residual(xn)
                break
            x, c, fx = xn, cn, residual(xn)
            lam = max(lam * 0.3, 1e-12)
        else:
            lam *= 10.0
            if lam > 1e12:
                break
    return x, c, iters


def _solve(A, b):
    """高斯消元（带部分主元）。A 是 n x n 的 list of list。"""
    n = len(A)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(M[r][col]))
        if abs(M[piv][col]) < 1e-14:
            raise ZeroDivisionError('singular')
        M[col], M[piv] = M[piv], M[col]
        pv = M[col][col]
        for r in range(col + 1, n):
            f = M[r][col] / pv
            for c2 in range(col, n + 1):
                M[r][c2] -= f * M[col][c2]
    out = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = M[r][n] - sum(M[r][c2] * out[c2] for c2 in range(r + 1, n))
        out[r] = s / M[r][r]
    return out
