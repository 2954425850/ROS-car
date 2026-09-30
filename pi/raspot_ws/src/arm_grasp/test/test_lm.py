import math

import pytest

from arm_grasp.lm import least_squares, _solve


def test_least_squares_finds_the_minimum_of_a_linear_system():
    """独立判据：线性残差的极小点就是解本身，不用管 LM 内部怎么走。"""
    def residual(x):
        return [x[0] - 3.0, x[1] + 1.0, 2.0 * x[2] - 5.0]

    x, cost, iters = least_squares(residual, [0.0, 0.0, 0.0])
    assert math.dist(x, (3.0, -1.0, 2.5)) < 1e-6, x
    assert cost < 1e-12, cost


def test_least_squares_handles_a_nonlinear_problem():
    """非线性也要收敛（这是实际用法）。"""
    def residual(x):
        return [math.sin(x[0]) - 0.5, x[1] ** 2 - 4.0]

    x, cost, iters = least_squares(residual, [0.2, 1.5])
    assert abs(x[0] - math.asin(0.5)) < 1e-6, x
    assert abs(x[1] - 2.0) < 1e-6, x
    assert cost < 1e-12, cost


def test_least_squares_is_deterministic():
    def residual(x):
        return [math.sin(x[0]) - 0.5, x[1] ** 3 - 8.0]

    a = least_squares(residual, [0.2, 1.0])
    b = least_squares(residual, [0.2, 1.0])
    assert a == b


def test_solve_raises_on_a_singular_matrix():
    """★ 手写消元必须把奇异当场喊出来。

    不喊的后果是**静默 0 除 -> NaN -> 拟合悄悄停在初值**，
    而报告里的残差还是"能算出来"的（只是很大），很容易被当成数据不好。
    这条是把 `_solve` 里那个主元阈值钉住 —— 变异自检发现
    把它改坏成恒不触发时没有任何用例会红（数据太好，永远碰不到奇异）。
    """
    with pytest.raises(ZeroDivisionError):
        _solve([[0.0, 0.0], [0.0, 0.0]], [1.0, 2.0])
    with pytest.raises(ZeroDivisionError):
        _solve([[1.0, 2.0], [2.0, 4.0]], [1.0, 2.0])   # 秩 1


def test_solve_actually_solves_a_system():
    assert math.dist(_solve([[2.0, 1.0], [1.0, 3.0]], [5.0, 10.0]),
                     (1.0, 3.0)) < 1e-12


def test_solve_rejects_a_near_singular_matrix():
    """近奇异必须当成奇异处理，不能"除出一个巨大的数"糊过去。

    ★ 2026-09-28 变异自检发现：只用全零矩阵测不出主元阈值的死活
      （`0/0` 本来就会抛 ZeroDivisionError，把阈值改坏也照样"通过"）。
      1e-20 的主元除下去是 1e20 量级的解 —— 数值上毫无意义，
      但**不会**抛异常，于是拟合会悄悄停在一个垃圾点上、报告里
      残差还"算得出来"。这条才真正钉住了那个阈值。
    """
    with pytest.raises(ZeroDivisionError):
        _solve([[1e-20, 0.0], [0.0, 1.0]], [1.0, 1.0])
    with pytest.raises(ZeroDivisionError):
        _solve([[1.0, 0.0], [1.0, 1e-18]], [1.0, 2.0])
