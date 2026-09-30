# /home/cy/k230-vision/pi/geom.py —— 目标框/安全区的纯几何
#
# **纯函数，不碰 socket、不碰硬件、不碰 ROS。** Pi 侧和单测都直接用。
#
# ⚠️ 这里的规则必须和**板子**上的 `targets.box_window_fits` 一致 ——
# 板子那边是权威（它会拒绝），Pi 侧这份只是为了在 fire-and-forget 之前
# 自己先检查一遍（那种模式下收不到 `err`）。
#
# 2026-09-19 实测得出的边界（贴边跟到死的那次）：
#   目标框**完整可见**时跟踪一直很准（偏差 3~4px，哪怕模板窗口已经 20% 出画）；
#   框一开始出画就崩（框塌缩、ar_dev 跳、lost）。
#   所以门槛就是「框必须整个在画面内」—— 不是保守，是撞上了实测极限。


def box_complete(box, margin=0.0):
    """目标框是否整个落在画面内（归一化）。margin>0 时要求再往里缩一点。

    box = [l, t, r, b]，归一化到推流画面 —— 与板子 results/objs 同约定。
    """
    l, t, r, b = box
    return (l >= margin and t >= margin
            and r <= 1.0 - margin and b <= 1.0 - margin)


def box_size(box):
    l, t, r, b = box
    return (r - l, b - t)


def safe_center_range(box, margin=0.0):
    """**中心点**的可用范围 (umin, umax, vmin, vmax) —— 也就是「框完整可见」时中心能落在哪。

    报错/夹取用。范围可能是空的（框比画面还大）—— 调用方要判 umin > umax。
    """
    w, h = box_size(box)
    du = w / 2.0 + margin
    dv = h / 2.0 + margin
    return (du, 1.0 - du, dv, 1.0 - dv)


def clamp_center(box, margin=0.0):
    """把框中心夹进安全区，返回夹取后的中心 (cx, cy)。

    **只夹中心、不改框大小** —— 框尺寸是跟踪器的模板，动它会改变模板。
    夹完框可能还是贴边的，所以夹取之后**必须再用 box_complete 复查**。
    """
    l, t, r, b = box
    umin, umax, vmin, vmax = safe_center_range(box, margin)
    cx = (l + r) / 2.0
    cy = (t + b) / 2.0
    if umin > umax:
        cx = 0.5
    else:
        cx = min(max(cx, umin), umax)
    if vmin > vmax:
        cy = 0.5
    else:
        cy = min(max(cy, vmin), vmax)
    return (cx, cy)
