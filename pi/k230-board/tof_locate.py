# /sdcard/k230vision/tof_locate.py —— 框 → ToF 区域 → 最小有效深度（**纯逻辑**）
#
# 和 a010_frame.py 一个路子：把不碰硬件的东西拎出来，好在板上零风险测红。
#
# ## 为什么取「最小」而不是中位数/中心像素
#
# 2026-10-10 实测（41~42cm，UNIT=4，25 帧）：
#   * 中心区**最小** 420.6mm / 1σ 2.9mm；**中位数** 440.6mm（差 20mm）；
#     **中心单像素** 448.3mm（差 28mm）。尺量真值 41~42cm ⇒ **最小最接近真值**。
#
# ## 为什么要 150mm 量程下限
#
# ToF 视野里能看到爪子/臂身，产生**近场假回波**（实测 12 计数 = 48mm、
# 30 计数 = 120mm）—— 裸取最小会被它们拉到场强。模块**额定下限就是 15cm**，
# 比它还近的回波按定义不可信 ⇒ 直接滤掉。
#
# ## 为什么不做一维聚类（做过，2026-10-12 废弃）
#
# 40cm 处 100x100、UNIT=4 时，瓶盖顶面与桌面只差 **3 计数**，而**同一个面**
# 因倾斜/纵深就有 ~8 计数散度 ⇒ 任何 jump 不是把盖子与桌面合掉、就是把面切碎。
# 实测：聚类版对「目标 + 桌面」的用例给出 440mm（中位数口径），错 20mm。

VAL_MAX = 254          # ★ 255 = 无效/超量程（模块的约定），必须滤掉，别拿来当「距离」
MIN_MM = 150           # ★ 模块额定量程下限 15cm；比这更近的回波是伪影（爪尖/臂身/多路径）


def _region_bounds(box_norm, rows, cols, map_cfg, pad):
    l, t, r, b = box_norm
    if r <= l or b <= t:
        return None
    ax, bx = map_cfg.get("ax", 1.0), map_cfg.get("bx", 0.0)
    ay, by = map_cfg.get("ay", 1.0), map_cfg.get("by", 0.0)
    x0, x1 = ax * l + bx, ax * r + bx
    y0, y1 = ay * t + by, ay * b + by
    if x0 > x1:
        x0, x1 = x1, x0
    if y0 > y1:
        y0, y1 = y1, y0
    c0 = int((x0 - pad) * cols)
    c1 = int((x1 + pad) * cols + 0.999)
    r0 = int((y0 - pad) * rows)
    r1 = int((y1 + pad) * rows + 0.999)
    if c1 <= 0 or r1 <= 0 or c0 >= cols or r0 >= rows:
        return None                      # 完全出界
    r0 = max(0, min(rows, r0))
    r1 = max(0, min(rows, r1))
    c0 = max(0, min(cols, c0))
    c1 = max(0, min(cols, c1))
    if r1 <= r0 or c1 <= c0:
        return None
    return (r0, r1, c0, c1)


def box_to_region(box_norm, rows, cols, map_cfg=None, pad=0.06):
    """归一化框（推流坐标）→ ToF 像素区域 (r0, r1, c0, c1)；完全出界返回 None。"""
    return _region_bounds(box_norm, rows, cols, map_cfg or {}, pad)


def region_depth(frame, region, unit_mm, min_mm=MIN_MM, val_max=VAL_MAX):
    """区域内**最小**有效计数 → 毫米；没有合格值返回 None。"""
    rows, cols, pl = frame["rows"], frame["cols"], frame["payload"]
    r0, r1, c0, c1 = region
    if rows == 0 or cols == 0 or len(pl) < rows * cols:
        return None
    lo = max(1, int(min_mm / unit_mm + 0.999))    # 向上取整：只接受 ≥ min_mm 的值
    best = None
    for r in range(r0, r1):
        base = r * cols
        for c in range(c0, c1):
            v = pl[base + c]
            if lo <= v <= val_max and (best is None or v < best):
                best = v
    return None if best is None else best * unit_mm


def locate_depth(frame, box_norm, unit_mm, map_cfg=None, pad=0.06,
                 min_mm=MIN_MM, val_max=VAL_MAX):
    """app.py 要调的那一个：框 → 深度（毫米整数）或 None。"""
    if frame is None:
        return None
    region = box_to_region(box_norm, frame["rows"], frame["cols"], map_cfg, pad)
    if region is None:
        return None
    return region_depth(frame, region, unit_mm, min_mm, val_max)


def attach_depths(objs, frame, unit_mm, map_cfg=None, classes=None,
                  max_boxes=3, pad=0.06, min_mm=MIN_MM):
    """就地给 objs 里的目标框挂 `depth_mm`（毫米整数，或 None）。

    只处理 `classes` 里的类（None = 全部），且**最多 max_boxes 个**（按框面积从大到小）——
    MicroPython 下逐像素的 Python 循环很慢，框一多会让 10Hz 的推理节拍掉帧。
    """
    picks = [o for o in objs if classes is None or o.get("cls") in classes]
    picks.sort(key=lambda o: -((o["box"][2] - o["box"][0]) * (o["box"][3] - o["box"][1])))
    for o in picks[:max_boxes]:
        o["depth_mm"] = locate_depth(frame, o["box"], unit_mm, map_cfg,
                                     pad=pad, min_mm=min_mm)
