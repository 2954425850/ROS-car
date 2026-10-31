# -*- coding: utf-8 -*-
"""A010（MetaSense Lite）**纯逻辑**：帧解析 + 框→深度。零依赖，能离线单测。

移植自 K230 侧**已上机验证**的两个文件（不是重写，是搬运）：

    k230vision/a010_frame.py  feed()          —— 帧同步 / 校验状态机
    k230vision/tof_locate.py  region_depth()  —— 框 → 区内最小有效计数

★ 本模块**不碰硬件、不 import 任何第三方**（连 numpy 都不）。读口在 `serial_dev.py`，
  发话题在 `tof_node.py`，测试打的是这里。

## 帧格式（实测确认，2026-10-14 在 Pi 上）

    [0x00 0xFF] | 20B 头 | payload | sum8 | 0xDD
                  ↑ buf[2] | buf[3]<<8 = data_len = 16 + payload_len

- payload 每像素 **1 字节计数**，`mm = count * UNIT`。
  ★ 这条 2026-10-14 在真机上验过：同一场景下 UNIT=4/2/1/8 的 min 计数分别是
  12/24/48/6 ⇒ 折算 48/48/48/48 mm，**四个值完全一致** ⇒ `mm = count*UNIT` 成立，
  而且 **48mm 那个近场假回波是模块自身的**（当时臂不在视野里！见下）。
- **0 = 无回波**，**255 = 饱和 / 超量程**，两个都当**无效**。
- ⚠️ `AT+UNIT?` 在出厂态回 `+UNIT= 0`（一个退化值）。**别信它**，初始化时显式设。

## 别重蹈的两个坑（都来自 K230 侧）

1. **不做一维聚类**。40cm 处瓶盖顶面与桌面只差 **3 计数**，而**同一个面**因倾斜就有
   ~8 计数散度 —— 任何 jump 阈值不是把两个面合掉、就是把一个面切碎成几十簇。
   ⇒ 结论是「**区内取最小**」，不是聚类、不是中位数、不是中心像素。
   （实测 41~42cm：min 420.6 / 中位数 440.6 / 中心像素 448.3 mm，尺量真值 ⇒ **min 最准**。）
2. **量程下限 150mm 是硬闸**。2026-10-14 实测确认：模块前**没有**任何东西时，
   最左一列仍恒读 12 计数（=48mm），即**近场假回波是模块自身的**，不是夹爪造成的。
   150mm 这一刀照样把它切掉。
"""
from __future__ import annotations

# ---- 帧结构常量 -----------------------------------------------------------
FRAME_HEAD = b"\x00\xff"          # 实测：真机上帧就是这两个字节起头
FRAME_END = 0xDD
HEAD_SIZE = 20                    # 含 20 字节头里 payload 之前的所有字段
HEAD_DATA_SIZE = 16               # data_len = 16 + payload_len
CHECKSUM_SIZE = 1
END_SIZE = 1
MAX_PAYLOAD = 100 * 100           # 100x100，超过就是假帧头
MAX_DATA_LEN = HEAD_DATA_SIZE + MAX_PAYLOAD

# ---- 深度语义 -------------------------------------------------------------
INVALID = 0                       # 无回波
SATURATED = 255                   # 饱和 / 超量程
DEFAULT_UNIT_MM = 4.0             # mm/计数；满量程 255*4 = 1020mm
DEFAULT_MIN_MM = 150.0            # 量程下限（模块额定 15cm）；见模块头 ★
DEFAULT_VAL_MAX = 254             # 只接受 count <= 254（255 当无效）
DEFAULT_PAD = 0.06                # 框四周多留的余量（归一化）

_ROWS = 100
_COLS = 100


class Frame(object):
    """一帧深度。字段名与 K230 侧一致，便于两边对着看日志。"""

    __slots__ = ("frame_id", "rows", "cols", "output_mode", "sensor_temp",
                 "driver_temp", "error_code", "isp_version", "payload")

    def __init__(self, frame_id, rows, cols, output_mode, sensor_temp,
                 driver_temp, error_code, isp_version, payload):
        self.frame_id = frame_id
        self.rows = rows
        self.cols = cols
        self.output_mode = output_mode
        self.sensor_temp = sensor_temp
        self.driver_temp = driver_temp
        self.error_code = error_code
        self.isp_version = isp_version
        self.payload = payload

    def __repr__(self):
        return ("Frame(id=%s %dx%d mode=%d err=%d temp=%d/%d payload=%dB)"
                % (self.frame_id, self.rows, self.cols, self.output_mode,
                   self.error_code, self.sensor_temp, self.driver_temp,
                   len(self.payload)))


def parse_frame(buf):
    """从 `buf` 里取出**一个**合法帧。返回 `(Frame | None, 剩余字节)`。

    ## 重扫策略（★ 别改成"整段丢弃"）

    头部找不到 ⇒ 只保留末尾那个可能是半个帧头的 `0x00`。
    长度非法 / 校验和错 / 尾字节错 ⇒ **只丢 1 个字节**再重扫。

    为什么是 1 字节：payload 里**完全可能**恰好出现 `00 FF`（实测会有），
    整段丢弃会把后面那个真帧头一起吞掉 —— 症状是"偶尔连丢好几帧、帧率忽高忽低"。
    丢 1 字节的代价只是多扫几次，而这里最多 10022 字节，够快。
    """
    buf = bytes(buf)
    while True:
        i = buf.find(FRAME_HEAD)
        if i < 0:
            # 没有帧头。若末尾是 0x00，它可能是被切断的帧头前半，留下。
            return None, (buf[-1:] if buf.endswith(b"\x00") else b"")
        if i > 0:
            buf = buf[i:]
        if len(buf) < HEAD_SIZE:
            return None, buf                     # 头还没收全

        data_len = buf[2] | (buf[3] << 8)
        if data_len < HEAD_DATA_SIZE or data_len > MAX_DATA_LEN:
            buf = buf[1:]                        # 假帧头，只丢 1 字节
            continue
        plen = data_len - HEAD_DATA_SIZE
        total = HEAD_SIZE + plen + CHECKSUM_SIZE + END_SIZE
        if len(buf) < total:
            return None, buf                     # 帧没收全，留着等下一批

        csum = 0
        for b in buf[:HEAD_SIZE + plen]:
            csum += b
        csum &= 0xFF
        if csum != buf[HEAD_SIZE + plen] or buf[HEAD_SIZE + plen + CHECKSUM_SIZE] != FRAME_END:
            buf = buf[1:]                        # 校验/尾字节不符，只丢 1 字节
            continue

        fr = Frame(frame_id=buf[16] | (buf[17] << 8),
                   rows=buf[14], cols=buf[15],
                   output_mode=buf[5], sensor_temp=buf[6], driver_temp=buf[7],
                   error_code=buf[12], isp_version=buf[18],
                   payload=bytes(buf[HEAD_SIZE:HEAD_SIZE + plen]))
        return fr, bytes(buf[total:])


# --------------------------------------------------------------------------
# 框 → ToF 区域 → 深度
# --------------------------------------------------------------------------

def map_point(map_cfg, x, y):
    """推流归一化 `(x, y)` → ToF 归一化 `(x', y')`。

    支持两种写法（`region_bounds` 会挑）：

    - **4 参**（默认）：`x' = ax*x + bx`，`y' = ay*y + by`。
      隐含「两个光轴平行、只是缩放+平移」。同侧不共轴时够用。
    - **6 参**：`x' = a11*x + a12*y + a13`，`y' = a21*x + a22*y + a23`。
      多了**旋转/错切**。标定残差大就是缺这个（两块板子装歪了几度）。
    """
    m = map_cfg or {}
    if "a11" in m:
        return (float(m["a11"]) * x + float(m["a12"]) * y + float(m["a13"]),
                float(m["a21"]) * x + float(m["a22"]) * y + float(m["a23"]))
    return (float(m.get("ax", 1.0)) * x + float(m.get("bx", 0.0)),
            float(m.get("ay", 1.0)) * y + float(m.get("by", 0.0)))


def region_bounds(box_norm, map_cfg=None, pad=DEFAULT_PAD, rows=_ROWS, cols=_COLS):
    """归一化框（1280x720 推流系）→ ToF 像素区域 `(r0, r1, c0, c1)`，半开区间。

    映射是**仿射**（见 `map_point`）：ToF 与相机同侧但不共轴，
    所以这个映射是个**固定**的缩放+平移（腕上安装 ⇒ 两者一起动，映射不随姿态变）。

    默认恒等，**只在「ToF 与相机近似共轴同视场」时成立**。
    没标定就是错的 —— 症状是框落到桌面上，深度系统性偏大。
    """
    l, t, r, b = box_norm
    # ★ 四个角全映射再取外接矩形：6 参映射下框会变成平行四边形，
    #   只映射 (l,t) 和 (r,b) 两个对角点会**低估**区域（转得越厉害漏得越多）。
    xs, ys = [], []
    for (x, y) in ((l, t), (r, t), (l, b), (r, b)):
        px, py = map_point(map_cfg, x, y)
        xs.append(px)
        ys.append(py)
    x0, x1 = min(xs), max(xs)
    y0, y1 = min(ys), max(ys)
    # 整个框都落在 ToF 视野外 ⇒ 没有区域（不是"夹到边上"，是"根本没看到"）
    if x1 <= 0.0 or x0 >= 1.0 or y1 <= 0.0 or y0 >= 1.0:
        return None
    c0 = int((x0 - pad) * cols)
    c1 = int((x1 + pad) * cols + 0.999)          # +0.999 = 上沿向上取整
    r0 = int((y0 - pad) * rows)
    r1 = int((y1 + pad) * rows + 0.999)
    c0, c1 = max(0, min(cols, c0)), max(0, min(cols, c1))
    r0, r1 = max(0, min(rows, r0)), max(0, min(rows, r1))
    if c1 <= c0 or r1 <= r0:
        return None
    return (r0, r1, c0, c1)


def region_min(values, region, rows, cols, lo, hi, exclude=None, kth=1):
    """区域内落在 `[lo, hi]` 的最小值（`kth>1` = 第 kth 小）；一个都没有就 `None`。

    ★ `kth`（2026-10-31）：单个**飞点**（夹爪边缘的混合像素，实测第 20 列 55 帧里偶发 220mm，
      邻列都 ≥296）就能把裸最小值拉走 ⇒ 目标点高出几厘米、抓在茄子上方。取第 3 小：
      真实顶面总有好几个像素，飞点只有零星一两个。有效像素不足 kth 个 ⇒ 退回其中最大的那个。

    **核心是一条**：区域 + 下限 + 上限 → 取最小。`values` 的物理含义由调用方给
    （计数走 `region_depth`，毫米走 `tof_client` 那条），但**取最小的理由只有一份**：

    ★ **取最小、不取中位数**：框里混着更远的桌面，中位数会被拉偏 +20mm
      （实测 41~42cm：min 420.6 / 中位数 440.6 / 中心像素 448.3，尺量真值 ⇒ min 最准）；
      取最小天然贴住目标顶面。代价是怕近场假回波 —— 所以有下限兜着（见模块头 ★）。
    """
    r0, r1, c0, c1 = region
    if rows <= 0 or cols <= 0 or len(values) < rows * cols:
        return None

    skip = None
    if exclude:
        # 归一化排除区 → 像素半开区间，并夹进当前 region（只有交集才需要跳过）
        skip = []
        for (el, et, er, eb) in exclude:
            e0 = max(r0, min(r1, int(et * rows)))
            e1 = max(r0, min(r1, int(eb * rows + 0.999)))
            f0 = max(c0, min(c1, int(el * cols)))
            f1 = max(c0, min(c1, int(er * cols + 0.999)))
            if e1 > e0 and f1 > f0:
                skip.append((e0, e1, f0, f1))
        if not skip:
            skip = None

    import heapq
    k = max(1, int(kth))
    small = []                        # 最大堆（取负）存最小的 k 个
    for r in range(r0, r1):
        base = r * cols
        for c in range(c0, c1):
            if skip is not None and any(e0 <= r < e1 and f0 <= c < f1
                                        for (e0, e1, f0, f1) in skip):
                continue
            v = values[base + c]
            if lo <= v <= hi:
                v = int(v) if float(v).is_integer() else float(v)   # numpy uint16 取负会溢出
                if len(small) < k:
                    heapq.heappush(small, -v)
                elif v < -small[0]:
                    heapq.heapreplace(small, -v)
    return -small[0] if small else None


def region_depth(frame, region, unit_mm=DEFAULT_UNIT_MM,
                 min_mm=DEFAULT_MIN_MM, val_max=DEFAULT_VAL_MAX, exclude=None):
    """区域内的**最小有效计数** × UNIT → 毫米整数；一个有效回波都没有就 `None`。

    `exclude`：可选的排除区列表，每个是 **ToF 归一化**的 `(l, t, r, b)`。
    默认空 —— 下限已经是第一道保险（见模块头 ★），排除区只是第二道。
    """
    frame_rows, frame_cols, pl = frame.rows, frame.cols, frame.payload
    if frame_rows <= 0 or frame_cols <= 0 or len(pl) < frame_rows * frame_cols:
        return None
    lo = max(1, int(min_mm / unit_mm + 0.999))   # 向上取整：只接受 >= min_mm
    best = region_min(pl, region, frame_rows, frame_cols, lo, int(val_max), exclude)
    return None if best is None else int(best * unit_mm)


def box_depth(frame, box_norm, unit_mm=DEFAULT_UNIT_MM, map_cfg=None,
              pad=DEFAULT_PAD, min_mm=DEFAULT_MIN_MM, val_max=DEFAULT_VAL_MAX,
              exclude=None):
    """上面两步的合并入口：**框 → 毫米**。给抓取链路用的就是这个。"""
    region = region_bounds(box_norm, map_cfg=map_cfg, pad=pad,
                           rows=frame.rows, cols=frame.cols)
    if region is None:
        return None
    return region_depth(frame, region, unit_mm=unit_mm, min_mm=min_mm,
                        val_max=val_max, exclude=exclude)


def box_depth_mm(mm_values, rows, cols, box_norm, unit_mm=DEFAULT_UNIT_MM,
                 map_cfg=None, pad=DEFAULT_PAD, min_mm=DEFAULT_MIN_MM,
                 val_max=DEFAULT_VAL_MAX, exclude=None, kth=1):
    """同 `box_depth`，但吃的是**已经换算成毫米**的一维数组（0 = 无效）。

    `tof_client` 走这条：它从 `/tof/depth` 话题（16UC1，毫米）拿数，
    不必为了复用解析逻辑再把它除回计数。

    ★ 阈值跟着换单位：下限还是 `min_mm`，上限是 `val_max * unit_mm`
      （计数 255 的饱和值换算成毫米）。
    """
    region = region_bounds(box_norm, map_cfg=map_cfg, pad=pad,
                           rows=rows, cols=cols)
    if region is None:
        return None
    return region_min(mm_values, region, rows, cols,
                      lo=min_mm, hi=val_max * unit_mm, exclude=exclude, kth=kth)


def region_table_mm(values, region, rows, cols, top_mm, lo, hi,
                    gap_mm=10.0, bin_mm=8.0, min_frac=0.15):
    """区域里**背景（桌面）**的深度，毫米；认不出就 `None`。给"物体高 h = 桌面 − 顶面"用。

    做法：只看比顶面 `top_mm` 远 `gap_mm` 以上的有效像素，按 `bin_mm` 分箱取**众数**
    （相邻三箱合计最多的那个窗口，返回窗口内均值）。
    ★ 取众数、不取最大值：框边上露出的远处地面/桌沿外只占少数像素，最大值会被它拉大
      ⇒ h 被测大 ⇒ 下扎过深、合爪戳桌面。众数跟着"占多数的那个平面"走。
    ★ 众数窗口不到区域有效像素的 `min_frac` ⇒ 不认（物体把框占满、看不到桌面）。
    """
    r0, r1, c0, c1 = region
    if rows <= 0 or cols <= 0 or len(values) < rows * cols:
        return None
    n_valid, far = 0, []
    for r in range(r0, r1):
        base = r * cols
        for c in range(c0, c1):
            v = float(values[base + c])
            if lo <= v <= hi:
                n_valid += 1
                if v >= top_mm + gap_mm:
                    far.append(v)
    if not far or n_valid == 0:
        return None
    bins = {}
    for v in far:
        bins.setdefault(int(v // bin_mm), []).append(v)
    best_k, best_n = None, 0
    for k in bins:
        n = sum(len(bins.get(k + i, ())) for i in (-1, 0, 1))
        if n > best_n or (n == best_n and best_k is not None and k < best_k):
            best_k, best_n = k, n
    if best_n < min_frac * n_valid:
        return None
    win = [v for i in (-1, 0, 1) for v in bins.get(best_k + i, ())]
    return sum(win) / len(win)


# --------------------------------------------------------------------------
# 给人看的
# --------------------------------------------------------------------------

def depth_map_mm(frame, unit_mm=DEFAULT_UNIT_MM, val_max=DEFAULT_VAL_MAX):
    """整帧 → 毫米列表（无效处为 0）。给 ROS Image 用。"""
    hi = int(val_max)
    out = [0] * (frame.rows * frame.cols)
    for i, v in enumerate(frame.payload):
        if 0 < v <= hi:
            out[i] = int(v * unit_mm)
        # v == 0 (无回波) / v == 255 (饱和) / 超出标定 ⇒ 留 0
    return out


def block_map(frame, n=10, unit_mm=DEFAULT_UNIT_MM,
              min_mm=DEFAULT_MIN_MM, val_max=DEFAULT_VAL_MAX):
    """`n x n` 块图，每格 = 该块内**有效计数的最小值**。无效块 = `None`。

    这是排障时最有用的一张图（K230 侧当初就是靠它认出「右上大片 0 = 倾斜无回波」
    和「最下排 12 = 近场假回波」的）。
    """
    rows, cols, pl = frame.rows, frame.cols, frame.payload
    out = []
    for r in range(n):
        row = []
        for c in range(n):
            best = None
            for rr in range(r * rows // n, (r + 1) * rows // n):
                base = rr * cols
                for cc in range(c * cols // n, (c + 1) * cols // n):
                    v = pl[base + cc]
                    if min_mm <= v * unit_mm and 0 < v <= val_max:
                        if best is None or v < best:
                            best = v
            row.append(None if best is None else best)
        out.append(row)
    return out


def format_block_map(bm, unit_mm=DEFAULT_UNIT_MM):
    """`block_map()` → 文本。列一起打（人眼找落点靠列号）。"""
    n = len(bm)
    lines = ["      " + "".join("%6d" % c for c in range(n))]
    for r, row in enumerate(bm):
        cells = []
        for v in row:
            cells.append("     ." if v is None else "%6d" % int(v * unit_mm))
        lines.append("  r%2d " % r + "".join(cells))
    return "\n".join(lines)
