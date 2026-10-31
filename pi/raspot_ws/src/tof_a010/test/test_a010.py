# -*- coding: utf-8 -*-
"""`tof_a010.a010` 的离线单测。**不碰硬件、不碰 ROS。**"""
import pytest

from tof_a010 import a010


# --------------------------------------------------------------------------
# 造帧
# --------------------------------------------------------------------------

def mk_frame(payload, rows=100, cols=100, frame_id=7, *, csum=None, tail=0xDD,
             data_len=None):
    """按实测格式拼一帧：`00 FF | 20B 头 | payload | sum8 | DD`。"""
    payload = bytes(payload)
    if data_len is None:
        data_len = 16 + len(payload)
    head = bytearray(20)
    head[0], head[1] = 0x00, 0xFF
    head[2] = data_len & 0xFF
    head[3] = (data_len >> 8) & 0xFF
    head[5] = 0                     # output_mode: 0 = 只有深度
    head[14] = rows
    head[15] = cols
    head[16] = frame_id & 0xFF
    head[17] = (frame_id >> 8) & 0xFF
    body = bytes(head) + payload
    if csum is None:
        csum = sum(body) & 0xFF
    return body + bytes([csum, tail])


def flat(value, rows=100, cols=100):
    return [value] * (rows * cols)


# --------------------------------------------------------------------------
# 帧解析：同步、粘包、半包、坏校验
# --------------------------------------------------------------------------

def test_parses_a_whole_frame():
    raw = mk_frame(flat(40), frame_id=123)
    fr, rest = a010.parse_frame(raw)
    assert fr is not None and rest == b""
    assert (fr.rows, fr.cols) == (100, 100)
    assert fr.frame_id == 123
    assert len(fr.payload) == 10000
    assert fr.payload[0] == 40


def test_returns_none_on_half_a_frame_and_keeps_the_bytes():
    raw = mk_frame(flat(40))
    fr, rest = a010.parse_frame(raw[:500])
    assert fr is None
    assert rest == raw[:500]          # 一个字节都不能丢


def test_sticky_packets_two_frames_in_one_buffer():
    a = mk_frame(flat(10), frame_id=1)
    b = mk_frame(flat(20), frame_id=2)
    fr, rest = a010.parse_frame(a + b)
    assert fr.frame_id == 1 and fr.payload[0] == 10
    fr2, rest2 = a010.parse_frame(rest)
    assert fr2.frame_id == 2 and fr2.payload[0] == 20
    assert rest2 == b""


def test_byte_by_byte_feed_eventually_yields_the_frame():
    raw = mk_frame(flat(33), frame_id=99)
    buf = b""
    fr = None
    for i in range(len(raw)):
        fr, buf = a010.parse_frame(buf + raw[i:i + 1])
        if fr is not None:
            break
    assert fr is not None and fr.frame_id == 99
    assert fr.payload[0] == 33


def test_leading_noise_is_skipped():
    raw = b"\x01\x02\x03garbage" + mk_frame(flat(50), frame_id=5)
    fr, rest = a010.parse_frame(raw)
    assert fr is not None and fr.frame_id == 5


def test_bad_checksum_only_drops_one_byte_so_the_next_frame_survives():
    """★ 这条是"整段丢弃"与"只丢 1 字节"的分水岭。"""
    bad = mk_frame(flat(10), frame_id=1, csum=0x00 & 0xFF)
    good = mk_frame(flat(60), frame_id=2)
    # 故意让坏校验和"碰巧"不匹配
    if bad[a010.HEAD_SIZE + 10000] == sum(bad[:a010.HEAD_SIZE + 10000]) & 0xFF:
        pytest.skip("构造的坏校验和恰好正确")
    fr, rest = a010.parse_frame(bad + good)
    assert fr is not None and fr.frame_id == 2, "真帧头被整段丢弃吞掉了"


def test_bad_tail_byte_is_rejected():
    bad = mk_frame(flat(10), frame_id=1, tail=0x00)
    fr, rest = a010.parse_frame(bad)
    assert fr is None


def test_illegal_length_is_rejected():
    raw = mk_frame(flat(10), data_len=60000)      # > 16 + 10000
    fr, rest = a010.parse_frame(raw)
    assert fr is None


def test_zero_length_data_is_rejected():
    raw = mk_frame(flat(10), data_len=8)          # < 16
    fr, rest = a010.parse_frame(raw)
    assert fr is None


def test_header_split_across_two_reads_is_kept():
    raw = mk_frame(flat(10))
    fr, rest = a010.parse_frame(raw[:1])          # 只给了那个 0x00
    assert fr is None and rest == b"\x00"
    fr2, _ = a010.parse_frame(rest + raw[1:])
    assert fr2 is not None


# --------------------------------------------------------------------------
# 区域深度：min / 下限 / 饱和 / 排除
# --------------------------------------------------------------------------

def _frame_with(rows_vals):
    """`rows_vals`: dict {(r,c): count}，其余为 0（无回波）。"""
    pl = [0] * 10000
    for (r, c), v in rows_vals.items():
        pl[r * 100 + c] = v
    return a010.Frame(1, 100, 100, 0, 0, 0, 0, 0, bytes(pl))


def test_region_min_wins_over_far_background():
    """框里混着桌面(200)和目标顶面(105) ⇒ 必须取 105*4。"""
    f = _frame_with({(50, 50): 200, (52, 51): 105, (53, 52): 180})
    region = a010.region_bounds([0.45, 0.45, 0.55, 0.55], pad=0.0)
    assert a010.region_depth(f, region, unit_mm=4.0) == 105 * 4


def test_near_field_false_echo_is_cut_by_the_150mm_floor():
    """★ 模块自身的 48mm 假回波（实测：最左一列恒读 12 计数）必须被 150mm 切掉。"""
    f = _frame_with({(50, 1): 12, (50, 50): 110})
    region = a010.region_bounds([0.0, 0.40, 1.0, 0.60], pad=0.0)   # 把最左列框进来
    assert a010.region_depth(f, region, unit_mm=4.0, min_mm=150.0) == 110 * 4


def test_saturated_255_is_treated_as_invalid():
    f = _frame_with({(50, 50): 255, (50, 51): 120})
    region = a010.region_bounds([0.4, 0.4, 0.6, 0.6], pad=0.0)
    assert a010.region_depth(f, region, unit_mm=4.0) == 120 * 4


def test_all_invalid_region_returns_none_not_zero():
    """★ 契约：拿不到就 `None`，**绝不回落成常数**（回落会让闸形同虚设）。"""
    f = _frame_with({(50, 50): 0, (50, 51): 255})
    region = a010.region_bounds([0.4, 0.4, 0.6, 0.6], pad=0.0)
    assert a010.region_depth(f, region, unit_mm=4.0) is None


def test_box_entirely_outside_the_tof_view_returns_none():
    f = _frame_with({(50, 50): 100})
    assert a010.box_depth(f, [1.2, 0.4, 1.5, 0.6]) is None


def test_exclude_region_is_skipped():
    f = _frame_with({(50, 50): 40, (50, 60): 120})
    region = a010.region_bounds([0.4, 0.4, 0.7, 0.6], pad=0.0)
    assert a010.region_depth(f, region, unit_mm=4.0, min_mm=0.0) == 40 * 4
    assert a010.region_depth(f, region, unit_mm=4.0, min_mm=0.0,
                             exclude=[(0.0, 0.0, 0.55, 1.0)]) == 120 * 4


def test_honest_scale_is_applied():
    """把 UNIT 写错 1 ⇒ 结果必须跟着错（标定脚本靠这条能变红）。"""
    f = _frame_with({(50, 50): 100})
    region = a010.region_bounds([0.4, 0.4, 0.6, 0.6], pad=0.0)
    assert a010.region_depth(f, region, unit_mm=4.0) == 400
    assert a010.region_depth(f, region, unit_mm=5.0) == 500


# --------------------------------------------------------------------------
# 仿射 & 块图
# --------------------------------------------------------------------------

def test_identity_map_matches_the_box():
    r = a010.region_bounds([0.2, 0.3, 0.6, 0.7], pad=0.0)
    assert r == (30, 70, 20, 60)


def test_affine_map_shifts_and_scales():
    # x: 0.5*0.25+0.1 = 0.225 → c0=22 ; 0.5*0.75+0.1 = 0.475 → c1=ceil 47.5 = 48
    # y: 0.5*0.25     = 0.125 → r0=12 ; 0.5*0.75     = 0.375 → r1=ceil 37.5 = 38
    r = a010.region_bounds([0.25, 0.25, 0.75, 0.75], map_cfg={"ax": 0.5, "bx": 0.1,
                                                              "ay": 0.5, "by": 0.0},
                           pad=0.0)
    assert r == (12, 38, 22, 48)


def test_pad_is_ignored_at_the_border_after_clamping():
    r = a010.region_bounds([0.0, 0.0, 0.1, 0.1], pad=0.06)
    assert r[0] == 0 and r[2] == 0
    assert r[1] == 16 and r[3] == 16      # (0.1+0.06)*100 = 16


def test_block_map_reports_none_for_empty_blocks():
    f = _frame_with({(5, 5): 100})        # 只有左上角有回波
    bm = a010.block_map(f, n=10)
    assert bm[0][0] == 100
    assert bm[5][5] is None
    assert "r 0" in a010.format_block_map(bm)


def test_map_point_4_param():
    m = {"ax": 0.5, "bx": 0.1, "ay": 2.0, "by": -0.2}
    px, py = a010.map_point(m, 0.4, 0.5)
    assert (px, py) == pytest.approx((0.3, 0.8))


def test_map_point_6_param():
    m = {"a11": 1.0, "a12": 0.0, "a13": 0.1,
         "a21": 0.0, "a22": 1.0, "a23": -0.1}
    px, py = a010.map_point(m, 0.4, 0.5)
    assert (px, py) == pytest.approx((0.5, 0.4))


def test_six_param_map_uses_the_bounding_box_of_four_corners():
    """6 参下框会变成平行四边形。

    这个例子特意选成**只映射两个对角点会塌掉**的形状：
    `(x, y) -> (x + y, -x + y)`，框取 (0.2,0.2)-(0.4,0.4)。
    对角点 (0.2,0.2)→(0.4,0.0)、(0.4,0.4)→(0.8,0.0) ⇒ y 上下沿都是 0.0
    ⇒ 区域高度为 0 ⇒ **判成"没有区域"**（深度直接 None）。
    四角全映射才不会漏掉另外两个角跑出来的 y=0.2。
    """
    m = {"a11": 1.0, "a12": 1.0, "a13": 0.0,
         "a21": -1.0, "a22": 1.0, "a23": 0.0}
    assert (a010.map_point(m, 0.2, 0.2) == pytest.approx((0.4, 0.0)))
    r = a010.region_bounds([0.2, 0.2, 0.4, 0.4], map_cfg=m, pad=0.0)
    assert r == (0, 20, 40, 80)          # y: 0.0..0.2, x: 0.4..0.8


def test_box_depth_mm_path_matches_the_counts_path():
    """★ 两条路（计数 / 毫米）必须给同一个答案 —— 客户端走毫米那条，
    节点与离线工具走计数那条，差一个 bit 就是"节点里对、抓取偏"。"""
    f = _frame_with({(50, 50): 100, (52, 51): 40, (53, 52): 200})
    mm = a010.depth_map_mm(f, unit_mm=4.0)
    box = [0.45, 0.45, 0.55, 0.55]
    a = a010.box_depth(f, box, unit_mm=4.0, pad=0.0, min_mm=150.0)
    b = a010.box_depth_mm(mm, 100, 100, box, unit_mm=4.0, pad=0.0, min_mm=150.0)
    assert a == b == 40 * 4 == 160


def test_box_depth_mm_treats_below_floor_as_missing():
    f = _frame_with({(50, 50): 20})       # 20*4 = 80mm < 150mm 下限
    mm = a010.depth_map_mm(f, unit_mm=4.0)
    assert a010.box_depth_mm(mm, 100, 100, [0.4, 0.4, 0.6, 0.6],
                             pad=0.0, min_mm=150.0) is None


def test_depth_map_marks_invalid_as_zero():
    f = _frame_with({(0, 0): 100, (0, 1): 0, (0, 2): 255})
    mm = a010.depth_map_mm(f, unit_mm=4.0)
    assert mm[0] == 400 and mm[1] == 0 and mm[2] == 0


# --------------------------------------------------------------------------
# 桌面深度（物体高 h = 桌面 − 顶面）
# --------------------------------------------------------------------------

def _scene(rows=20, cols=20, top=200, table=260, obj=(5, 15, 5, 15), far=None):
    v = [table] * (rows * cols)
    r0, r1, c0, c1 = obj
    for r in range(r0, r1):
        for c in range(c0, c1):
            v[r * cols + c] = top
    if far:                          # 边上一条更远的地面
        for r in range(rows):
            for c in range(far):
                v[r * cols + c] = 600
    return v


def test_region_table_finds_the_background_plane():
    v = _scene()
    t = a010.region_table_mm(v, (0, 20, 0, 20), 20, 20, top_mm=200, lo=150, hi=1016)
    assert t == pytest.approx(260, abs=1)


def test_region_table_uses_mode_not_max():
    """★ 边上露出更远的地面（3 列 = 15%）不能把桌面拉远（拉远 ⇒ h 测大 ⇒ 扎太深戳桌面）。
    变异（已实测）：把众数换成 max(far) ⇒ 这条红。"""
    v = _scene(far=3)
    t = a010.region_table_mm(v, (0, 20, 0, 20), 20, 20, top_mm=200, lo=150, hi=1016)
    assert t == pytest.approx(260, abs=1)


def test_region_table_none_when_object_fills_region():
    v = _scene(obj=(0, 20, 0, 19))                 # 只剩 1 列桌面 = 5% < 15%
    assert a010.region_table_mm(v, (0, 20, 0, 20), 20, 20, top_mm=200,
                                lo=150, hi=1016) is None


def test_region_min_kth_ignores_isolated_flying_pixel():
    """★ 2026-10-31 抓茄子：区域里 1 个 212mm 飞点（夹爪边缘），真顶面 ≥296 ⇒
    kth=3 必须给出顶面，不能是飞点。变异（已实测）：kth 不传下去 ⇒ 这条红。"""
    v = [320] * 400
    for i in range(150, 160):
        v[i] = 300                    # 真顶面：10 个像素
    v[42] = 212                       # 飞点
    d = a010.box_depth_mm(v, 20, 20, [0.0, 0.0, 1.0, 1.0], pad=0.0, min_mm=150,
                          val_max=254, kth=3)
    assert d == 300
    assert a010.box_depth_mm(v, 20, 20, [0.0, 0.0, 1.0, 1.0], pad=0.0, min_mm=150,
                             val_max=254) == 212          # kth=1 = 旧行为


def test_region_min_kth_falls_back_when_few_pixels():
    v = [0] * 400
    v[10], v[11] = 300, 310
    assert a010.region_min(v, (0, 20, 0, 20), 20, 20, 150, 1016, kth=3) == 310


def test_region_min_kth_on_uint16_array():
    """/tof/depth 是 uint16：结果要对。（取负的回绕恰好保序，不转 int 也对、只是刷警告 ——
    这条**不**守 int() 转换，守的是 uint16 输入下 kth 的结果。）"""
    np = pytest.importorskip('numpy')
    v = np.full(400, 320, dtype=np.uint16)
    v[150:160] = 300
    v[42] = 212
    assert a010.region_min(v, (0, 20, 0, 20), 20, 20, 150, 1016, kth=3) == 300
