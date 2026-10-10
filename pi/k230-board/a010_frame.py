# /sdcard/k230vision/a010_frame.py —— MaixSense A010 ToF 串口帧的解析（**纯逻辑**）
#
# 为什么单独一个模块：和 results.py 一样，把**不碰硬件**的东西拎出来，
# 好让 t_tof.py 能在板上（甚至 PC 上）零风险地把它测红。
#
# 协议来源：官方 ROS 包 sipeed/MaixSense_ROS → sipeed_tof_ms_a010_ros
# 的 frame_struct.h / frame_handle.cc（**逐字段核过，不要凭记忆改**）。
#
# 帧布局（全部**小端**）：
#
#   偏移   长度  字段
#   0..1    2    帧头 0x00 0xFF            （= 0xFF00 小端）
#   2..3    2    frame_data_len            （= 16 + payload 字节数）
#   4       1    reserved1 (0xFF)
#   5       1    output_mode  0=depth only / 1=depth+ir
#   6       1    sensor_temp
#   7       1    driver_temp
#   8..11   4    exposure_time[4]
#   12      1    error_code
#   13      1    reserved2 (0x00)
#   14      1    resolution_rows
#   15      1    resolution_cols
#   16..17  2    frame_id (12 bit, 0~4095)
#   18      1    isp_version
#   19      1    reserved3 (0xFF)
#   20..     L    payload: rows*cols 个 **uint8**（深度值）
#   +L      1    checksum = 前面 (20 + L) 字节的 uint8 累加和
#   +L+1    1    帧尾 0xDD
#
#   其中 L = frame_data_len - 16；整帧字节数 = 20 + L + 2。
#   100x100 时 L = 10000，整帧 = 10022 字节。

FRAME_BEGIN_FLAG = 0xFF00
FRAME_END_FLAG = 0xDD
FRAME_HEAD_SIZE = 20          # 头部总字节数（含 payload 之外的元数据）
FRAME_HEAD_DATA_SIZE = 16     # frame_data_len 里"不算 payload"的那 16 字节
FRAME_CHECKSUM_SIZE = 1
FRAME_END_SIZE = 1
MAX_PAYLOAD = 100 * 100       # 官方解析器也是拿这个当上限（frame_handle.cc）

_BEGIN_LO = FRAME_BEGIN_FLAG & 0xFF   # 0x00
_BEGIN_HI = (FRAME_BEGIN_FLAG >> 8) & 0xFF  # 0xFF
_HEADER_BYTES = bytes((_BEGIN_LO, _BEGIN_HI))


def feed(buf):
    """往累积缓冲里喂数据，尝试取出一帧。

    buf: bytearray/bytes —— 上一次调用返回的 remainder 接上新读到的字节。

    返回 (frame, rest)：
      * 解出一个**校验通过**的完整帧 -> (dict, 帧后面的剩余字节)
      * 数据还不够/还没出现帧头      -> (None, 修剪后的缓冲，等下次再喂)

    设计说明（对应 frame_handle.cc 的做法）：
      - 帧头没找到时，只保留末尾可能是半截帧头的一个 0x00，其余丢弃；
      - 帧头之后的长度字段若非法（< 16 或 payload > 10000），
        丢掉**一个字节**再重找 —— 而不是整段丢，避免真帧头被误吞；
      - checksum 或帧尾不符，同样只丢一个字节重找。
    """
    if len(buf) < 2:
        # 还凑不出帧头：如果这一个是 0x00，留着（可能是帧头前半）
        if len(buf) == 1 and buf[0] == _BEGIN_LO:
            return None, bytes(buf)
        return None, b""

    while True:
        i = buf.find(_HEADER_BYTES)
        if i < 0:
            # 没有 "00 FF"。留末尾一个 0x00（可能是下一帧头的前半）。
            if buf[-1] == _BEGIN_LO:
                return None, bytes(buf[-1:])
            return None, b""
        if i > 0:
            buf = buf[i:]

        if len(buf) < FRAME_HEAD_SIZE:
            return None, bytes(buf)      # 头部还没收全

        data_len = buf[2] | (buf[3] << 8)
        if data_len < FRAME_HEAD_DATA_SIZE:
            buf = buf[1:]
            continue
        plen = data_len - FRAME_HEAD_DATA_SIZE
        if plen > MAX_PAYLOAD:
            buf = buf[1:]
            continue

        total = FRAME_HEAD_SIZE + plen + FRAME_CHECKSUM_SIZE + FRAME_END_SIZE
        if len(buf) < total:
            return None, bytes(buf)      # payload 还没收全

        csum = 0
        for b in buf[:FRAME_HEAD_SIZE + plen]:
            csum += b
        csum &= 0xFF
        if csum != buf[FRAME_HEAD_SIZE + plen] or \
                buf[FRAME_HEAD_SIZE + plen + FRAME_CHECKSUM_SIZE] != FRAME_END_FLAG:
            buf = buf[1:]
            continue                      # 坏帧：丢一个字节重找

        frame = {
            "frame_id": buf[16] | (buf[17] << 8),
            "rows": buf[14],
            "cols": buf[15],
            "output_mode": buf[5],
            "sensor_temp": buf[6],
            "driver_temp": buf[7],
            "error_code": buf[12],
            "isp_version": buf[18],
            "payload": bytes(buf[FRAME_HEAD_SIZE:FRAME_HEAD_SIZE + plen]),
        }
        return frame, bytes(buf[total:])


def build_frame(rows, cols, payload, frame_id=1, output_mode=0,
                error_code=0, corrupt_checksum=False, corrupt_tail=False,
                head_len_delta=0):
    """**只给测试用**：造一帧合法（或故意弄坏）的字节流。

    这不属于生产路径 —— 它的存在是为了让 t_tof.py 能造出"必须被接受"和
    "必须被拒"的输入。corrupt_* 参数就是变异锚点。
    """
    head = bytearray(FRAME_HEAD_SIZE)
    head[0] = _BEGIN_LO
    head[1] = _BEGIN_HI
    data_len = FRAME_HEAD_DATA_SIZE + len(payload) + head_len_delta
    head[2] = data_len & 0xFF
    head[3] = (data_len >> 8) & 0xFF
    head[4] = 0xFF
    head[5] = output_mode
    head[12] = error_code
    head[14] = rows
    head[15] = cols
    head[16] = frame_id & 0xFF
    head[17] = (frame_id >> 8) & 0xFF
    head[19] = 0xFF

    body = bytes(head) + bytes(payload)
    csum = 0
    for b in body:
        csum += b
    if corrupt_checksum:
        csum = (csum + 1) & 0xFF
    csum &= 0xFF
    tail = 0x00 if corrupt_tail else FRAME_END_FLAG
    return body + bytes((csum, tail))


def center_mm(frame, unit_mm, frac=0.2, val_max=255):
    """取画面**中心** frac×frac 区域里的深度中位数，换算成毫米。

    unit_mm: 一个深度计数代表多少毫米（由 AT+UNIT 决定，**必须实测标定**）。
    返回 None 表示该区域全是无效值（0）。

    为什么要中位数、不要均值：单点噪声大、还有"无回波"的洞（值 0）；
    中位数对少量坏点不敏感。为什么滤 0：0 通常表示无效（超出量程/无回波），
    不是"距离 0"，混进来会把统计拉飞。
    """
    rows = frame["rows"]
    cols = frame["cols"]
    payload = frame["payload"]
    if rows == 0 or cols == 0 or len(payload) < rows * cols:
        return None
    r0 = int(rows * (0.5 - frac / 2.0))
    r1 = max(r0 + 1, int(rows * (0.5 + frac / 2.0)))
    c0 = int(cols * (0.5 - frac / 2.0))
    c1 = max(c0 + 1, int(cols * (0.5 + frac / 2.0)))
    vals = []
    for r in range(r0, r1):
        base = r * cols
        for c in range(c0, c1):
            v = payload[base + c]
            if 0 < v <= val_max:
                vals.append(v)
    if not vals:
        return None
    vals.sort()
    mid = vals[len(vals) // 2]
    return mid * unit_mm
