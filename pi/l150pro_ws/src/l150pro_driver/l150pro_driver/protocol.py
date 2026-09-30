"""L150Pro C30D(2.0) 下位机串口协议（纯 Python，无 ROS 依赖）。

本文件是 STM32 固件 (usartx.c) 的对端实现。帧格式为双方冻结版本，
任何字段改动都必须先与固件侧确认，不要单方面修改本文件。

三个易错点（都踩过，写在这里防止后人重踩）：

1. C 的 `(short)float` 是【向零截断】，不是四舍五入。
   对端 Python 必须用 int()，不能用 round()。
   例：pitch = -5deg -> 真值 -872.665 -> 固件给 -872，round() 会给 -873。

2. 固件全程 float32，Python 是 double，两者末位可能差 1 LSB。
   本模块用 f32() 模拟 float32，因此可判【严格相等】而非容差比较。
   容差比较会掩盖"某个系数写错一位"这类只差 1 的真实 bug。

3. wrap 必须在【弧度域】做（固件在 rad 域 wrap）。角度域 wrap 数学等价
   但浮点路径不等价，实测分叉率约 0.2%。
"""

import struct
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# --------------------------------------------------------------------------
# 协议常量
# --------------------------------------------------------------------------

HDR_MOTION = 0xAA   # 下行：运动控制帧
HDR_SERVO = 0xAB    # 下行：舵机帧
HDR_UPLINK = 0x55   # 上行：状态帧

MOTION_FRAME_SIZE = 12
SERVO_FRAME_SIZE = 17
UPLINK_FRAME_SIZE = 28

# 下行 flags 位
FLAG_ENABLE = 1 << 0    # 使能；固件上电默认不动，必须置位才动
FLAG_ESTOP = 1 << 1     # 急停

# 上行 fault 位
FAULT_UNDERVOLT = 1 << 0    # 欠压 (<10V)
FAULT_OVERVOLT = 1 << 1     # 过压
FAULT_ESTOP = 1 << 2        # 急停 (Flag_Stop)
FAULT_IMU = 1 << 3          # IMU 故障 / 初始化失败
FAULT_RX_TIMEOUT = 1 << 4   # 下行超时（看门狗触发）
FAULT_ENCODER = 1 << 5      # 编码器异常

# 舵机
SERVO_CHANNELS = 6
SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CENTER_US = 1500
SERVO_HOLD = 0              # 值 0 表示"该路保持不变"

# 云台水平通道（ch5）；ch6 是俯仰。
PAN_CHANNEL = 5


def servo_command_to_pulse(channel: int, us: int, pan_invert: bool = True) -> int:
    """把 /servo_cmd 的指令值换算成实际要发出去的脉宽（微秒）。

    对外契约（客户端要遵守的是这个，客户端不需要知道舵机怎么装的）：
        ch5  值增大 = 云台往右
        ch6  值增大 = 云台往上
        1500 为中位；0（或 <500）表示该路保持不动

    但 ch5 那颗舵机是**反装的** —— 物理上脉宽变大是往左。所以这一路要翻一次。
    翻转是对称的（500<->2500、1500<->1500），中位和行程都不变。
    哪天把舵机装正了，把 pan_invert 置 False 即可回到直通。

    us < SERVO_MIN_US（含 0 = 保持不变）不参与翻转：那不是位置指令，
    翻了会变成 3000。
    """
    if us < SERVO_MIN_US:
        return us
    if pan_invert and channel == PAN_CHANNEL:
        return SERVO_MIN_US + SERVO_MAX_US - us
    return us


# 底盘参数（四驱差速，与固件 robot_select_init.h 一致）
# 2026-09-17 换车后实测：轮径 125mm、MG540 霍尔 13 线 30 减速比
WHEEL_SPACING = 0.460       # FourWheel_real_wheelSpacing（左右轮心距，全值）
AXLE_SPACING = 0.540        # FourWheel_real_axlespacing（前后轮心距，全值）
TRACK_EFF = WHEEL_SPACING + AXLE_SPACING   # 有效全轮距 = 1.000 m
# 注意：逆解写的是 Vz*(W+A)/2，所以这里 1.000 是【全】轮距。
# 用 0.500 会让角速度大一倍，症状酷似"PID 调过头"，极难查。
# 1.000 是【外推初值】不是标定值：四驱滑移转向的等效轮距含侧滑修正，
# 必须原地转标定（monitor.py 按 c 键），标定前 odom 的 yaw 不要信。

# 固件侧限幅
MAX_WHEEL_SPEED = 3.5       # Drive_Motor 的 amplitude，m/s，超出被固件截断

CRC_CHECK_VECTOR = 0x4B37   # crc16_modbus(b"123456789")，两边必须一致


# --------------------------------------------------------------------------
# 基础工具
# --------------------------------------------------------------------------

def f32(x: float) -> float:
    """把 double 降级为 IEEE-754 binary32，模拟固件的 float 运算。"""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def sat_i16(v: float) -> int:
    """饱和到 int16 并【向零截断】，与固件 ros_sat_i16() 一致。

    注意用 int() 而不是 round()。Python 的 int() 同样是向零截断。
    """
    if v > 32767.0:
        return 32767
    if v < -32768.0:
        return -32768
    return int(v)


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS: poly 0x8005(反射 0xA001), init 0xFFFF, xorout 0x0000.

    自检：crc16_modbus(b"123456789") == 0x4B37
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def _wrap_pi(rad: float) -> float:
    """把弧度 wrap 到 (-pi, pi]，全程按 float32 路径。

    【必须在弧度域 wrap】，与固件同序。在角度域 wrap 虽然数学等价，
    但浮点路径不同，实测约 0.2% 的取值会在末位差 1。
    """
    PI = f32(3.14159265358979)
    TWO_PI = f32(6.28318530717959)
    while rad > PI:
        rad = f32(rad - TWO_PI)
    while rad < -PI:
        rad = f32(rad + TWO_PI)
    return rad


def unwrap_delta(prev: float, cur: float) -> float:
    """把回绕到 (-pi, pi] 的相邻两帧角度差解回【真实增量】。

    固件上行帧的 yaw 回绕到 +-PI（usartx.h 协议定义第 18 字段，特意加的）。
    原地转标定必然跨过 +-PI，此时直接做 cur - prev 会在跨越那一下凭空注入
    ∓2pi：转一整圈累计出来是 0 而不是 2pi。现象只是"标出来的轮距很离谱"，
    很难往回查到是回绕。凡是要累加连续转角，都必须走这个函数。
    """
    d = f32(cur - prev)
    PI = f32(3.14159265358979)
    TWO_PI = f32(6.28318530717959)
    while d > PI:
        d = f32(d - TWO_PI)
    while d < -PI:
        d = f32(d + TWO_PI)
    return d


class YawUnwrapper:
    """把回绕的 yaw 序列累加成连续转角。

        u = YawUnwrapper()
        for yaw in yaw_sequence:
            total = u.feed(yaw)      # 连续、跨 +-PI 不跳变
    """

    def __init__(self) -> None:
        self._prev: Optional[float] = None
        self.total = 0.0

    def feed(self, yaw: float) -> float:
        if self._prev is None:
            self._prev = yaw
            return self.total
        self.total = f32(self.total + unwrap_delta(self._prev, yaw))
        self._prev = yaw
        return self.total

    def reset(self) -> None:
        self._prev = None
        self.total = 0.0


def angle_deg_to_wire(deg: float) -> int:
    """度 -> 上行帧里的 angle 字段 (rad x 10000, int16)。

    这是固件侧那条链的参考实现，用于交叉验证：
        度 -> 弧度(float32) -> wrap到(-pi,pi] -> x10000 -> 向零截断

    固件实际的链还包含一次 rad->deg->rad 的往返（see imu_task.c），
    但那条往返在 float32 下是幂等的，实测与直接转弧度结果一致。
    """
    rad = _wrap_pi(f32(deg * f32(0.01745329252)))
    return sat_i16(f32(rad * 10000.0))


# --------------------------------------------------------------------------
# 下行帧构造
# --------------------------------------------------------------------------

def build_motion_frame(vx: float, vy: float, wz: float,
                       flags: int = 0, seq: int = 0) -> bytes:
    """构造 12 字节运动控制帧。

    vx, vy : m/s
    wz     : rad/s
    flags  : FLAG_ENABLE / FLAG_ESTOP
    """
    body = struct.pack("<BBhhhBB",
                       HDR_MOTION,
                       MOTION_FRAME_SIZE,
                       sat_i16(f32(vx * 1000.0)),    # mm/s
                       sat_i16(f32(vy * 1000.0)),    # mm/s
                       sat_i16(f32(wz * 1000.0)),    # mrad/s
                       flags & 0xFF,
                       seq & 0xFF)
    return body + struct.pack("<H", crc16_modbus(body))


def build_servo_frame(angles_us: List[int], seq: int = 0) -> bytes:
    """构造 17 字节舵机帧。

    angles_us : 长度 6 的列表，单位微秒。
                0 表示"该路保持当前位置不变"。
                合法范围 500~2500，1500 为中位。
    """
    if len(angles_us) != SERVO_CHANNELS:
        raise ValueError(f"需要 {SERVO_CHANNELS} 路舵机值，收到 {len(angles_us)}")
    # 0..1 帧头+长度, 2..13 六路 int16, 14 seq = 共 15 字节，之后 15..16 是 CRC
    body = struct.pack("<BB", HDR_SERVO, SERVO_FRAME_SIZE)
    body += b"".join(struct.pack("<h", sat_i16(float(a))) for a in angles_us)
    body += struct.pack("<B", seq & 0xFF)
    assert len(body) == 15, f"舵机帧 CRC 前应为 15 字节，实际 {len(body)}"
    return body + struct.pack("<H", crc16_modbus(body))


# --------------------------------------------------------------------------
# 上行帧解析
# --------------------------------------------------------------------------

@dataclass
class Uplink:
    """解析后的上行状态帧。"""
    tick_ms: int            # STM32 上电起的 FreeRTOS tick(ms)。
                            # 【不要当 ROS 时间戳用】——与 Pi 时钟不同源。
                            # 只用于丢帧检测：相邻帧差应约 20ms，40ms 说明丢帧。
    vA: float               # A 轮速度 m/s（已极性校正的物理前进速度）
    vB: float
    vC: float
    vD: float
    roll: float             # rad，重力参考，绝对不漂
    pitch: float            # rad，同上
    yaw: float              # rad，【纯陀螺仪积分，会漂，勿当绝对航向】
                            # 【回绕到 (-pi, pi]】：要累计连续转角请用
                            # YawUnwrapper，别直接做 cur - prev
    gz: float               # rad/s，陀螺仪原始测量，可信
    voltage: float          # V
    fault: int              # fault 位，见 FAULT_*
    seq: int

    # ---- 派生量 ----

    @property
    def v_left(self) -> float:
        """左侧轮速 = (A+B)/2（A/B 为左侧，C/D 为右侧）。"""
        return f32((self.vA + self.vB) * 0.5)

    @property
    def v_right(self) -> float:
        """右侧轮速 = (C+D)/2。"""
        return f32((self.vC + self.vD) * 0.5)

    @property
    def vx(self) -> float:
        """车体前进速度 m/s。"""
        return f32((self.v_left + self.v_right) * 0.5)

    def wz(self, track_eff: float = TRACK_EFF) -> float:
        """车体角速度 rad/s。用【全】轮距，不是半轮距。

        是方法不是属性：标定等效轮距是个迭代试值的过程（原地转一次、比对一次、
        按比例修正），monitor 用 --track-eff 把值传进来，免得每试一个值都要
        改常量 + colcon build。默认仍取模块常量 TRACK_EFF=1.000。
        """
        return f32((self.v_right - self.v_left) / track_eff)

    def fault_names(self) -> List[str]:
        names = []
        for bit, name in ((FAULT_UNDERVOLT, "欠压"),
                          (FAULT_OVERVOLT, "过压"),
                          (FAULT_ESTOP, "急停"),
                          (FAULT_IMU, "IMU故障"),
                          (FAULT_RX_TIMEOUT, "下行超时"),
                          (FAULT_ENCODER, "编码器异常")):
            if self.fault & bit:
                names.append(name)
        return names


def parse_uplink_frame(frame: bytes) -> Optional[Uplink]:
    """解析一帧完整的 28 字节上行帧。长度或 CRC 不符返回 None。"""
    if len(frame) != UPLINK_FRAME_SIZE:
        return None
    if frame[0] != HDR_UPLINK or frame[1] != UPLINK_FRAME_SIZE:
        return None
    if crc16_modbus(frame[:26]) != struct.unpack("<H", frame[26:28])[0]:
        return None

    (tick, vA, vB, vC, vD, roll, pitch, yaw, gz,
     voltage, fault, seq) = struct.unpack("<IhhhhhhhhHBB", frame[2:26])

    return Uplink(
        tick_ms=tick,
        vA=vA / 1000.0, vB=vB / 1000.0, vC=vC / 1000.0, vD=vD / 1000.0,
        roll=roll / 10000.0, pitch=pitch / 10000.0, yaw=yaw / 10000.0,
        gz=gz / 1000.0,
        voltage=voltage / 1000.0,
        fault=fault,
        seq=seq,
    )


def build_uplink_frame(up: Uplink) -> bytes:
    """构造 28 字节上行帧。

    这个函数【只用于自测与回环仿真】，真机上该帧由固件产生。
    有了它，ROS 侧可以完全不接硬件就跑通解析 -> 里程计 -> 发布的链路。
    """
    body = struct.pack(
        "<BB", HDR_UPLINK, UPLINK_FRAME_SIZE
    ) + struct.pack(
        "<IhhhhhhhhHBB",
        up.tick_ms & 0xFFFFFFFF,
        sat_i16(f32(up.vA * 1000.0)),
        sat_i16(f32(up.vB * 1000.0)),
        sat_i16(f32(up.vC * 1000.0)),
        sat_i16(f32(up.vD * 1000.0)),
        sat_i16(f32(up.roll * 10000.0)),
        sat_i16(f32(up.pitch * 10000.0)),
        sat_i16(f32(up.yaw * 10000.0)),
        sat_i16(f32(up.gz * 1000.0)),
        int(up.voltage * 1000.0) & 0xFFFF,
        up.fault & 0xFF,
        up.seq & 0xFF,
    )
    assert len(body) == 26, f"上行帧 CRC 前应为 26 字节，实际 {len(body)}"
    return body + struct.pack("<H", crc16_modbus(body))


# --------------------------------------------------------------------------
# 流式解析器（处理粘包 / 半包 / 丢字节 / 噪声）
# --------------------------------------------------------------------------

@dataclass
class ParserStats:
    frames: int = 0
    crc_err: int = 0
    head_err: int = 0
    dropped: int = 0        # 通过 tick 差值检测到的丢帧


class UplinkParser:
    """把串口字节流切成上行帧。

    统计口径【刻意对齐固件侧】，否则交叉验证会对不上：

      head_err : 遇到帧头 0x55，但紧随的【长度字节不等于 28】的次数。
                 扫描阶段的普通字节【不计入】——固件在 state 0 看到非帧头
                 字节时静默忽略，不计数。
      crc_err  : 帧头与长度都对、但 CRC 校验失败的次数。
      dropped  : 由 tick 差值推算的丢帧数。

    已知的语义差异（写在这里免得后人查半天）：
      本实现用 Python 的 find() 一次性跳过非帧头字节，固件是逐字节状态机。
      两者的 head_err/crc_err 计数在正常流量下一致；在极端噪声下可能有
      一两帧的差异（因为丢字节后的重新扫描起点不同）。对拍时若发现
      head_err 差几，先看是不是这个原因，不要直接判定协议有问题。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.stats = ParserStats()
        self._last_tick: Optional[int] = None

    def feed(self, data: bytes) -> List[Uplink]:
        """喂入任意字节流，返回本次新解析出的所有帧。"""
        self._buf.extend(data)
        out: List[Uplink] = []

        while True:
            if not self._buf:
                break

            # 1) 找帧头。扫描期间【不计数】，对齐固件 state 0 的静默忽略。
            if self._buf[0] != HDR_UPLINK:
                idx = self._buf.find(bytes([HDR_UPLINK]))
                if idx < 0:
                    self._buf.clear()
                    break
                del self._buf[:idx]
                continue

            # 2) 至少要能读到长度字节
            if len(self._buf) < 2:
                break

            # 3) 长度不符 -> head_err，丢一字节重扫
            if self._buf[1] != UPLINK_FRAME_SIZE:
                self.stats.head_err += 1
                del self._buf[0]
                continue

            # 4) 等够一整帧
            if len(self._buf) < UPLINK_FRAME_SIZE:
                break

            # 5) 取帧并校验
            frame = bytes(self._buf[:UPLINK_FRAME_SIZE])
            parsed = parse_uplink_frame(frame)
            if parsed is None:
                self.stats.crc_err += 1
                del self._buf[0]          # 只丢一字节，重新同步
                continue

            del self._buf[:UPLINK_FRAME_SIZE]
            self.stats.frames += 1
            self._check_drop(parsed)
            out.append(parsed)

        return out

    def _check_drop(self, up: Uplink) -> None:
        """用 tick 差值检测丢帧（上行标称 50Hz，即 20ms 一帧）。"""
        if self._last_tick is not None:
            delta = (up.tick_ms - self._last_tick) & 0xFFFFFFFF
            if delta > 30:                 # 明显超过 20ms
                self.stats.dropped += max(0, delta // 20 - 1)
        self._last_tick = up.tick_ms


# --------------------------------------------------------------------------
# 下行解码器：固件接收侧的逐字镜像（用于交叉验证）
# --------------------------------------------------------------------------
#
# 这两个类刻意【逐字复刻】固件 HARDWARE/usartx.c 里 ros_rx_byte() /
# ros_sv_byte() 的行为，包括那些看起来奇怪的地方：
#   - state 0 看到非帧头字节时【不计数】（静默忽略）
#   - state 1 看到帧头时当作新帧头重启（不计数）
#   - CRC 失败后只丢一字节并重扫 0xAA，而不是丢掉整个窗口
#   - 舵机解码器没有 head_err 计数器，长度字节不符时计入 crc_err
#
# 复刻的目的不是"写得好看"，是让 Python 与 C 的输出可以逐字段严格比对。
# 任何"顺手优化"都会让对拍产生假告警。

class MotionRxParser:
    """下行运动帧（0xAA）解码器，镜像 ros_rx_byte()。"""

    SIZE = MOTION_FRAME_SIZE
    CRC_IDX = 10

    def __init__(self) -> None:
        self.buf = bytearray(self.SIZE)
        self.idx = 0
        self.state = 0
        self.frames = 0
        self.crc_err = 0
        self.head_err = 0
        # 最后一条成功解出的帧的【原始线值】，便于严格比对
        self.last = None      # (vx_i16, vy_i16, wz_i16, flags, seq)

    def reset(self) -> None:
        self.idx = 0
        self.state = 0

    def feed(self, data: bytes) -> int:
        for c in data:
            self._byte(c)
        return self.frames

    def _byte(self, c: int) -> None:
        if self.state == 0:
            if c == HDR_MOTION:
                self.buf[0] = c
                self.idx = 1
                self.state = 1

        elif self.state == 1:
            if c == MOTION_FRAME_SIZE:
                self.buf[1] = c
                self.idx = 2
                self.state = 2
            elif c == HDR_MOTION:
                self.buf[0] = c
                self.idx = 1
            else:
                self.head_err += 1
                self.reset()

        else:
            self.buf[self.idx] = c
            self.idx += 1
            if self.idx < self.SIZE:
                return
            crc = self.buf[self.CRC_IDX] | (self.buf[self.CRC_IDX + 1] << 8)
            if crc == crc16_modbus(bytes(self.buf[:self.CRC_IDX])):
                self._publish()
                self.reset()
            else:
                self.crc_err += 1
                self._resync()

    def _resync(self) -> None:
        k = 1
        while k < self.SIZE and self.buf[k] != HDR_MOTION:
            k += 1
        if k < self.SIZE:
            seg = bytes(self.buf[k:self.SIZE])
            self.buf[:len(seg)] = seg
            self.idx = len(seg)
            self.state = 1
        else:
            self.reset()

    def _publish(self) -> None:
        raw = struct.unpack("<hhhBB", bytes(self.buf[2:10]))
        self.last = raw
        self.frames += 1


class ServoRxParser:
    """下行舵机帧（0xAB）解码器，镜像 ros_sv_byte()。

    注意：固件这个解码器【没有 head_err 计数器】，长度字节不符时
    计入的是 crc_err。这是一个不对称之处，照实复刻。
    """

    SIZE = SERVO_FRAME_SIZE
    CRC_IDX = 15

    def __init__(self) -> None:
        self.buf = bytearray(self.SIZE)
        self.idx = 0
        self.state = 0
        self.frames = 0
        self.crc_err = 0
        self.last = None      # (s1..s6, seq)

    def reset(self) -> None:
        self.idx = 0
        self.state = 0

    def feed(self, data: bytes) -> int:
        for c in data:
            self._byte(c)
        return self.frames

    def _byte(self, c: int) -> None:
        if self.state == 0:
            if c == HDR_SERVO:
                self.buf[0] = c
                self.idx = 1
                self.state = 1

        elif self.state == 1:
            if c == SERVO_FRAME_SIZE:
                self.buf[1] = c
                self.idx = 2
                self.state = 2
            elif c == HDR_SERVO:
                self.buf[0] = c
                self.idx = 1
            else:
                self.crc_err += 1        # 注意：不是 head_err，照实复刻
                self.reset()

        else:
            self.buf[self.idx] = c
            self.idx += 1
            if self.idx < self.SIZE:
                return
            crc = self.buf[self.CRC_IDX] | (self.buf[self.CRC_IDX + 1] << 8)
            if crc == crc16_modbus(bytes(self.buf[:self.CRC_IDX])):
                self._publish()
                self.reset()
            else:
                self.crc_err += 1
                self._resync()

    def _resync(self) -> None:
        k = 1
        while k < self.SIZE and self.buf[k] != HDR_SERVO:
            k += 1
        if k < self.SIZE:
            seg = bytes(self.buf[k:self.SIZE])
            self.buf[:len(seg)] = seg
            self.idx = len(seg)
            self.state = 1
        else:
            self.reset()

    def _publish(self) -> None:
        us = struct.unpack("<6h", bytes(self.buf[2:14]))
        self.last = (us, self.buf[14])
        self.frames += 1


# --------------------------------------------------------------------------
# 离线自检
# --------------------------------------------------------------------------

if __name__ == "__main__":
    assert crc16_modbus(b"123456789") == CRC_CHECK_VECTOR, "CRC 校验向量不匹配！"

    # 与固件侧对齐的转换向量（严格相等）
    assert sat_i16(f32(f32(-170.0 * f32(0.01745329252)) * 10000.0)) == -29670
    assert sat_i16(f32(f32(-5.0 * f32(0.01745329252)) * 10000.0)) == -872
    assert sat_i16(f32(f32(10.0 * f32(0.01745329252)) * 10000.0)) == 1745

    # 帧长自检
    assert len(build_motion_frame(0.1, -0.02, 0.3)) == MOTION_FRAME_SIZE
    assert len(build_servo_frame([1500] * 6)) == SERVO_FRAME_SIZE

    print("protocol.py 自检通过")
    print(f"  CRC 向量       : 0x{crc16_modbus(b'123456789'):04X}")
    print(f"  舵机帧         : {build_servo_frame([1500]*6).hex(' ')}")
    print(f"  运动帧(vx=0.1) : {build_motion_frame(0.1, 0, 0).hex(' ')}")
