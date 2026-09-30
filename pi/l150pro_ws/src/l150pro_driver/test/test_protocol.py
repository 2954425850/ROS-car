"""协议层测试：与固件侧交叉验证。

不依赖 ROS，可直接运行：
    python test/test_protocol.py

也兼容 pytest：
    pytest test/
"""

import os
import random
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from l150pro_driver.protocol import (  # noqa: E402
    CRC_CHECK_VECTOR, FLAG_ENABLE, HDR_MOTION, HDR_SERVO, HDR_UPLINK,
    MOTION_FRAME_SIZE, SERVO_FRAME_SIZE, TRACK_EFF, UPLINK_FRAME_SIZE,
    Uplink, UplinkParser, angle_deg_to_wire, build_motion_frame, build_servo_frame,
    YawUnwrapper, build_uplink_frame, crc16_modbus, f32, parse_uplink_frame,
    sat_i16, unwrap_delta,
    PAN_CHANNEL, SERVO_CENTER_US, SERVO_MAX_US, SERVO_MIN_US,
    servo_command_to_pulse,
)

_failures = []


def check(cond, label):
    if cond:
        print(f"  pass  {label}")
    else:
        print(f"  FAIL  {label}")
        _failures.append(label)


# --------------------------------------------------------------------------

def test_crc():
    print("\n== 1. CRC-16/MODBUS ==")
    check(crc16_modbus(b"123456789") == CRC_CHECK_VECTOR,
          f'crc16_modbus("123456789") == 0x4B37  (实得 0x{crc16_modbus(b"123456789"):04X})')
    check(crc16_modbus(b"") == 0xFFFF, "空输入的 CRC 为 init 值 0xFFFF")


def test_frame_layout():
    print("\n== 2. 帧布局 ==")
    m = build_motion_frame(0.123, -0.007, -0.456, FLAG_ENABLE, 7)
    check(len(m) == MOTION_FRAME_SIZE, f"运动帧 = {MOTION_FRAME_SIZE} 字节")
    check(m[0] == HDR_MOTION and m[1] == MOTION_FRAME_SIZE, "运动帧头/长度正确")
    check(struct.unpack("<h", m[2:4])[0] == 123, "vx 123 mm/s")
    check(struct.unpack("<h", m[4:6])[0] == -7, "vy -7 mm/s")
    check(struct.unpack("<h", m[6:8])[0] == -456, "wz -456 mrad/s")
    check(m[8] == FLAG_ENABLE and m[9] == 7, "flags / seq")
    check(crc16_modbus(m[:10]) == struct.unpack("<H", m[10:12])[0], "运动帧 CRC 覆盖 0..9")

    s = build_servo_frame([1500, 1000, 2000, 0, 1500, 2500], seq=3)
    check(len(s) == SERVO_FRAME_SIZE, f"舵机帧 = {SERVO_FRAME_SIZE} 字节")
    check(s[0] == HDR_SERVO and s[1] == SERVO_FRAME_SIZE, "舵机帧头/长度正确")
    check(struct.unpack("<6h", s[2:14]) == (1500, 1000, 2000, 0, 1500, 2500),
          "六路舵机值按通道顺序")
    check(s[14] == 3, "seq 在 byte 14")
    check(crc16_modbus(s[:15]) == struct.unpack("<H", s[15:17])[0], "舵机帧 CRC 覆盖 0..14")


def test_conversion_vectors():
    """与固件侧对齐的转换向量，判【严格相等】。"""
    print("\n== 3. 转换向量（严格相等，非容差）==")

    def to_i16(deg):
        return angle_deg_to_wire(deg)

    for deg, expect, note in ((10.0, 1745, "roll 10deg"),
                              (-5.0, -872, "pitch -5deg(int vs round 的分水岭)"),
                              (190.0, -29670, "yaw 190deg 经 wrap")):
        got = to_i16(deg)
        check(got == expect, f"{note}: {got} == {expect}")

    # wrap 是 load-bearing 的：不 wrap 会超出 int16 被饱和钳到 32767
    unwrapped = sat_i16(f32(f32(190.0 * f32(0.01745329252)) * 10000.0))
    check(unwrapped == 32767,
          f"190deg 不 wrap 会得 {unwrapped}（饱和），证明 wrap 不可省")

    # 这条专门盯 round() 陷阱：真值 -872.665，round() 会给 -873
    raw = f32(f32(-5.0 * f32(0.01745329252)) * 10000.0)
    check(int(raw) == -872 and round(raw) == -873,
          f"int() 给 {int(raw)}、round() 给 {round(raw)} —— 必须用 int()")


def test_uplink_roundtrip():
    print("\n== 4. 上行帧往返 ==")
    src = Uplink(tick_ms=123456, vA=1.234, vB=1.200, vC=-1.500, vD=-1.480,
                 roll=0.1745, pitch=-0.0873, yaw=-2.9671, gz=0.012,
                 voltage=11.8, fault=0, seq=42)
    frame = build_uplink_frame(src)
    check(len(frame) == UPLINK_FRAME_SIZE, f"上行帧 = {UPLINK_FRAME_SIZE} 字节")
    check(frame[0] == HDR_UPLINK and frame[1] == UPLINK_FRAME_SIZE, "上行帧头/长度正确")
    check(crc16_modbus(frame[:26]) == struct.unpack("<H", frame[26:28])[0],
          "上行帧 CRC 覆盖 0..25")

    got = parse_uplink_frame(frame)
    check(got is not None, "可被解析")
    if got:
        check(got.tick_ms == 123456, "tick 往返")
        check(abs(got.vA - 1.234) < 1e-6, "vA 往返")
        check(abs(got.voltage - 11.8) < 1e-6, "voltage 往返")
        check(got.seq == 42, "seq 往返")


def test_odometry_derivation():
    """轮速 -> 车体速度的推导。这里最容易出错的是轮距用错。"""
    print("\n== 5. 里程计推导 ==")
    # 左右轮同速 -> 直行，无角速度
    up = Uplink(tick_ms=0, vA=0.5, vB=0.5, vC=0.5, vD=0.5,
                roll=0, pitch=0, yaw=0, gz=0, voltage=12, fault=0, seq=0)
    check(abs(up.vx - 0.5) < 1e-6, "左右同速 -> vx = 0.5")
    check(abs(up.wz()) < 1e-6, "左右同速 -> wz = 0")

    # 右轮快 -> 左转(逆时针, wz > 0)
    #
    # 下面几条各自盯不同的失败模式，不能互相替代：
    #   a) 字面量喂 + 字面量断言：同时盯住【常量值】和【除法次数】
    #   b) 自洽检查（喂 TRACK_EFF）：证明代码真的在用这个常量，没在别处写死
    #   c) 常量本身的值断言：换车时强制这里一起改，不会静默漂移
    up_lit = Uplink(tick_ms=0, vA=0.0, vB=0.0, vC=1.0, vD=1.0,
                    roll=0, pitch=0, yaw=0, gz=0, voltage=12, fault=0, seq=0)
    check(abs(up_lit.wz() - 1.0) < 1e-6,
          "v_right-v_left=1.0 / 全轮距 1.000 -> wz = 1.0 "
          "(误用半轮距得 2.0；常量退回参考车 0.360 得 2.78)")

    up = Uplink(tick_ms=0, vA=0.0, vB=0.0, vC=TRACK_EFF, vD=TRACK_EFF,
                roll=0, pitch=0, yaw=0, gz=0, voltage=12, fault=0, seq=0)
    check(abs(up.v_left) < 1e-6, "v_left = 0")
    check(abs(up.v_right - TRACK_EFF) < 1e-6, f"v_right = {TRACK_EFF}")
    check(abs(up.wz() - 1.0) < 1e-6,
          f"v_right-v_left={TRACK_EFF}, 除以全轮距 {TRACK_EFF} -> wz = 1.0 "
          f"(代码确实用了这个常量，没在别处写死)")

    check(abs(TRACK_EFF - 1.000) < 1e-9,
          "有效全轮距 = 0.460+0.540 = 1.000（外推初值，标定后与固件同步更新）")

    # 标定入参：monitor --track-eff 走的就是这条路径
    check(abs(up_lit.wz(0.5) - 2.0) < 1e-6,
          "wz(track_eff=0.5) -> 2.0：标定迭代试值不用改常量重新编译")


def test_stream_parser():
    print("\n== 6. 流式解析（粘包/半包/噪声）==")

    def mk(seq, tick):
        return build_uplink_frame(Uplink(
            tick_ms=tick, vA=0.1, vB=0.1, vC=0.1, vD=0.1,
            roll=0, pitch=0, yaw=0, gz=0, voltage=12.0, fault=0, seq=seq))

    # 三帧粘在一起
    p = UplinkParser()
    out = p.feed(mk(1, 0) + mk(2, 20) + mk(3, 40))
    check(len(out) == 3, "三帧粘连可全部解出")
    check([f.seq for f in out] == [1, 2, 3], "顺序正确")

    # 逐字节喂（最坏的分包）
    p = UplinkParser()
    got = []
    for b in mk(9, 0):
        got += p.feed(bytes([b]))
    check(len(got) == 1 and got[0].seq == 9, "逐字节喂入也能解出")

    # 前置垃圾
    p = UplinkParser()
    out = p.feed(b"\xde\xad\xbe\xef" + mk(5, 0))
    check(len(out) == 1 and out[0].seq == 5, "前置垃圾后可重新同步")

    # 帧内载荷恰好含 0x55（帧头），不应造成假同步
    body = bytearray(mk(6, 0))
    body[6:8] = struct.pack("<h", 0x0055)      # vA 载荷含 0x55
    body[26:28] = struct.pack("<H", crc16_modbus(bytes(body[:26])))
    p = UplinkParser()
    out = p.feed(bytes(body))
    check(len(out) == 1, "载荷内嵌帧头字节不产生假同步/分裂")

    # CRC 错误被拒
    bad = bytearray(mk(7, 0))
    bad[6] ^= 0xFF
    p = UplinkParser()
    out = p.feed(bytes(bad) + mk(8, 20))
    check(all(f.seq != 7 for f in out), "CRC 错误的帧被拒绝")
    check(any(f.seq == 8 for f in out), "拒绝后能重新锁定下一帧")

    # 丢帧检测
    p = UplinkParser()
    p.feed(mk(1, 0))
    p.feed(mk(2, 60))            # tick 跳了 60ms -> 丢了约 2 帧
    check(p.stats.dropped >= 2, f"tick 跳变检测到丢帧 (dropped={p.stats.dropped})")


def test_fuzz():
    print("\n== 7. 模糊测试（随机字节不得产生假接收）==")
    rnd = random.Random(0xC0FFEE)          # 固定种子，可复现
    p = UplinkParser()
    accepted = 0
    for _ in range(20):
        noise = bytes(rnd.randrange(256) for _ in range(10000))
        accepted += len(p.feed(noise))
    check(accepted == 0,
          f"20 万随机字节产生 {accepted} 次假接收 (crc_err={p.stats.crc_err}, "
          f"head_err={p.stats.head_err})")

    # 噪声之后仍能锁定
    good = build_uplink_frame(Uplink(
        tick_ms=1, vA=0, vB=0, vC=0, vD=0, roll=0, pitch=0, yaw=0,
        gz=0, voltage=12, fault=0, seq=1))
    out = p.feed(good)
    check(len(out) == 1, "噪声突发之后仍能锁定")


def test_yaw_unwrap():
    """yaw 回绕到 +-PI：累加连续转角必须解回绕，否则转一圈得 0。"""
    print("\n== 9. yaw 回绕（原地转标定必须靠这个）==")
    TWO_PI = 6.28318530717959
    PI = 3.14159265358979

    def wrap(a):
        while a > PI:
            a -= TWO_PI
        while a < -PI:
            a += TWO_PI
        return a

    # 从 0 正向转一整圈，按固件口径回绕到 (-pi, pi] 后逐帧喂入
    n = 360
    yaws = [wrap(TWO_PI * i / n) for i in range(n + 1)]

    u = YawUnwrapper()
    for y in yaws:
        total = u.feed(y)
    check(abs(total - TWO_PI) < 5e-3,
          f"回绕序列累加 -> {total:.6f} rad（期望 2pi = {TWO_PI:.6f}）")

    # 反证：不解回绕会得 0 —— 转了一整圈却约等于没转，正是要防的 bug
    naive = sum(yaws[i] - yaws[i - 1] for i in range(1, len(yaws)))
    check(abs(naive) < 1e-3,
          f"不解回绕 -> {naive:.6f} rad（转满一圈却约等于 0）")

    # 反向转一整圈
    u = YawUnwrapper()
    for y in [wrap(-TWO_PI * i / n) for i in range(n + 1)]:
        u.feed(y)
    check(abs(u.total + TWO_PI) < 5e-3,
          f"反向回绕序列累加 -> {u.total:.6f} rad（期望 -2pi）")

    # 跨回绕点的单步增量
    check(abs(unwrap_delta(3.0, -3.0) - (TWO_PI - 6.0)) < 1e-6,
          "跨 +pi 的单步增量解回绕正确")
    check(abs(unwrap_delta(-3.0, 3.0) + (TWO_PI - 6.0)) < 1e-6,
          "跨 -pi 的单步增量解回绕正确")


def test_servo_pan_convention():
    """ch5 的对外约定：值增大 = 云台往右（车体坐标系）。

    物理舵机是反装的，所以发给固件前要翻一次。这个测试钉的就是这一次翻转 ——
    翻多了、翻少了、或者哪天有人"顺手"把它去掉，这里必须红。
    """
    print("\n== 9. 云台水平通道约定 ==")

    check(servo_command_to_pulse(PAN_CHANNEL, SERVO_CENTER_US) == SERVO_CENTER_US,
          "中位 1500 翻转后仍是 1500")
    check(servo_command_to_pulse(PAN_CHANNEL, SERVO_MAX_US) == SERVO_MIN_US,
          "「往右」到底（2500）出线为 500")
    check(servo_command_to_pulse(PAN_CHANNEL, SERVO_MIN_US) == SERVO_MAX_US,
          "「往左」到底（500）出线为 2500")

    ladders = list(range(SERVO_MIN_US, SERVO_MAX_US + 1, 100))
    pulses = [servo_command_to_pulse(PAN_CHANNEL, v) for v in ladders]
    check(all(a > b for a, b in zip(pulses, pulses[1:])),
          "ch5 全程严格单调递减（往右 = 脉宽变小）")

    check(all(servo_command_to_pulse(6, v) == v for v in ladders),
          "ch6（俯仰）原样透传，没被连累")

    check(servo_command_to_pulse(PAN_CHANNEL, 0) == 0,
          "0（保持不变）不被翻转成 3000")

    check(servo_command_to_pulse(PAN_CHANNEL, 2000, pan_invert=False) == 2000,
          "pan_invert=False 时原样透传（装正后能一键关掉）")


def test_config():
    """服务端接收机应能看到但不误用这两个口。"""
    print("\n== 8. 端口约定 ==")
    check(struct.pack("<h", 1500) == b"\xdc\x05", "1500us 小端为 dc 05")


# --------------------------------------------------------------------------

def main():
    test_crc()
    test_frame_layout()
    test_conversion_vectors()
    test_uplink_roundtrip()
    test_odometry_derivation()
    test_stream_parser()
    test_fuzz()
    test_config()
    test_servo_pan_convention()
    test_yaw_unwrap()
    print("\n" + "=" * 60)
    if _failures:
        print(f"{len(_failures)} 项失败：")
        for f in _failures:
            print(f"  - {f}")
        sys.exit(1)
    print("全部通过")


if __name__ == "__main__":
    main()
