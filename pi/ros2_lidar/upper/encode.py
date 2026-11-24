"""upper 可视化服务的纯函数层。

不 import rclpy —— 这样数据编码、坐标变换可以在 Windows 上直接跑测试，
Pi 关机时也能验证前端。

口径说明：
  * OccupancyGrid 的 data 是 int8：-1 未知、0 空闲、100 占用。
    base64 传输时按无符号字节发（-1 -> 0xFF），页面再还原成有符号值。
  * 雷达点按 map 系发送。激光在车体后方 25.6 cm、左 0.65 cm（见
    urdf/robot.urdf），所以先把 base 位姿平移到 laser 原点，再按
    angle_min + i * angle_increment + yaw 投影。
"""

import base64
import math
import struct

# urdf/robot.urdf: laser 相对 base_link  x=-0.256  y=+0.0065（rpy=0）
LASER_X = -0.256
LASER_Y = 0.0065


def b64_bytes(raw):
    """bytes -> base64 str。"""
    return base64.b64encode(raw).decode("ascii")


def occupancy_payload(msg):
    """nav_msgs/OccupancyGrid -> SSE 'map' 事件的 dict。"""
    info = msg.info
    raw = bytes((int(v) & 0xFF) for v in msg.data)
    return {
        "w": int(info.width),
        "h": int(info.height),
        "res": float(info.resolution),
        "ox": float(info.origin.position.x),
        "oy": float(info.origin.position.y),
        "data": b64_bytes(raw),
    }


def yaw_from_quat(x, y, z, w):
    """四元数 -> 平面偏航角（rad）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def laser_origin(tx, ty, yaw):
    """base 位姿 -> 激光原点在 map 系的坐标。"""
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * LASER_X - s * LASER_Y, ty + s * LASER_X + c * LASER_Y


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max,
                   tx, ty, yaw, step=1):
    """LaserScan -> map 系点列。无效/超量程的回波直接丢掉。"""
    ox, oy = laser_origin(tx, ty, yaw)
    out = []
    for i in range(0, len(ranges), step):
        r = ranges[i]
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = yaw + angle_min + i * angle_increment
        out.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return out


def pack_points(points):
    """[(x, y), ...] -> base64(<ff 小端 float32 对)。页面按 Float32Array 读。"""
    buf = bytearray()
    for x, y in points:
        buf += struct.pack("<ff", x, y)
    return b64_bytes(bytes(buf))
