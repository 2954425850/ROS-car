#!/usr/bin/env python3
"""L150Pro 轮速环（固件内 PI）调参工具。

【为什么需要它】

固件里速度环的唯一两个增益 `Velocity_KP` / `Velocity_KI` 是**编译期常量**
（`BALANCE/system.c:79`，默认 700/700）。这台车换过电机、换过车身、轮径
65→125mm、载重也变了很多，**增益一次都没跟着调过**。

其中轮径那一项是可以直接算出来的，而且**同时进了环路增益的两端**：

    MOTOR_x.Encoder [m/s] = 计数 x 100Hz x Wheel_perimeter / Encoder_precision
                                           ^^^^^^^^^^^^^^^
    plant 增益 d(实测速度)/d(PWM) ∝ perimeter        -> x1.92
    误差信号 Bias = Target - Encoder 也 ∝ perimeter   -> x1.92
                                    等效环路增益     -> x1.92

也就是说**环路被动地热了将近一倍**。所以第一次试应该往**小**调，不是往大调。

【为什么是运行时改，而不是改源码】

`stmflash.c` 里那套 `Flash_Read()` / `Flash_Write()` 在本 build 里是**死代码**
（全树只有定义和声明，**没有任何调用点**）。所以：

  · 源码里的 700/700 就是生效值（不是被 flash 覆盖过的）
  · 运行时改的值**不会持久化** —— 掉电就回到 700/700

这正好是迭代想要的：随便试，掉电就复位。摸出好值之后再烘进 `system.c` 重烧一次。

【通道】

STM32 的 **USART2**（`PD5`=TX / `PD6`=RX），和 ROS 用的 USART1 是**分开的**，
互不干扰。需要一根 USB-TTL 接到树莓派上。

  ⚠️ 别接到 ROS 那根线上 —— 那是二进制协议，混进 ASCII 会把 CRC 打乱。

【帧格式】

来自官方文档《2.单片机与APP通信协议.pdf》，并与固件 `usartx.c` 的解析器逐字对上：

    { <选择符> : <十进制数字> }        ASCII
        选择符  '0' = RC_Velocity   '1' = Velocity_KP   '2' = Velocity_KI
        例：{1:350} 设 KP=350，字节序列 7B 31 3A 33 35 30 7D

    回读：{Q:P}  ->  固件把 PID_Send 置 1（判的是第 4 字节 == 'P'），然后
                     printf("{C<vel>:<kp>:<ki>}$") 从**同一个口**出来

⚠️ 固件 `Receive[]` 只有 50 字节且**不检查越界**，别发长帧。

【用法】

    端口用稳定符号名 /dev/l150pro-app（见 /etc/udev/rules.d/99-l150pro.rules，
    按转接板的 vid/pid 钉的）。别用 ttyUSB0 —— 编号会漂。

    # 1) 只读，不改任何东西
    python3 velocity_pid_tune.py --port /dev/l150pro-app read

    # 2) 设值（立即生效，掉电失效 —— 这正是迭代想要的）
    python3 velocity_pid_tune.py --port /dev/l150pro-app set kp 350
    python3 velocity_pid_tune.py --port /dev/l150pro-app set ki 350

    # 3) 阶跃测试（**会真的动车**，不加 --move 只是空跑打印）
    python3 velocity_pid_tune.py --port /dev/l150pro-app step --speed 0.4 --move

【调参顺序】

  1. `set ki 0`，只调 `kp`：加到场跃响应干脆、不持续振铃
  2. 再加 `ki`：加到稳态误差消失、且不出现超调
  3. 三个工况都要过一遍：**架空 / 落地空载 / 落地加载**
     （架空没负载，能推的增益比落地高；落地才是真实工况）
  4. 定下来的值烘进 `BALANCE/system.c:79`，重烧一次

【两个坑，调之前得知道】

  · 固件的公式 `Pwm += Kp*(e-e_prev) + Ki*e` **没有 dt**，增益是绑死在
    **100Hz** 上的。改控制频率就必须重算增益。
  · 欠压或 `EN==0` 时四个 PI 函数**根本不执行**（`balance.c:197` 那个分支），
    而 `Pwm` 是 `static` 且全树无复位 -> 冻在停机值上，恢复时从那儿继续。
    调参时反复扳使能开关会看到莫名其妙的行为，那不是增益问题。
"""

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    print("需要 pyserial：pip install pyserial", file=sys.stderr)
    sys.exit(1)

# 固件里三个可设的选择符
SELECTORS = {"vel": "0", "kp": "1", "ki": "2"}

# 回读格式：printf("{C%d:%d:%d}$", RC_Velocity, Velocity_KP, Velocity_KI)
READBACK_RE = re.compile(r"\{C(-?\d+):(-?\d+):(-?\d+)\}\$")

# 固件 Receive[] 是 50 字节且**没有边界检查**，发太长会溢出它。自作主张拦一下。
MAX_FRAME_BYTES = 40


def build_frame(selector: str, value: int) -> bytes:
    """{ <选择符> : <十进制> }，见模块头的帧格式说明。"""
    frame = "{%s:%d}" % (selector, int(value))
    if len(frame) > MAX_FRAME_BYTES:
        raise ValueError("帧太长（固件 Receive[] 只有 50 字节且不检查越界）")
    return frame.encode("ascii")


def build_readback_frame() -> bytes:
    """{ Q : P } —— 第 4 个字节是 'P'(0x50)，固件据此把 PID_Send 置 1。"""
    return b"{Q:P}"


def read_gains(ser, timeout=1.5):
    """发回读指令，解析 {C<vel>:<kp>:<ki>}$。返回 (vel, kp, ki) 或 None。

    ⚠️ 这个函数是**整条链路的试金石**：回读不通就说明通道/格式/波特率有问题，
    此时**不要**继续去 set —— 你并不知道自己改没改到东西。
    """
    ser.reset_input_buffer()
    ser.write(build_readback_frame())
    deadline = time.monotonic() + timeout
    buf = b""
    while time.monotonic() < deadline:
        buf += ser.read(64)
        m = READBACK_RE.search(buf.decode("ascii", "ignore"))
        if m:
            return tuple(int(g) for g in m.groups())
    return None


def set_gain(ser, which: str, value: int) -> None:
    ser.write(build_frame(SELECTORS[which], value))


# --------------------------------------------------------------------------
# 阶跃测试
# --------------------------------------------------------------------------

def cmd_step(args):
    """发一个 vx 阶跃，录轮速，报上升时间/超调/稳态误差/纹波。

    录的是 `wheel_odom.twist.linear.x` —— 那正是轮速的平均，也就是速度环
    在调的那个量（固件调的是每轮，我们看平均，够用）。

    不加 `--move` 时**不发任何指令**，但仍然会连上 ROS 检查采集链路 ——
    否则真跑起来才发现 wheel_odom 没数据，就白动一次车了。
    """
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    move = args.move
    rclpy.init()
    node = rclpy.create_node("velocity_pid_step")
    samples = []
    hold = []                     # 保持段的采样快照（不含回零），见下
    node.create_subscription(
        Odometry, "wheel_odom",
        lambda m: samples.append((time.monotonic(), m.twist.twist.linear.x)), 10)
    pub = node.create_publisher(Twist, "cmd_vel", 10) if move else None

    def drive(vx, secs, wz=0.0):
        # wz 默认 0：基线和回零这两段**必须**是纯直线，否则车会"停下平移
        # 但还在转"。只有保持段才用 --wz（弧线 = 转向负载）。
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        end = time.monotonic() + secs
        last_pub = 0.0
        # 发指令 20Hz 就够（驱动只是锁存最后一个值），但要**转得比它快**，
        # 否则采样率被 publish 拖到 17Hz 左右，上升时间就只剩两三个采样点。
        while time.monotonic() < end:
            now = time.monotonic()
            if pub is not None and now - last_pub >= 0.05:
                pub.publish(t)
                last_pub = now
            rclpy.spin_once(node, timeout_sec=0.004)

    try:
        if not move:
            print("== 空跑：不发任何指令、不动车 ==")
            print("   监听 wheel_odom（最多等 12s —— DDS 发现可能要几秒，"
                  "固定转 2 秒会误报『链路不通』）……")
            end = time.monotonic() + 12.0
            while len(samples) < 100 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            n = len(samples)
            if n < 10:
                print(f"!! 12 秒只收到 {n} 个 wheel_odom —— 采集链路不通，"
                      f"先别动车。查驱动在不在：pgrep -af l150pro_driver")
                return 1
            span = samples[-1][0] - samples[0][0]
            vs = [v for _, v in samples]
            print(f"   收到 {n} 个样本，实测采样率 ≈ {n / span:.1f} Hz"
                  f"（驱动 50Hz 上行；采样率只影响上升时间的分辨率）")
            print(f"   vx 范围 [{min(vs):+.4f}, {max(vs):+.4f}] m/s"
                  f"  —— 车没动的话应该贴着 0")
            print("\n   采集链路 OK。真要测请加 --move：")
            print(f"   {args.speed} m/s 保持 {args.hold}s 再回零，"
                  f"总共约 {args.hold + 0.9:.1f}s")
            print("   测之前确认：场地清空、车在实地（不是架起来）、手能随时断电")
            return 0

        # 先等 DDS 发现完成再开始。发现可能要好几秒，这期间订阅是
        # match 不上的 —— 只靠 0.5s 基线等不到，阶跃的上升沿会被整个吃掉
        # （踩过：2 秒只收到 4 个样本，指标全废）。**车还没动，可以安全等。**
        print("等 DDS 发现 wheel_odom（车还没动）……")
        deadline = time.monotonic() + 15.0
        while len(samples) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        if len(samples) < 5:
            print("!! 15 秒没等到 wheel_odom，中止（车没动）。"
                  "查驱动：pgrep -af l150pro_driver")
            return 1
        print(f"   收到 {len(samples)} 个样本，发现已完成")

        print("静置 0.5s 采基线……")
        drive(0.0, 0.5)
        samples.clear()
        print(f"阶跃 -> {args.speed} m/s"
              + (f" 且 wz={args.wz:+.2f} rad/s（弧线）" if args.wz else "")
              + f"，保持 {args.hold}s")
        drive(args.speed, args.hold, args.wz)
        # ⚠️ 到此为止。后面回零那段的采样**不能**算进稳态 ——
        # 否则"末 N 个样本"正好落在减速阶段，会算出假的稳态误差和纹波
        # （踩过：把 0.30 的稳态报成 0.163、纹波报成 131.9%）。
        hold = list(samples)
        print("回零")
        drive(0.0, 0.4)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(hold) < 20:
        print(f"!! 样本太少（{len(hold)} 个），wheel_odom 没在发？")
        return 1

    report(hold, args.speed)
    return 0


def report(samples, target):
    """上升时间 / 超调 / 稳态误差 / 纹波。"""
    t0 = samples[0][0]
    rel = [((t - t0), v) for t, v in samples]

    # 阶跃时刻 = 第一个明显离开 0 的样本
    t_step = next((t for t, v in rel if abs(v) > 0.1 * abs(target)), rel[0][0])
    after = [(t, v) for t, v in rel if t >= t_step]
    if not after:
        print("!! 没有阶跃后的样本")
        return

    hold = [v for t, v in after if t <= t_step + 0.8]
    final_v = sum(v for _, v in after[-15:]) / min(15, len(after))
    peak = max((abs(v) for _, v in after), default=0.0)

    print("\n" + "=" * 56)
    print(f"  指令          {target:+.3f} m/s")

    # 先判"到底稳没稳"。没稳的话下面那个"稳态值"是爬升途中的值，
    # 会算出荒谬的稳态误差（踩过：Ki=200 时上升要 1.3s，2s 保持段里
    # 还在爬，报出"稳态 0.431 / 误差 +43.7%"，其实根本没有稳态）。
    last = [v for _, v in after[-24:]]
    unsettled = False
    if len(last) >= 8:
        h = len(last) // 2
        a1, a2 = sum(last[:h]) / h, sum(last[h:]) / (len(last) - h)
        drift = abs(a2 - a1) / abs(target) * 100 if target else 0.0
        unsettled = drift > 5.0
        if unsettled:
            print(f"  ⚠️  末段还在漂（半段间 {drift:.1f}%）—— **没有稳态**。"
                  f"下面的“稳态值”是爬升途中的值，不可用。")
            print(f"      处理：加长保持段重测（--hold 5），或把 Ki 加回去")

    print(f"  稳态值        {final_v:+.3f} m/s" + ("   (不可信)" if unsettled else ""))
    err = (final_v - target) / target * 100 if target else 0.0
    label = "   (不可信)" if unsettled else "   <- 有积分器还差这么多，就是没到位"
    print(f"  稳态误差      {err:+.1f}%{label}")

    # 上升时间（到 90% 指令值）
    t90 = next((t for t, v in after if abs(v) >= 0.9 * abs(target)), None)
    if t90 is not None:
        print(f"  上升时间(90%) {t90 - t_step:.3f} s")
    else:
        print(f"  上升时间(90%) 没到 90% —— 到不了，或者太慢")

    over = (peak / abs(target) - 1) * 100 if target else 0.0
    print(f"  峰值          {peak:.3f} m/s  (超调 {over:+.1f}%)")

    # 纹波/振荡**只看保持段后半**。从 t_step+0.4 起算会把上升阶段的采样
    # 也算进去，报出和超调自相矛盾的数（踩过：超调 +6.9% 却报纹波 39.9%）。
    half = after[len(after) // 2:]
    tail = [v for _, v in half]
    if len(tail) > 6:
        ripple = (max(tail) - min(tail)) / abs(target) * 100 if target else 0.0
        print(f"  保持段纹波    {ripple:.1f}%（后半段极差）"
              f"  <- 大=持续振荡")
        cross = sum(1 for a, b in zip(tail, tail[1:])
                    if (a - final_v) * (b - final_v) < 0)
        span = half[-1][0] - half[0][0]
        if cross >= 2 and span > 0:
            print(f"  振荡          ≈ {2 * span / cross:.2f} s 周期"
                  f"（后半段穿越稳态值 {cross} 次；周期跟采样率同量级"
                  f"就只是在数噪声）")
    else:
        print("  保持段纹波    样本太少，跳过")

    print("=" * 56)
    print("  稳不住/有稳态误差 -> 加 Ki；超调大、来回摆 -> 减 Kp")


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="L150Pro 轮速环调参（走 STM32 的 USART2，不是 ROS 那根线）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", required=True,
                    help="USB-TTL 的设备名，用 ls /dev/serial/by-id/ 确认")
    ap.add_argument("--baud", type=int, default=115200,
                    help="默认 115200（与 USART2 初始化一致）")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("read", help="读回当前增益（先跑这个验证通道）")

    p_set = sub.add_parser("set", help="设一个增益（立即生效，掉电失效）")
    p_set.add_argument("which", choices=sorted(SELECTORS))
    p_set.add_argument("value", type=int)

    p_step = sub.add_parser("step", help="阶跃测试（会动车，除非不加 --move）")
    p_step.add_argument("--speed", type=float, default=0.4)
    p_step.add_argument("--hold", type=float, default=2.0)
    p_step.add_argument("--wz", type=float, default=0.0,
                        help="同时给偏航角速度 -> 走弧线。转向时四轮互相较劲，"
                             "是速度环负载最重的工况（当初那个 20%% 亏就是"
                             "在弧线下测到的），复验必须带上它。单位 rad/s")
    p_step.add_argument("--move", action="store_true",
                        help="**真的发指令动车**。不加就是空跑")

    a = ap.parse_args()

    if a.cmd == "step":
        sys.exit(cmd_step(a))

    ser = serial.Serial(a.port, a.baud, timeout=0.2)
    try:
        if a.cmd == "read":
            got = read_gains(ser)
            if got is None:
                print("!! 没回读到 {C<vel>:<kp>:<ki>}$")
                print("   别急着往下 set —— 现在你并不知道改没改到东西。检查：")
                print("     1. 接的是不是 USART2（PD5=TX / PD6=RX），不是 ROS 那根")
                print("     2. 波特率是不是 115200")
                print("     3. TX/RX 有没有接反、有没有共地")
                print("     4. 帧格式我是从解析器反推的，没实测过 —— "
                      "必要时用串口助手手动发 {0#P} 看一眼")
                return 1
            vel, kp, ki = got
            print(f"  RC_Velocity = {vel}")
            print(f"  Velocity_KP = {kp}")
            print(f"  Velocity_KI = {ki}")
            print("  （注意：这两个值只活在 RAM 里，掉电回 700/700）")
            return 0

        if a.cmd == "set":
            before = read_gains(ser)
            if before is None:
                print("!! 设值前回读不到。**先跑 read 把通道打通** —— "
                      "现在改没改到东西是无法确认的。")
                return 1
            set_gain(ser, a.which, a.value)
            time.sleep(0.2)
            after = read_gains(ser)
            if after is None:
                print("!! 设完回读不到，无法确认。")
                return 1
            fmt = "  set {:<3} {:<5} 之前 {} -> 之后 {}"
            print(fmt.format(a.which, a.value, before, after))
            if after == before and a.value not in before:
                print("   ⚠️ 值没变！帧可能没被固件接受 —— "
                      "核对帧格式与选择符")
            return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main() or 0)
