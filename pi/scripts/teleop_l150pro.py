#!/usr/bin/env python3
"""L150Pro 键盘遥控：底盘 + 云台。

用法:
    python3 ~/teleop_l150pro.py

前置（另开一个终端先把驱动跑起来，否则发了也没人收）:
    source /opt/ros/jazzy/setup.bash
    source ~/l150pro_ws/install/setup.bash
    ros2 launch l150pro_driver l150pro.launch.py port:=/dev/l150pro use_ekf:=false

按键（在程序里按 h 可再打一次这份帮助）:
  底盘（四驱差速，没有横移；a/d 是原地转向不是平移）
    w 前进        s 后退
    a 原地左转    d 原地右转
    q 左前弧      e 右前弧
    z 左后弧      c 右后弧
    空格  急停（底盘归零；云台不动）
  云台
    ← / j  水平左        → / l  水平右
    ↑ / i  俯仰上        ↓ / k  俯仰下
    G  云台回中位 1500us
  其他
    + / -  调速（步进 0.05，范围 0.05~0.50 m/s）    r  恢复默认速度
    m  切换「点按锁定 / 按住才动」
    h  帮助            x 或 Ctrl-C  退出

两种模式:
    锁定(默认)  按一下就一直走，直到按空格或换方向。不依赖键盘重复，不会顿挫。
    按住        松开键约 0.5s 后底盘自动归零。更"跟手"，但键盘重复有 ~500ms
                初始延迟，所以每次按住的头半秒会停着 —— 这是键盘的锅，不是程序的。

安全约定:
    · 本程序只驱动 ch5/ch6，另外四路一律发 0（固件约定 0 = 该路保持不动），
      不会碰底盘上其它舵机。
    · 云台没有看门狗：本程序退出后云台保持原位，不会乱甩（固件设计如此）。
    · 退出时发一帧零速，并留一个 0.3s 的余量让驱动把帧发出去。
    · 底盘有 200ms 丢帧看门狗，本程序按 25Hz 持续发帧，不会触发。
    · ⚠️ 里程计还没标定，实际车速会明显大于指令值。第一次测就用默认的
      0.15 m/s，别急着按 + 。
    · ⚠️ 上路前把车架起来（轮子悬空）先试一遍按键，确认方向对得上。

标定前先看: ~/l150pro_ws/src/l150pro_driver/README.md 的「标定」一节。
"""

import argparse
import os
import select
import sys
import termios
import threading
import time


# ---- 自动加载 ROS 环境：没 source 就自己重跑一遍 ----
def _ensure_ros():
    if "--dry-run" in sys.argv or "--keys" in sys.argv:
        return  # 演练模式不需要 ROS
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_L150PRO_ROS") == "1":
        sys.stderr.write(
            "错误：加载 ROS 2 环境失败。请手动执行：\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~/l150pro_ws/install/setup.bash\n")
        sys.exit(1)
    os.environ["_L150PRO_ROS"] = "1"
    cmd = ('source /opt/ros/jazzy/setup.bash; '
           'source "$HOME/l150pro_ws/install/setup.bash" 2>/dev/null; '
           'exec python3 "$0" "$@"')
    os.execvp("bash", ["bash", "-c", cmd, sys.argv[0]] + sys.argv[1:])


_ensure_ros()

SERVO_CHANNELS = 6
SERVO_HOLD = 0        # 固件约定：0 = 该路保持不动
SERVO_CENTER_US = 1500

# 方向键的转义序列尾字节 -> 键名
ARROWS = {b"A": "up", b"B": "down", b"C": "right", b"D": "left"}

# 每个底盘动作 = (前进方向, 转向方向)，两者都是 -1/0/+1
# 角速度为正 = 逆时针 = 左转（REP-103）
MOTION_KEYS = {
    "w": (+1, 0), "s": (-1, 0),
    "a": (0, +1), "d": (0, -1),
    "q": (+1, +1), "e": (+1, -1),
    "z": (-1, +1), "c": (-1, -1),
}

GIMBAL_ALIAS = {"j": "left", "l": "right", "i": "up", "k": "down"}


def _us_range(s):
    try:
        lo, hi = s.split(":")
        lo, hi = int(lo), int(hi)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"格式应为 下限:上限，例如 800:2200，收到 {s!r}")
    if not 500 <= lo < hi <= 2500:
        raise argparse.ArgumentTypeError(
            f"范围必须在 500~2500 内且下限小于上限，收到 {s!r}")
    return lo, hi


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="L150Pro 键盘遥控（底盘 + 云台）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pan-ch", type=int, default=5,
                   help="云台【水平】舵机通道，默认 5 (PB14)")
    p.add_argument("--tilt-ch", type=int, default=6,
                   help="云台【俯仰】舵机通道，默认 6 (PB15)")
    p.add_argument("--pan-range", type=_us_range, default=(800, 2200),
                   help="水平轴行程 us，默认 800:2200（保守值，防堵转扫齿）")
    p.add_argument("--tilt-range", type=_us_range, default=(800, 2200),
                   help="俯仰轴行程 us，默认 800:2200")
    p.add_argument("--pan-invert", action="store_true",
                   help="若 ← 让云台往右转，加这个反转水平轴")
    p.add_argument("--tilt-invert", action="store_true",
                   help="若 ↑ 让云台往下走，加这个反转俯仰轴")
    p.add_argument("--servo-step", type=int, default=20,
                   help="云台每次按键的脉宽步进 us，默认 20（约 1.8°）")
    p.add_argument("--center-on-start", action="store_true",
                   help="启动时就把云台压回 1500us。默认不动云台，"
                        "等第一次按云台键才发命令")
    p.add_argument("--speed", type=float, default=0.15,
                   help="默认前进速度 m/s，默认 0.15（标定前别调大）")
    p.add_argument("--speed-step", type=float, default=0.05,
                   help="+/- 调速步进，默认 0.05")
    p.add_argument("--turn-gain", type=float, default=4.0,
                   help="原地转向角速度 = 速度 x 该系数，默认 4.0")
    p.add_argument("--arc-gain", type=float, default=0.5,
                   help="弧线转向速率相对原地转向的倍数，默认 0.5")
    p.add_argument("--max-wz", type=float, default=1.5,
                   help="角速度上限 rad/s，默认 1.5（与驱动 max_wz 一致）")
    p.add_argument("--mode", choices=("latch", "hold"), default="latch",
                   help="latch=点按锁定（默认）, hold=按住才动")
    p.add_argument("--hold-timeout", type=float, default=0.5,
                   help="hold 模式松开多久后归零，默认 0.5s")
    p.add_argument("--rate", type=float, default=25.0, help="发布频率 Hz，默认 25")
    p.add_argument("--dry-run", action="store_true",
                   help="不连 ROS、不发消息，只打印将要发布的内容。用来在不接车的"
                        "情况下验证按键映射")
    p.add_argument("--keys", default=None,
                   help="演练用：逗号分隔的按键序列，跑完即退出。例如 "
                        "\"w,left,left,space,m,w,x\"；空格键写作 space，"
                        "wait0.6 表示空等 0.6 秒。隐含 --dry-run")
    return p.parse_args(argv)


class Teleop:
    def __init__(self, args):
        self.a = args
        self.lock = threading.Lock()
        self.running = True

        # 底盘
        self.speed = args.speed
        self.default_speed = args.speed
        self.vx = 0.0
        self.wz = 0.0
        self.motion_key = None       # 当前生效的方向键，调速时按它重算
        self.deadline = 0.0          # hold 模式的归零时刻
        self.mode = args.mode

        # 云台：只记录我们要驱动的那两路，其余保持 0（不动）
        self.servo = [SERVO_HOLD] * SERVO_CHANNELS
        self.gimbal = {
            args.pan_ch: SERVO_CENTER_US,    # 假定值：固件上电即中位
            args.tilt_ch: SERVO_CENTER_US,
        }
        self.gimbal_rng = {
            args.pan_ch: args.pan_range,
            args.tilt_ch: args.tilt_range,
        }
        self.gimbal_sign = {
            args.pan_ch: -1 if args.pan_invert else +1,
            args.tilt_ch: -1 if args.tilt_invert else +1,
        }
        self.servo_dirty = False

        if args.center_on_start:
            for ch, us in self.gimbal.items():
                self.servo[ch - 1] = us
            self.servo_dirty = True

        # 发布器（dry-run 时为 None）
        self.pub_vel = self.pub_servo = self.node = None
        self._dry_last = None
        self.notices = []            # pump 线程塞、主线程打印

    # ==================================================================
    # 发布
    # ==================================================================

    def start_publishers(self):
        """非 dry-run 时由 main() 调用。"""
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import JointState

        self._Twist = Twist
        self._JointState = JointState
        self._rclpy = rclpy
        rclpy.init(args=[])          # 传空列表：别让 rclpy 去解析我们自己的命令行参数
        self.node = Node("l150pro_teleop")
        self.pub_vel = self.node.create_publisher(Twist, "cmd_vel", 10)
        self.pub_servo = self.node.create_publisher(JointState, "servo_cmd", 10)

    def _apply_motion(self):
        """按 motion_key 和当前速度档重算 vx/wz。调用方必须已持锁。"""
        if self.motion_key is None:
            self.vx = self.wz = 0.0
            return
        fwd, turn = MOTION_KEYS[self.motion_key]
        arc = turn != 0 and fwd != 0
        gain = self.a.turn_gain * (self.a.arc_gain if arc else 1.0)
        self.vx = fwd * self.speed
        self.wz = max(-self.a.max_wz, min(self.a.max_wz, turn * self.speed * gain))

    def pump(self):
        period = 1.0 / self.a.rate
        t0 = time.monotonic()
        link_checked = False
        while self.running:
            with self.lock:
                vx, wz = self.vx, self.wz
                if self.mode == "hold" and time.monotonic() > self.deadline:
                    vx = wz = 0.0
                dirty = self.servo_dirty
                self.servo_dirty = False
                servo = list(self.servo)

            if self.pub_vel is not None:
                tw = self._Twist()
                tw.linear.x = vx
                tw.angular.z = wz
                self.pub_vel.publish(tw)
                if dirty:
                    js = self._JointState()
                    js.header.stamp = self.node.get_clock().now().to_msg()
                    js.name = [f"ch{i}" for i in range(1, SERVO_CHANNELS + 1)]
                    js.position = [float(x) for x in servo]
                    self.pub_servo.publish(js)
            else:
                self._dry_pump(vx, wz, servo, dirty)

            # 起 2.5s 后查一次有没有人订阅，对应 README 排错表「车不动」的第一条
            if not link_checked and time.monotonic() - t0 > 2.5:
                link_checked = True
                if self.pub_vel is not None and \
                        self.pub_vel.get_subscription_count() == 0:
                    self.notices.append(
                        "⚠️  cmd_vel 上没有订阅者 —— 驱动节点没在跑？"
                        "另开一个终端执行：ros2 launch l150pro_driver "
                        "l150pro.launch.py port:=/dev/l150pro use_ekf:=false")
            time.sleep(period)

    def _dry_pump(self, vx, wz, servo, dirty):
        """演练模式：只在状态真的变化时打印，避免刷屏和按键前的旧快照。"""
        key = (round(vx, 4), round(wz, 4), tuple(servo) if dirty else None)
        if key == self._dry_last:
            return
        self._dry_last = key
        self._dry_line(vx, wz, servo, dirty)

    def _dry_line(self, vx, wz, servo, dirty):
        s = f"  cmd_vel  linear.x={vx:+.3f}  angular.z={wz:+.3f}"
        if dirty:
            s += f"   | servo_cmd  ch5={servo[4]}us  ch6={servo[5]}us"
        sys.stdout.write(s + "\n")
        sys.stdout.flush()

    def stop_now(self):
        """退出路径：发一帧零速。"""
        with self.lock:
            self.motion_key = None
            self._apply_motion()
        if self.pub_vel is None:
            return
        try:
            tw = self._Twist()
            tw.linear.x = 0.0
            tw.angular.z = 0.0
            for _ in range(8):                 # 0.3s 余量，确保驱动真发出去
                self.pub_vel.publish(tw)
                time.sleep(0.04)
        except Exception as e:                 # noqa: BLE001
            sys.stderr.write(f"退出时发零速失败：{e}\n")

    # ==================================================================
    # 按键处理
    # ==================================================================

    def handle(self, k):
        """处理一个按键。返回要显示的一句话，或 None。"""
        a = self.a
        with self.lock:
            # --- 底盘方向 ---
            if k in MOTION_KEYS:
                self.motion_key = k
                self._apply_motion()
                self.deadline = time.monotonic() + a.hold_timeout
                return None

            if k == " ":
                self.motion_key = None
                self._apply_motion()
                return "急停：底盘归零"

            # --- 云台 ---
            if k in GIMBAL_ALIAS:
                k = GIMBAL_ALIAS[k]
            if k in ("left", "right", "up", "down"):
                pan = a.pan_ch
                if k in ("left", "right"):
                    ch = pan
                    delta = (1 if k == "right" else -1) * self.gimbal_sign[ch]
                else:
                    ch = a.tilt_ch
                    delta = (1 if k == "up" else -1) * self.gimbal_sign[ch]
                lo, hi = self.gimbal_rng[ch]
                new = max(lo, min(hi, self.gimbal[ch] + delta * a.servo_step))
                if new == self.gimbal[ch]:
                    label = "水平" if ch == pan else "俯仰"
                    return f"云台{label} ch{ch} 已在限位 {new}us"
                self.gimbal[ch] = new
                self.servo[ch - 1] = new
                self.servo_dirty = True
                label = "水平" if ch == pan else "俯仰"
                hit = "（到限位了）" if new in (lo, hi) else ""
                return f"云台{label} ch{ch} -> {new}us{hit}"

            if k == "G":
                for ch in self.gimbal:
                    self.gimbal[ch] = SERVO_CENTER_US
                    self.servo[ch - 1] = SERVO_CENTER_US
                self.servo_dirty = True
                return f"云台回中位 {SERVO_CENTER_US}us"

            # --- 调速：按完立刻在当前方向上生效，不用再按一次方向键 ---
            if k in ("+", "=", "-", "_"):
                step = a.speed_step if k in ("+", "=") else -a.speed_step
                new = max(0.05, min(0.50, round(self.speed + step, 3)))
                if new == self.speed:
                    return f"速度档 {new:.2f} m/s 已是上下限"
                self.speed = new
                self._apply_motion()
                self.deadline = time.monotonic() + a.hold_timeout
                return f"速度档 -> {self.speed:.2f} m/s"
            if k == "r":
                self.speed = self.default_speed
                self._apply_motion()
                return f"速度档已复位 -> {self.speed:.2f} m/s"

            # --- 模式 ---
            if k == "m":
                self.mode = "hold" if self.mode == "latch" else "latch"
                self.deadline = time.monotonic() + a.hold_timeout
                return ("模式 -> 点按锁定（按一下就一直走，空格停）"
                        if self.mode == "latch" else
                        f"模式 -> 按住才动（松开 {a.hold_timeout:g}s 归零）")

            if k == "h":
                return None          # 帮助由 main 循环单独打印
            if k == "x":
                self.running = False
                return None
        return None

    # ==================================================================
    # 显示
    # ==================================================================

    def status(self):
        with self.lock:
            vx, wz, mode, speed = self.vx, self.wz, self.mode, self.speed
            if mode == "hold" and time.monotonic() > self.deadline:
                vx = wz = 0.0
            g = (f"水平={self.gimbal[self.a.pan_ch]}"
                 f" 俯仰={self.gimbal[self.a.tilt_ch]}")
        tag = "锁定" if mode == "latch" else "按住"
        return (f"[{tag}] vx={vx:+.3f} m/s  wz={wz:+.3f} rad/s  "
                f"档={speed:.2f} | 云台 {g} | h=帮助 m=模式 x=退出")


def read_key(timeout):
    """非阻塞读一键。方向键会规范化成 left/right/up/down。"""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    fd = sys.stdin.fileno()
    b = os.read(fd, 1)
    if not b:
        return None
    if b == b"\x1b":
        seq = b""
        while len(seq) < 2:
            r2, _, _ = select.select([sys.stdin], [], [], 0.03)
            if not r2:
                break
            c = os.read(fd, 1)
            if not c:
                break
            seq += c
        if len(seq) == 2 and seq[0:1] in (b"[", b"O"):
            return ARROWS.get(seq[1:2])
        return None      # 孤立的 ESC，忽略（别拿它当退出，SSH 下容易误触）
    return b.decode("utf-8", "ignore")


SHORT_HELP = """\
按 h 看完整帮助。常用键：
  w/s 前进后退    a/d 原地左右转    q/e/z/c 四个弧线    空格 急停
  <-/->/j/l 云台水平   ^/v/i/k 云台俯仰    G 云台回中
  +/- 调速        m 切换锁定/按住   x 退出
"""


def main():
    args = parse_args(sys.argv[1:])
    dry = args.dry_run or args.keys is not None

    for ch in (args.pan_ch, args.tilt_ch):
        if not 1 <= ch <= SERVO_CHANNELS:
            sys.exit(f"错误：通道号必须在 1~{SERVO_CHANNELS}，收到 {ch}")
    if args.pan_ch == args.tilt_ch:
        sys.exit("错误：水平和俯仰不能是同一个通道")

    t = Teleop(args)
    if not dry:
        t.start_publishers()

    print(f"L150Pro 键盘遥控  |  水平 ch{args.pan_ch} "
          f"{args.pan_range[0]}~{args.pan_range[1]}us，"
          f"俯仰 ch{args.tilt_ch} "
          f"{args.tilt_range[0]}~{args.tilt_range[1]}us，"
          f"模式 {args.mode}，速度档 {args.speed:.2f} m/s")
    if not args.center_on_start:
        print(f"云台：启动时不动，第一次按云台键才发命令"
              f"（假定当前在 {SERVO_CENTER_US}us 中位）")
    if dry:
        print("*** 演练模式：不连 ROS、不发消息 ***")
    print(SHORT_HELP)

    th = threading.Thread(target=t.pump, daemon=True)
    th.start()

    fd = None
    old = None
    try:
        if dry and args.keys is not None:
            for tok in [x.strip() for x in args.keys.split(",") if x.strip()]:
                if tok.startswith("wait"):        # wait0.6 = 什么都不按，等 0.6s
                    time.sleep(float(tok[4:]))
                    continue
                k = {"space": " "}.get(tok, tok)
                print(f"按键 {k!r}")
                msg = t.handle(k)
                if msg:
                    print(f"  {msg}")
                time.sleep(0.09)      # 让 pump 把真正发出去的值打印出来
                print(f"  -> {t.status()}")
        else:
            fd = sys.stdin.fileno()
            if not sys.stdin.isatty():
                sys.exit("错误：stdin 不是终端。请直接在终端里交互运行，不要用 "
                         "`ssh host 'python3 ...'` 这种把 stdin 吃掉的方式。")
            old = termios.tcgetattr(fd)
            new = old.copy()
            new[3] &= ~(termios.ICANON | termios.ECHO)   # cbreak，保留 ISIG 让 Ctrl-C 生效
            termios.tcsetattr(fd, termios.TCSADRAIN, new)

            sys.stdout.write(t.status())
            sys.stdout.flush()
            while t.running:
                k = read_key(0.05)
                if k == "h":
                    sys.stdout.write("\r\033[K")
                    print(__doc__)
                else:
                    msg = t.handle(k) if k is not None else None
                    if msg:
                        sys.stdout.write("\r\033[K" + msg + "\n")
                if not t.running:
                    break
                while t.notices:
                    sys.stdout.write("\r\033[K" + t.notices.pop(0) + "\n")
                sys.stdout.write("\r" + t.status() + "\033[K")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        t.running = False
        time.sleep(0.15)
        if not dry:
            t.stop_now()
        if old is not None:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if not dry:
            print()
        if t.node is not None:
            t.node.destroy_node()
            t._rclpy.shutdown()
        print("\n已退出 —— 底盘已归零；云台保持最后位置"
              "（固件设计如此，不会回中）")


if __name__ == "__main__":
    main()
