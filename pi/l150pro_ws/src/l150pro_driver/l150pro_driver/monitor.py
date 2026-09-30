"""独立串口监视器：不依赖 ROS，用于接线/协议/硬件排查。

用法：
    python -m l150pro_driver.monitor --port /dev/ttyUSB0
    # 或装包后：l150pro_monitor --port /dev/ttyUSB0
    # 标定等效轮距时传 --track-eff，边转边试，不用重新编译：
    l150pro_monitor --port /dev/l150pro --track-eff 0.95

按键（回车生效）：
    d  测 yaw 漂移率（车必须静止）—— 输出 度/分钟
    v  切换是否显示每条字段（默认只显示汇总行）
    r  清零链路统计
    q  退出

为什么需要它：
  调试时最常撞的两个坑是 ① 串口助手没开十六进制模式，二进制帧显示为乱码
  ② 分不清"没数据"和"数据不对"。本工具直接告诉你收到了什么。
"""

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("需要 pyserial：pip install pyserial", file=sys.stderr)
    sys.exit(1)

from .protocol import (FAULT_ENCODER, FAULT_ESTOP, FAULT_IMU, FAULT_OVERVOLT,
                       FAULT_RX_TIMEOUT, FAULT_UNDERVOLT, TRACK_EFF,
                       UplinkParser, YawUnwrapper, unwrap_delta,
                       build_motion_frame, FLAG_ENABLE)


# 原地转标定时，IMU 实测转角低于这个值就认定"车体没真的转"（多半是架起来了），
# 不出标定值。整圈是 6.2832 rad，取 0.5 rad(~29 度) 作为"这不是标定动作"的下界。
MIN_CAL_SPIN_RAD = 0.5


class Monitor:

    def __init__(self, port, baud, send_enable, track_eff=TRACK_EFF):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        # 标定用的有效全轮距：wz 是积分偏航的唯一来源，改这个值不用重新编译
        self.track_eff = track_eff
        self.parser = UplinkParser()
        self.verbose = False
        self.sending = send_enable
        self.running = True
        self._last = None
        self._last_rx = None
        self._seq = 0
        # 漂移测量
        self._drift_active = False
        self._drift_t0 = None
        self._drift_yaw0 = None
        # 里程累加（标定用）
        self._cal_active = False
        self._cal_t0 = None
        self._cal_dist = 0.0        # 由 vx 积分得到的距离 m
        self._cal_yaw = 0.0         # 由 wz 积分得到的【上报】偏航 rad
        self._cal_imu = YawUnwrapper()   # 由 IMU yaw 累加得到的【实测】转角
        self._cal_last_t = None

    def read_loop(self):
        while self.running:
            try:
                data = self.ser.read(256)
            except Exception as e:
                print(f"\n串口读取失败: {e}")
                self.running = False
                return
            for up in self.parser.feed(data):
                self._last = up
                self._last_rx = time.monotonic()
                self._accumulate(up)
                if self.verbose:
                    self._print_detail(up)

    def _accumulate(self, up):
        """积分出距离与偏航，用于标定 wheel_scale / track_eff。

        这里累加的是 up.wz —— 由轮速反解出的【角速度】，是速率，没有回绕
        问题（转多少圈都能积）。回绕的是 up.yaw，所以它单独走 YawUnwrapper
        累加，作为标定时的"实际转角"参考。
        """
        now = time.monotonic()
        if self._cal_last_t is not None:
            dt = now - self._cal_last_t
            if 0.0 < dt < 0.5:
                self._cal_dist += up.vx * dt
                self._cal_yaw += up.wz(self.track_eff) * dt
                self._cal_imu.feed(up.yaw)
        self._cal_last_t = now

    def send_loop(self):
        """持续发零速 + 使能帧，模拟 ROS 节点的心跳。"""
        while self.running and self.sending:
            self._seq = (self._seq + 1) & 0xFF
            try:
                self.ser.write(build_motion_frame(0.0, 0.0, 0.0,
                                                  FLAG_ENABLE, self._seq))
            except Exception:
                pass
            time.sleep(0.05)

    def _print_detail(self, up):
        names = "/".join(up.fault_names()) or "-"
        print(f"  tick={up.tick_ms:<8d} "
              f"vA={up.vA:+.3f} vB={up.vB:+.3f} vC={up.vC:+.3f} vD={up.vD:+.3f} "
              f"vx={up.vx:+.3f} wz={up.wz(self.track_eff):+.3f} | "
              f"r={up.roll:+.3f} p={up.pitch:+.3f} y={up.yaw:+.3f} gz={up.gz:+.3f} | "
              f"{up.voltage:.2f}V fault={up.fault:02X}({names}) seq={up.seq}")

    def summary_line(self):
        if self._last_rx is None:
            return "等待数据……（检查：端口选对了吗？波特率 115200？是否共地？）"
        age = time.monotonic() - self._last_rx
        if age > 1.0:
            return (f"⚠ {age:.1f}s 无数据 —— 下位机可能已复位/掉线/未上电 "
                    f"(已收 {self.parser.stats.frames} 帧)")
        up = self._last
        s = self.parser.stats
        names = "/".join(up.fault_names()) or "正常"
        cal = ""
        if self._cal_active:
            cal = (f" ‖ 标定: 距离={self._cal_dist:+.3f}m "
                   f"偏航={self._cal_yaw:+.4f}rad "
                   f"IMU={self._cal_imu.total:+.4f}rad")
        return (f"vx={up.vx:+.3f}m/s wz={up.wz(self.track_eff):+.3f}rad/s "
                f"yaw={up.yaw:+.3f}rad 电池={up.voltage:.2f}V [{names}] | "
                f"帧{s.frames} crc错{s.crc_err} 头错{s.head_err} 丢{s.dropped}"
                + cal)

    def start_cal(self):
        """开始标定测量：清零累加器。

        用法（标 wheel_scale）：
          1. 按 c 清零
          2. 让车直行一段【已知距离】（卷尺量，建议 2~3 m）
          3. 按 c 读出上报距离 -> wheel_scale = 实际距离 / 上报距离
        用法（标 track_eff）：
          1. 按 c 清零
          2. 让车原地转【已知角度】（建议 360 度 = 6.2832 rad）
             ⚠️ 必须在【实地】上转，不能把车架起来：架起来四轮空转、车体
             不动，IMU yaw 全程不变，标定值毫无意义。等效轮距的物理本质
             就是轮胎侧滑修正，轮子离地就没有侧滑可言。转速 0.3~0.5 rad/s。
          3. 按 c 读出上报偏航 -> track_eff_new = track_eff_old x (上报角 / 实际角)
             例：上报 7.0 rad 而实际转了 6.2832 rad，说明角度算大了，
                 新轮距 = 旧轮距 x 7.0/6.2832
             "实际角"可以直接读报告里的【IMU 实测转角】，不必自己拿量角器：
             它已解回绕（YawUnwrapper），转一圈不会变成 0。但 IMU yaw 是纯
             陀螺仪积分，电机振动下有整流误差，建议同时用地面标记核对一次。
          4. 试下一轮：重启时带上 --track-eff <新值>，不用改代码重新编译；
             量准了再把 protocol.py 的 TRACK_EFF 固化成最终值，build 一次收尾
        """
        self._cal_dist = 0.0
        self._cal_yaw = 0.0
        self._cal_imu.reset()
        self._cal_active = True
        self._cal_last_t = None
        print("\n标定累加已清零，开始测量……再按 c 读结果")

    def report_cal(self):
        if not self._cal_active:
            return
        print(f"\n>>> 上报距离 = {self._cal_dist:.3f} m"
              f"   上报偏航 = {self._cal_yaw:.4f} rad"
              f" ({self._cal_yaw * 57.29578:.2f} 度)")
        imu = self._cal_imu.total
        print(f"    IMU 实测转角 = {imu:.4f} rad ({imu * 57.29578:.2f} 度)"
              f"   <- 已解回绕，可当\"实际角\"用")
        if abs(imu) < MIN_CAL_SPIN_RAD:
            # 车体几乎没转。与其拿一个 ~0 的实测角去除、吐出一个荒谬的轮距
            # 让人怀疑工具，不如直接说清原因。
            print(f"    ⚠️ IMU 转角 {abs(imu):.3f} rad < {MIN_CAL_SPIN_RAD}，"
                  f"车体几乎没转 —— 不给标定值。\n"
                  f"       最常见原因：把车【架起来】了。架起来四轮空转、"
                  f"车体不动，IMU yaw 全程不变。\n"
                  f"       等效轮距的物理本质就是【轮胎侧滑修正】，"
                  f"轮子离地就没有侧滑可言，必须落地转。")
        else:
            print(f"    按 IMU 参考：track_eff 新值 = {self.track_eff:.4f} x "
                  f"({self._cal_yaw:.4f} / {imu:.4f}) = "
                  f"{self.track_eff * self._cal_yaw / imu:.4f}")
        print("    wheel_scale = 实际距离 / 上报距离")
        print(f"    track_eff 新值 = 旧值({self.track_eff:.3f}) x (上报角 / 实际角)")
        print("    ⚠️ IMU yaw 是纯陀螺仪积分，振动下有整流误差；"
              "原地转标定建议同时用地面标记核对整圈")
        self._cal_active = False

    def start_drift_test(self):
        if self._last is None:
            print("还没收到数据，无法测漂移")
            return
        if self._drift_active:
            return
        self._drift_active = True
        self._drift_t0 = time.monotonic()
        self._drift_yaw0 = None
        print("漂移测试开始：请让车【保持静止】30 秒，期间 yaw 会累积……")

    def tick_drift(self):
        if not self._drift_active or self._last is None:
            return
        y = self._last.yaw
        if self._drift_yaw0 is None:
            self._drift_yaw0 = y
            return
        elapsed = time.monotonic() - self._drift_t0
        if elapsed < 30.0:
            return
        # 解回绕：yaw 回绕到 +-PI，跨过那一下不处理会凭空多出 ∓2pi
        delta = unwrap_delta(self._drift_yaw0, y)
        deg_per_min = (delta * 57.29578) / (elapsed / 60.0)
        print(f"\n>>> yaw 漂移率 = {deg_per_min:+.2f} 度/分钟 "
              f"(30s 内漂 {delta * 57.29578:+.2f} 度)")
        print("    1~2 度/分 -> 方案 B(发 orientation, 大协方差) 也够用")
        print("    几十度/分 -> 必须用方案 A(不发布 orientation)，让 EKF 靠轮速观测航向")
        self._drift_active = False

    def run(self):
        threading.Thread(target=self.read_loop, daemon=True).start()
        if self.sending:
            threading.Thread(target=self.send_loop, daemon=True).start()
            print("已开启零速心跳帧（模拟 ROS 节点）")
        print(f"按 d 测 yaw 漂移 | c 标定距离/角度 | v 详细模式 | "
              f"r 清统计 | q 退出    [当前 track_eff = {self.track_eff:.3f}]\n")
        threading.Thread(target=self._stdin_loop, daemon=True).start()

        try:
            while self.running:
                self.tick_drift()
                sys.stdout.write("\r" + self.summary_line() + " " * 10)
                sys.stdout.flush()
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.ser.close()
            print("\n已退出")

    def _stdin_loop(self):
        for line in sys.stdin:
            c = line.strip().lower()
            if c == "q":
                self.running = False
                break
            if c == "v":
                self.verbose = not self.verbose
                print(f"\n详细模式: {'开' if self.verbose else '关'}")
            elif c == "r":
                self.parser.stats.frames = 0
                self.parser.stats.crc_err = 0
                self.parser.stats.head_err = 0
                self.parser.stats.dropped = 0
                print("\n统计已清零")
            elif c == "d":
                self.start_drift_test()
            elif c == "c":
                if self._cal_active:
                    self.report_cal()
                else:
                    self.start_cal()


def main():
    ap = argparse.ArgumentParser(description="L150Pro 串口监视器")
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--track-eff", type=float, default=TRACK_EFF,
                    help=f"标定用的有效全轮距(m)，默认 {TRACK_EFF}。"
                         f"原地转标定时迭代试值用，不必重新编译")
    ap.add_argument("--no-heartbeat", action="store_true",
                    help="不发零速心跳帧（只读监听）")
    a = ap.parse_args()
    Monitor(a.port, a.baud, not a.no_heartbeat, a.track_eff).run()


if __name__ == "__main__":
    main()
