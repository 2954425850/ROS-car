#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""PS2 手柄遥控节点（并控布局：边开车边操作机械臂）。

发布:
  /cmd_vel      geometry_msgs/Twist     底盘 vx(m/s)/wz(rad/s)，恒频发布（含零速，供驱动节点判活）
  /arm/command  sensor_msgs/JointState  position=[p1..p5 raw 0..1000, base field ±1000]，
                                        有臂输入及其尾随期按 loop_hz 发布
  /estop        std_msgs/Bool           急停锁存状态（边沿发布）
服务客户端:
  /arm/rehome   std_srvs/Trigger        START 键触发机械臂回中（锚定+回中）

运动模型 = **云平台前端 `Gv`/轮盘的逐行移植**（实现在 `chassis.py`，别在这儿重写）:
  vx∈±0.5 m/s, wz∈±1.5 rad/s, 最小转弯半径 0.6m（不能原地转）,
  有转向输入时最低 0.12 m/s, 轮盘**径向**死区 0.12, **没有低通**, 档位 0.15~0.50 步进 0.05。
  ⚠️ 照抄的两个后果，都是**故意的**，别当 bug 顺手改：
    · 只推左右（前后居中）⇒ 会以 0.12 m/s 前蠕画弧 —— 云端就是这样。
    · 云端缺原厂 `balance.c` 的 `if(Move_X<0) Move_Z=-Move_Z`，所以**后退时左右是反的**
      （见记忆 `l150pro-turn-decisions` ②）。本次要求是"完全照抄"，故没有补这一条。
键位在下方 CFG，--probe 实测编号后按需修改。
"""
import argparse
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .chassis import clamp, stick_to_drive
from .js_input import JsReader, JsState, run_probe
from .protocol import ARM_JOINT_NAMES

CFG = {
    # —— 轴（2026-09-27 22:07 实测确认：0=左X 1=左Y 2=右X 3=右Y 4=十字X 5=十字Y；
    #    每个轴的负方向 = 该摇杆/十字键的 左 或 上；DIR=±1 纠方向）——
    # 底盘走的是云平台的"轮盘"模型（二维矢量 + 径向死区，见 chassis.py），
    # 所以左摇杆是当**一根轮盘**读的：X→轮盘 x（右为正）、Y→轮盘 y（下为正）。
    'AX_PAD_X': 0, 'PAD_X_DIR': 1,                # 左摇杆X：轴正方向=右推 → 轮盘 x 同号
    'AX_PAD_Y': 1, 'PAD_Y_DIR': 1,                # 左摇杆Y：轴负方向=上推、轮盘 y 下为正 → 同号
    'AX_SHOULDER': 3, 'DIR_SHOULDER': 1,          # 右摇杆Y 上下=大臂（5 号舵机，方向已翻）
    'AX_BASE': 2, 'DIR_BASE': -1,                 # 右摇杆X 左右=底座（用户此前实测该轴方向反了）
    'AX_HAT_Y': 5,                                # 十字键 上下=速度档升/降
    'AX_HAT_X': 4,                                # 十字键 左右=预留

    # —— 按钮（2026-09-27 22:07 全量 --probe 实测，按物理位置逐个按、顺序 1:1 对上）——
    #     面键：上=0 下=2 左=3 右=1        → Y=0 A=2 X=3 B=1（左/右已由 ROLL 那两个实测印证）
    #     肩/扳机：L1=4 R1=5 L2=6 R2=7
    #     中间：SELECT=8 START=9 MODE=12
    #     摇杆按下：L3=10 R3=11
    #     注意：十字键是 AXIS 4/5，不是按键；摇杆按下才是按键。
    # —— 1~4 号舵机：按键表照抄 LeArm 出厂源码的单舵机模式 ——
    #     出处 Hiwonder/Porting/src/ps2_porting.c 的 ps2_run()：
    #       舵机1 △/×    舵机2 □/○    舵机3 R1/R2
    #       舵机4 L1/L2   舵机5 ↑/↓    舵机6 ←/→
    #     MAX = 往大端（target_duty → 1000），MIN = 往小端（→ 0）；按住走、松手停
    #     （源码那边是一对 robot_arm_knot_run / robot_arm_knot_stop）。
    #     面键按【物理位置】换成 Xbox 丝印：△=上=Y=0  ×=下=A=2  □=左=X=3  ○=右=B=1
    #     本轮只改 1~4 号；5/6 号仍挂在右摇杆上（源码用的是十字键），按你的要求先不动。
    #     START 已经是"回中"，和源码的 robot_arm_reset() 对得上，不用改。
    #     名字用 MAX/MIN 而不是"上下/开合/左右"：源码只定义到"往哪一端"，物理方向
    #     谁也没实测过，写成 UP/DOWN 等于在断言一件没验证的事。方向反了就翻 DIR_*。
    'BTN_GRIP_MAX': 0,            # 1 号 夹爪   —— Y（源码 △）
    'BTN_GRIP_MIN': 2,            # 1 号 夹爪   —— A（源码 ×）
    'DIR_GRIP': 1,
    'BTN_ROLL_MAX': 3,            # 2 号 腕旋   —— X（源码 □）
    'BTN_ROLL_MIN': 1,            # 2 号 腕旋   —— B（源码 ○）
    'DIR_ROLL_BTN': -1,           # 上机实测：这一路方向反了（只翻方向，编号不动）
    'BTN_PITCH_MAX': 5,           # 3 号 腕俯仰 —— R1（源码 R1）
    'BTN_PITCH_MIN': 7,           # 3 号 腕俯仰 —— R2（源码 R2）
    'DIR_PITCH_BTN': -1,          # R1/R2 效果对调（R1 往小端、R2 往大端）；编号不动
    'BTN_FOREARM_MAX': 4,         # 4 号 小臂   —— L1（源码 L1）
    'BTN_FOREARM_MIN': 6,         # 4 号 小臂   —— L2（源码 L2）
    'DIR_FOREARM_BTN': -1,        # L1/L2 效果对调；编号不动
    'BTN_REHOME': 9,              # START 回中（用户实测编号）
    'BTN_ESTOP': 8,               # SELECT 急停（实测编号 8；按一下锁存，再按一下解除）

    # —— 底盘（云平台前端同款参数，见 chassis.py）——
    'JS_DEVICE': '/dev/input/js0',
    'LOOP_HZ': 25,
    'GEAR_DEFAULT': 0.15, 'GEAR_MIN': 0.15, 'GEAR_MAX': 0.50, 'GEAR_STEP': 0.05,
    'VX_MAX': 0.50,
    'WZ_MAX': 1.50,
    'R_MIN': 0.6,
    'V_MIN': 0.12,
    'LAT_EPS': 0.02,
    'CHASSIS_TAIL_S': 0.30,       # 松杆后继续发零速 N 秒（明确刹车），之后不发、让出 cmd_mux
    'STICK_DEADZONE': 0.12,       # 底盘轮盘的**径向**死区（云端 joystickDeadzone）
    'DEADZONE': 0.15,             # 臂摇杆的**逐轴**死区（和上面那条不是一回事）
    'SMOOTHING': 0.35,            # 臂摇杆低通系数（底盘已照抄云端：没有低通）
    # ★ 2026-10-12（用户要求，**临时值**）：3.0 → 30.0。
    #   症状：Linux joydev **只在值变化时**才发事件，所以"握着摇杆不动"3 秒 = 没有事件
    #   = 被判成手柄失联 → 底盘清零。表现就是「一直朝一个方向开 2~3 秒自己停」
    #   （换个方向=有新事件=续命），转弯尤其明显（转弯本来就是持续输入）。
    #   30 秒是暂时放过它；**根治**是把判据从"3 秒没事件"换成真的探活（select 轮询 fd，
    #   设备没了 JsReader 本来就会 clear axes ⇒ 拔线仍然秒停），见记忆 raspot-freeze-chassis-base。
    'IDLE_ZERO_S': 30.0,          # 手柄无事件 N 秒 → 底盘清零
    'HAT_EDGE_CNT': 16000,        # 十字键判沿阈值（|值|超过视为按下）

    # —— 机械臂 ——
    'GRIP_MAX': 600,
    # 夹爪**张开**位（count）。只在"首帧 p1 没读到（0）"时当兜底目标用；
    # 绝不能拿 0 当目标（0 = 没读到，不是夹爪位置）。
    'GRIP_OPEN': 240,
    'RATE_STICK': 300.0,          # 摇杆关节速率 raw/s（大臂/底座，满偏约 3.3s 走完）
    'RATE_BTN': 250.0,            # 按键关节速率 raw/s（夹爪/腕/小臂）
    'ARM_TAIL_S': 0.30,           # 停止输入后继续发布 N 秒（走完坡道）
    'ARM_ACC': 2500.0,            # 臂通道加速度 raw/s²（250 raw/s 约 0.1s 起停）
    'ARM_SYNC_TOL': 30.0,        # 空闲时回读与目标偏差超过它才同步（count / 底座 field）
    'ARM_DEBOUNCE_TICKS': 2,      # 臂通道去抖：连续 N 拍有效才生效（滤鬼影事件）
}


def norm(raw):
    return clamp(raw / 32767.0, -1.0, 1.0)


def dead(x, d):
    return 0.0 if abs(x) < d else x


class Ps2TeleopNode(Node):

    def __init__(self):
        super().__init__('ps2_teleop_node')
        self.declare_parameter('js_device', CFG['JS_DEVICE'])
        self.declare_parameter('loop_hz', CFG['LOOP_HZ'])
        self.declare_parameter('gear_default', float(CFG['GEAR_DEFAULT']))

        js_dev = self.get_parameter('js_device').value
        hz = float(self.get_parameter('loop_hz').value)
        self.gear = clamp(float(self.get_parameter('gear_default').value),
                          CFG['GEAR_MIN'], CFG['GEAR_MAX'])

        # ★ 2026-10-21：'cmd_vel' → 'cmd_vel_teleop'，经 cmd_mux 仲裁后才到驱动。
        #   原来 25Hz 恒发（含全零），任何别的速度源（语音/云端/导航）都会和这串 0 交替。
        #   现在只在摇杆有输入时发（外加 CHASSIS_TAIL_S 的零速尾巴），松手即让出控制权。
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.pub_arm = self.create_publisher(JointState, 'arm/command', 10)
        self.pub_estop = self.create_publisher(Bool, 'estop', 10)
        self.cli_rehome = self.create_client(Trigger, 'arm/rehome')
        # 跟随实际回读：用户未操作臂时，目标同步到当前位姿，避免接管瞬间大幅跳变
        self.sub_fb = self.create_subscription(JointState, 'arm/feedback', self.on_fb, 10)

        self.js = JsState()
        JsReader(js_dev, self.js).start()
        self.create_timer(1.0 / hz, self.tick)
        self.period = 1.0 / hz

        # 运行状态
        self.estop = False
        self.last_estop_pub = None
        self.prev_sel = 0
        self.prev_buttons = {}
        self.sm_sh = self.sm_bs = 0.0
        self.prev_hat_up = 0
        self.prev_hat_dn = 0
        self.targets = None           # 首条 /arm/feedback 到达后按实际位姿初始化
        self.base_t = None
        self.last_arm_input = 0.0
        self.last_drive_input = None  # 最近一次底盘非零输入的时刻（None = 从没动过）
        self.arm_hold_cnt = {}        # 臂通道去抖计数
        self.arm_vel = [0.0] * 6      # 各臂通道当前速度 raw/s（加速度限幅后的）

        self.get_logger().info(
            f'ps2_teleop_node 启动：手柄 {js_dev} @ {hz:.0f}Hz，默认档 {self.gear:.2f} m/s')

    def on_fb(self, msg):
        pos = list(msg.position)
        if len(pos) < 6:
            return
        if self.targets is None:
            # ★ 首帧可能是"该拍没读到"（协议里 0 = 没读到）。拿 0 当目标 =
            #   把对应关节往 0 推。2026-10-21 真机：抓完恢复服务时首帧 p1=0，
            #   目标被初始化成 0 ⇒ **夹爪张开、刚抓到的椒掉了**。
            # ⚠️ 但**不能无限等**（2026-10-21 用户反馈"第一次开手柄臂不能动"）：
            #   p1 的读回本来就会偶发 0；若一直等 p1，`targets` 永远 None，
            #   `tick()` 里整个机械臂分支都不执行 ⇒ **臂完全没反应**（底盘仍能动）。
            #   所以：p2–p5 必须真读到（没读到才等），p1 没读到就用**张开位**兜底。
            if any(pos[i] == 0.0 for i in (1, 2, 3, 4)):
                return
            p1 = pos[0] if pos[0] > 0.0 else CFG['GRIP_OPEN']
            self.targets = [clamp(p1, 0, CFG['GRIP_MAX'])] + [
                clamp(pos[i], 0, 1000) for i in range(1, 5)]
            self.base_t = clamp(pos[5], -1000, 1000)
            self.get_logger().info(f'臂目标已按实际位姿初始化: {self.targets} base={self.base_t:.0f}')
            return
        if time.monotonic() - self.last_arm_input < 1.0:
            return   # 用户正在操作，别让回读覆盖目标
        # ★ 2026-10-01：只在【偏差大】时才同步（回中 / 别的程序动过臂）。
        #   原来是无条件覆盖：带载关节的回读比指令低几~十几 count（重力下垂），
        #   松手 1s 后目标被改成"垂下去的位置"，下次一推摇杆第一帧就是往下走 ⇒ 起步先沉一下。
        #   小偏差保留上次指令目标，固件插补从原指令平滑续走。
        tol = CFG['ARM_SYNC_TOL']
        for i in range(5):
            if pos[i] != 0.0 and abs(pos[i] - self.targets[i]) > tol:   # 0 = 该拍没读到
                self.targets[i] = clamp(pos[i], 0, CFG['GRIP_MAX'] if i == 0 else 1000)
        if abs(pos[5] - self.base_t) > tol:
            self.base_t = clamp(pos[5], -1000, 1000)

    # —— 主循环 ——
    def tick(self):
        c = CFG
        now = time.monotonic()
        with self.js.lock:
            axes = dict(self.js.axes)
            buttons = dict(self.js.buttons)
            last_ev = self.js.last_event
        stale = (now - last_ev) > c['IDLE_ZERO_S']
        if stale:
            axes, buttons = {}, {}

        # 急停：SELECT 按下沿【切换】—— 按一下锁存，再按一下解除。
        # 原来是"锁存后必须按住 2 秒才解除"，那个方案有个死结：estop_t0 在
        # 每个上升沿都会被重置，而手柄静默 3 秒后按键状态又被清零（IDLE_ZERO_S），
        # 所以解除窗口只有 2~3 秒 —— 手柄一抖或手一快点，计时就重来，永远解不开。
        # 边沿切换没有这个窗口，也不会因为"按久了反而失效"。
        sel = 1 if buttons.get(c['BTN_ESTOP']) else 0
        if sel and not self.prev_sel:
            self.estop = not self.estop
            if self.estop:
                self.get_logger().warn('急停锁存：底盘清零、机械臂保持（再按一次解除）')
            else:
                self.get_logger().info('急停解除')
        if self.estop != self.last_estop_pub:
            m = Bool()
            m.data = self.estop
            self.pub_estop.publish(m)
            self.last_estop_pub = self.estop
        self.prev_sel = sel

        # 速度档：十字键 上=升档 下=降档（按下沿）
        hat_y = axes.get(c['AX_HAT_Y'], 0)
        hat_up = 1 if hat_y < -c['HAT_EDGE_CNT'] else 0
        hat_dn = 1 if hat_y > c['HAT_EDGE_CNT'] else 0
        if hat_up and not self.prev_hat_up:
            self.gear = round(clamp(self.gear + c['GEAR_STEP'],
                                    c['GEAR_MIN'], c['GEAR_MAX']), 2)
            self.get_logger().info(f'速度档 → {self.gear:.2f} m/s')
        if hat_dn and not self.prev_hat_dn:
            self.gear = round(clamp(self.gear - c['GEAR_STEP'],
                                    c['GEAR_MIN'], c['GEAR_MAX']), 2)
            self.get_logger().info(f'速度档 → {self.gear:.2f} m/s')
        self.prev_hat_up = hat_up
        self.prev_hat_dn = hat_dn

        # 底盘：云平台"轮盘 + Gv"的逐行移植（实现在 chassis.py，这里只做轴→轮盘坐标）
        vx = wz = 0.0
        if not self.estop and not stale:
            x = c['PAD_X_DIR'] * norm(axes.get(c['AX_PAD_X'], 0))
            y = c['PAD_Y_DIR'] * norm(axes.get(c['AX_PAD_Y'], 0))
            vx, wz = stick_to_drive(x, y, self.gear, c)

        if vx != 0.0 or wz != 0.0:
            self.last_drive_input = now
        if (self.last_drive_input is not None
                and now - self.last_drive_input <= c['CHASSIS_TAIL_S']):
            msg = Twist()
            msg.linear.x = clamp(vx, -c['VX_MAX'], c['VX_MAX'])
            msg.angular.z = clamp(wz, -c['WZ_MAX'], c['WZ_MAX'])
            self.pub_cmd.publish(msg)

        # 机械臂：增量目标积分
        active = False
        if not self.estop and not stale and self.targets is not None:
            a = c['SMOOTHING']
            raw_sh = c['DIR_SHOULDER'] * (-norm(axes.get(c['AX_SHOULDER'], 0)))
            raw_bs = c['DIR_BASE'] * norm(axes.get(c['AX_BASE'], 0))
            self.sm_sh += a * (dead(raw_sh, c['DEADZONE']) - self.sm_sh)
            self.sm_bs += a * (dead(raw_bs, c['DEADZONE']) - self.sm_bs)
            in_sh = dead(self.sm_sh, c['DEADZONE'])
            in_bs = dead(self.sm_bs, c['DEADZONE'])
            in_wr = c['DIR_ROLL_BTN'] * (
                (1 if buttons.get(c['BTN_ROLL_MAX']) else 0)
                - (1 if buttons.get(c['BTN_ROLL_MIN']) else 0))
            in_wp = c['DIR_PITCH_BTN'] * (
                (1 if buttons.get(c['BTN_PITCH_MAX']) else 0)
                - (1 if buttons.get(c['BTN_PITCH_MIN']) else 0))
            in_fu = c['DIR_FOREARM_BTN'] * (
                (1 if buttons.get(c['BTN_FOREARM_MAX']) else 0)
                - (1 if buttons.get(c['BTN_FOREARM_MIN']) else 0))
            in_gr = c['DIR_GRIP'] * (
                (1 if buttons.get(c['BTN_GRIP_MAX']) else 0)
                - (1 if buttons.get(c['BTN_GRIP_MIN']) else 0))

            # 去抖：通道连续 N 拍有效才生效（滤单帧鬼影事件）
            nd = c['ARM_DEBOUNCE_TICKS']
            chans = {'wr': in_wr, 'wp': in_wp, 'sh': in_sh,
                     'bs': in_bs, 'fu': in_fu, 'gr': in_gr}
            for ch, val in chans.items():
                self.arm_hold_cnt[ch] = self.arm_hold_cnt.get(ch, 0) + 1 if val else 0
                chans[ch] = val if self.arm_hold_cnt[ch] >= nd else 0.0
            in_wr, in_wp, in_sh, in_bs, in_fu, in_gr = (
                chans[k] for k in ('wr', 'wp', 'sh', 'bs', 'fu', 'gr'))

            # ★ 2026-10-01：速度带加速度限幅（梯形起停）。
            #   原来按键一按就是 0→250 raw/s 的速度阶跃、一松就是 250→0 —— 带载的
            #   大臂/小臂被这一下"踢"起来，到位后像果冻一样晃（舵机自身阻尼很弱）。
            #   现在每个通道的速度以 ARM_ACC 爬升/回落：起步 ~0.1s、松手多滑 ~3°。
            dt = self.period
            want = [in_gr * c['RATE_BTN'], in_wr * c['RATE_BTN'], in_wp * c['RATE_BTN'],
                    in_fu * c['RATE_BTN'], in_sh * c['RATE_STICK'], in_bs * c['RATE_STICK']]
            dv = c['ARM_ACC'] * dt
            for k in range(6):
                self.arm_vel[k] += clamp(want[k] - self.arm_vel[k], -dv, dv)
            hi = [c['GRIP_MAX'], 1000, 1000, 1000, 1000]
            for k in range(5):
                t = self.targets[k] + self.arm_vel[k] * dt
                self.targets[k] = clamp(t, 0, hi[k])
                if self.targets[k] != t:
                    self.arm_vel[k] = 0.0      # 顶到限位就停，别在限位上"攒速度"
            t = self.base_t + self.arm_vel[5] * dt
            self.base_t = clamp(t, -1000, 1000)
            if self.base_t != t:
                self.arm_vel[5] = 0.0

            active = (any(abs(x) > 1e-9 for x in (in_wr, in_wp, in_sh, in_bs, in_fu, in_gr))
                      or any(abs(v) > 1e-6 for v in self.arm_vel))
            if active:
                self.last_arm_input = now

            # 回中（按下沿 → 服务）
            cur_re = 1 if buttons.get(c['BTN_REHOME']) else 0
            old_re = 1 if self.prev_buttons.get(c['BTN_REHOME']) else 0
            if cur_re and not old_re:
                if self.cli_rehome.service_is_ready():
                    self.cli_rehome.call_async(Trigger.Request())
                    self.get_logger().info('回中请求已发送（锚定+回中约需 10 秒以上）')
                else:
                    self.get_logger().warn('arm/rehome 服务不可用（驱动节点没在跑？）')

            if active or (now - self.last_arm_input) < c['ARM_TAIL_S']:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                # 来源标记：抓取服务看到 'teleop' 的臂指令 ⇒ 人接管，立刻中止抓取
                js.header.frame_id = 'teleop'
                js.name = list(ARM_JOINT_NAMES)
                js.position = [float(t) for t in self.targets] + [float(self.base_t)]
                self.pub_arm.publish(js)
        else:
            self.arm_vel = [0.0] * 6           # 急停 / 手柄失联：速度立即清零，不滑行

        self.prev_buttons = buttons

    def cleanup(self):
        self.pub_cmd.publish(Twist())          # 零速
        if self.last_estop_pub:
            m = Bool()
            m.data = False
            self.pub_estop.publish(m)


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--probe', action='store_true',
                    help='打印手柄原始事件（校键号用），不启动 ROS')
    opts, ros_args = ap.parse_known_args(args)
    if opts.probe:
        run_probe(CFG['JS_DEVICE'])
        return

    def _on_term(*_):
        raise KeyboardInterrupt()
    rclpy.init(args=ros_args)
    signal.signal(signal.SIGTERM, _on_term)
    node = Ps2TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
