# -*- coding: utf-8 -*-
"""视觉抓取 ROS 2 action 服务：`/arm/grasp`（raspot_interfaces/action/Grasp）。

和 `tools/grasp_once.py` 跑的是**同一个** `grasp.run()`，区别在怎么接硬件：

| | grasp_once（CLI） | 本服务 |
|---|---|---|
| 驱动 | sudo 停 ps2-teleop、另起一个驱动 | **就用常驻驱动**（服务不停、底盘照常可用） |
| arm_t_ms | 另起驱动时 -p arm_t_ms:=100 | 运行时把常驻驱动的参数 set 成 100，完了 set 回去 |
| 收尾 | 重启服务 ⇒ INIT_HOME，臂自己归位、东西可能掉 | 臂停在抬起位，**不归位** |
| 观测 | 读 /tmp/k230/latest-result.json（年龄按 mtime 估） | 订阅 /k230/result（带拍摄时刻 stamp_mono） |
| ToF | 取最新一帧 | 按拍摄时刻取对齐的那帧（TofClient at_mono） |
| 结果 | 打印 + 退出码 | Result：ok / code / reason / 目标点 xyz |

## 谁能打断
- **取消 goal**（语音唤醒词急停走这条）⇒ CANCELED
- **PS2 动了机械臂**（/arm/command 上出现 frame_id='teleop'）⇒ ABORTED_BY_OPERATOR，人优先
- **/estop=true** ⇒ ESTOP（驱动那边本来也会丢弃臂指令）
打断时臂停在最后一条指令的位置（不归位、不松爪）。

## 不做口头确认
用户定：说了"抓 X"就直接抓（见记忆 phase2-plan-decisions）。

## 语音动作 `/arm/motion`（raspot_interfaces/action/ArmMotion）+ `/arm/describe`（2026-10-31 加）
设计 `pi/voice-chatbot/docs/plans/2026-10-31-arm-voice-motion-design.md` §4 §8。
spec_json → `motion_plan.plan`（纯函数）→ `motion_exec.run_motion`（逐帧发一次 + 回读确认）。
与抓取共用 `_busy`/`_abort`/`_on_cmd`/`_on_estop`/`_set_driver_t_ms`：一个在跑另一个被拒。
发布打 frame_id='motion'。打断规则同上；取消时额外发一帧"冻结"（目标=当前回读、t_ms=100）；
`/arm/cmd_gate` 变 dead（复位/回中）也算人接管，零发布。
"""
import dataclasses
import json
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from raspot_interfaces.action import ArmMotion, Grasp

from arm_grasp import grasp, ltrace, observe
from arm_grasp.collect import JOINT_NAMES, k230_host_from_result
from arm_grasp.grasp_codes import code_for
from arm_grasp import motion_exec, motion_plan, motion_spec

OBS_WAIT_S = 5.0          # 锁框后等结果流出现这个框
REC_MAX_AGE_S = 1.5       # 结果记录超过这么老就不用（同 observe.fresh_result）

# 上电复位位姿：p2(腕旋) p3(腕俯) p4(小臂) p5(大臂) base。夹爪 p1 保持当前闭合值不回。
# 值来自开机时 ps2_teleop 按实际位姿初始化的 targets=[214,496,180,129,415,3]。
HOME_JOINTS = (496.0, 180.0, 129.0, 415.0, 0.0)   # 复位位姿 p2..p5, base（判到位用）


class _Abort(Exception):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code, self.reason = code, reason


def _stable(fb):
    return fb is not None and len(fb) >= 6 and all(fb[i] != 0.0 for i in (1, 2, 3, 4))


class ServerLink:
    """`grasp.Link`：spin 里查打断标志，publish 打 'grasp' 来源标记。"""

    def __init__(self, srv, want, tof, latency_s):
        self.srv, self.want, self.tof, self.latency_s = srv, want, tof, latency_s
        self.follow = observe.BoxFollower(want)
        self.last_frame = None
        self._last_fb = None
        self.n_obs = self.n_none = self.n_depth = 0

    def now(self):
        return time.monotonic()

    def spin(self, dt):
        end = time.monotonic() + dt
        while True:
            self.srv.check_abort()
            rest = end - time.monotonic()
            if rest <= 0:
                return
            time.sleep(min(rest, 0.02))

    def fb(self):
        f = self.srv.fb
        if _stable(f):
            self._last_fb = [float(v) for v in f]
        return self._last_fb

    def obs(self):
        rec = self.srv.latest_rec()
        if rec is None or rec.get('frame') == self.last_frame:
            return None
        self.last_frame = rec.get('frame')
        b = self.follow.pick(rec)
        if b is None:
            self.n_none += 1
            return None
        self.n_obs += 1
        o = self.srv.make_obs(rec, b, self.tof, self.latency_s)
        if o.depth_mm is not None:
            self.n_depth += 1
        return o

    def publish(self, fields):
        self.srv.publish_arm(fields)


MOTION_FB_FRESH_S = 0.5   # 动作用的回读超过这么老算没有
_MOTION_ABORT_TEXT = {'手柄动了机械臂，抓取让出控制权': '手柄动了机械臂，动作让出控制权'}


class MotionLink:
    """`motion_exec` 的 Link：spin 里查打断（含 cmd_gate 变 dead），publish 打 'motion' 来源标记。"""

    def __init__(self, srv):
        self.srv = srv

    def now(self):
        return time.monotonic()

    def fb(self):
        return self.srv.motion_fb()

    def set_t_ms(self, ms):
        return bool(self.srv._set_driver_t_ms(ms))

    def spin(self, dt):
        end = time.monotonic() + dt
        while True:
            self.srv.check_motion_abort()
            rest = end - time.monotonic()
            if rest <= 0:
                return
            time.sleep(min(rest, 0.02))

    def publish(self, fields):
        self.srv.publish_motion(fields)


class GraspServer(Node):

    def __init__(self):
        super().__init__('arm_grasp_server')
        # 2026-10-30 真机：同一目标 α=−88 下扎后爪尖离目标 3.8mm，α=−81 却是 22.7mm
        # （浅角下扎横向漂）。这臂可靠的抓取角在 −84…−88，默认种子改 −84；
        # 需要浅角的目标仍可在 goal 里显式给 alpha0。
        self.declare_parameter('alpha0_default', -84.0)
        self.declare_parameter('lift_m_default', 0.055)      # 用户流程：抬 5~6cm
        # 关节下垂补偿增益（GraspConfig.joint_comp）。0 = 关掉。A/B 用（2026-10-22 查扎深）：
        # ros2 param set /arm_grasp_server joint_comp 0.0 后抓一次，对比是否还压进桌面。
        self.declare_parameter('joint_comp', 1.0)
        self.declare_parameter('grasp_arm_t_ms', 100)        # 同 grasp_once 的 --t-ms
        self.declare_parameter('normal_arm_t_ms', 40)        # 抓完还原成这个（= 驱动默认）
        self.declare_parameter('driver_node', 'l150pro_driver')
        self.declare_parameter('use_tof', True)
        # 径向偏置（mm，+ = 目标向外/远离车，− = 往车这边拉）。可运行时调：
        #   ros2 param set /arm_grasp_server bias_r_mm 5
        # ★ 2026-10-31 归 0：10-30 照 grasp_once 那句自相矛盾的帮助（"偏后 ~1cm 填 −1"）
        #   设成了 −10 ⇒ 目标点被往车这边拉 1cm ⇒ 用户实测抓取"偏后"。真偏后要往**正**调。
        #   10-01 文档原话：慢档横向已 ±2mm，"−1 那次的偏后是快档的锅"，先别乱填。
        self.declare_parameter('bias_r_mm', 0.0)
        # 合拢后爪尖落在物体顶面以下 frac·h（0.5 = 半高）；0 = 关（对准顶面的旧行为）。
        # d 的算法与约束见 grasp.grip_depth()。
        self.declare_parameter('grip_depth_frac', 0.5)
        # ★ 2026-10-31 用户实测：抓取**总是偏高** ⇒ 在按物体高算出的 d 上**固定再加 3mm**
        #   （用户原话"直接固定往下多走 0.3cm"）。这是标定修正，不是兜底：
        #   h 测没测到都加。A/B：ros2 param set /arm_grasp_server grip_extra_down_m 0.0
        self.declare_parameter('grip_extra_down_m', 0.003)

        cb = ReentrantCallbackGroup()
        self.fb = None
        self._rec = None
        self._rec_t = 0.0
        self._lock = threading.Lock()
        self._busy = False
        self._abort = None             # (code, reason)
        self._goal = None

        self.pub_arm = self.create_publisher(JointState, 'arm/command', 10)
        self.create_subscription(JointState, 'arm/feedback', self._on_fb, 10, callback_group=cb)
        self.create_subscription(JointState, 'arm/command', self._on_cmd, 10, callback_group=cb)
        self.create_subscription(Bool, 'estop', self._on_estop, 10, callback_group=cb)
        self.create_subscription(String, 'k230/result', self._on_rec, 10, callback_group=cb)

        self.tof = None
        if self.get_parameter('use_tof').value:
            try:
                from tof_a010.tof_client import TofClient
                self.tof = TofClient(node_name='arm_grasp_server_tof')
                if not self.tof.start():
                    self.get_logger().warn(f'ToF 没起来：{self.tof.reason}（深度只能靠 K230 结果）')
                    self.tof = None
            except ImportError as e:
                self.get_logger().warn(f'没有 tof_a010：{e}')

        self._grip_last = None           # 最近非零的夹爪回读（回位时保持夹住）
        self.create_service(Trigger, 'arm/go_home', self._go_home_srv, callback_group=cb)

        self._as = ActionServer(self, Grasp, 'arm/grasp', execute_callback=self._execute,
                                goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                                callback_group=cb)
        self.get_logger().info('arm_grasp_server 就绪：/arm/grasp')
        self._init_motion(cb)

    # ---- 订阅 ----

    def _on_fb(self, msg):
        self.fb = list(msg.position)
        if len(msg.position) > 0 and msg.position[0] > 0:
            self._grip_last = float(msg.position[0])

    def _on_cmd(self, msg):
        if self._busy and msg.header.frame_id == 'teleop':
            self._abort = self._abort or ('ABORTED_BY_OPERATOR', '手柄动了机械臂，抓取让出控制权')

    def _on_estop(self, msg):
        if self._busy and msg.data:
            self._abort = self._abort or ('ESTOP', '急停')

    def _on_rec(self, msg):
        try:
            rec = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._rec, self._rec_t = rec, time.monotonic()

    def latest_rec(self):
        with self._lock:
            if self._rec is None or time.monotonic() - self._rec_t > REC_MAX_AGE_S:
                return None
            return self._rec

    # ---- action ----

    def _on_goal(self, goal):
        if self._busy:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal):
        self._abort = self._abort or ('CANCELED', '取消了')
        return CancelResponse.ACCEPT

    def check_abort(self):
        if self._abort is not None:
            raise _Abort(*self._abort)

    def publish_arm(self, fields):
        self.check_abort()             # 被打断后一条都不再发
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'grasp'
        m.name = list(JOINT_NAMES)
        m.position = [float(v) for v in fields]
        self.pub_arm.publish(m)

    def make_obs(self, rec, b, tof, latency_s):
        """一条记录 + 挑中的框 → grasp.Obs。时间用拍摄时刻，深度按拍摄时刻对齐。"""
        t_cap = rec.get('stamp_mono')
        if not isinstance(t_cap, (int, float)):
            t_cap = time.monotonic() - 0.1
        d = None
        self._last_depth = ('k230', b.get('depth_mm'), {})
        if tof is not None:
            try:
                d, info = tof.depth_for_box(b['box'], at_mono=t_cap)
                info = dict(info or {})
                try:      # 记下框映射到的 ToF 区域 (r0,r1,c0,c1)，排查"量到桌面"
                    from tof_a010 import a010 as _a010
                    info['region'] = _a010.region_bounds(
                        b['box'], map_cfg=tof.cfg.get('map'),
                        pad=float(tof.cfg['pad']), rows=tof._rows, cols=tof._cols)
                except Exception:      # noqa: BLE001
                    pass
                self._last_depth = ('tof', d, info)
            except Exception:          # noqa: BLE001 —— ToF 出错降级成没深度
                d = None
        if d is None:
            d = b.get('depth_mm')
        u, v = observe.box_center_ai(b['box'])
        # estimate_point 用 t_arrive - latency_s 当拍摄时刻 ⇒ 这里反加回去，正好落在 t_cap
        return grasp.Obs(t_cap + latency_s, u, v, rec.get('frame'), b.get('src', 'det'),
                         b['box'], d)

    def _go_home_srv(self, req, resp):
        if self._busy:
            resp.success, resp.message = False, '抓取进行中，先等它结束或取消'
            return resp
        try:
            self._go_home()
            resp.success, resp.message = True, '已回到复位位姿（夹爪保持）'
        except _Abort as a:
            resp.success, resp.message = False, a.reason
        except Exception as e:         # noqa: BLE001
            resp.success, resp.message = False, repr(e)
        return resp

    def _go_home(self):
        """回到上电复位位姿，**夹爪保持闭合** —— 调驱动专用服务 /arm/rehome_hold（flags 0x03）。"""
        from std_srvs.srv import Trigger
        cli = self.create_client(Trigger, 'arm/rehome_hold')
        try:
            if not cli.wait_for_service(timeout_sec=3.0):
                raise _Abort('ERROR', 'arm/rehome_hold 服务不在（驱动没起来？）')
            fut = cli.call_async(Trigger.Request())
            end = time.monotonic() + 3.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            if not fut.done() or not fut.result().success:
                raise _Abort('ERROR', 'arm/rehome_hold 没回成功')
        finally:
            self.destroy_client(cli)
        self._wait_home_done()

    def _wait_home_done(self, timeout=20.0):
        """等归位结束：回读稳定在复位位姿。归位期间回读是固件总线计数器（假值），
        稳定下来 = 交棒回 READY。"""
        import math as _m
        good = 0
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.check_abort()
            fb = self.fb
            if _stable(fb):
                ok = (abs(fb[1] - HOME_JOINTS[0]) <= 25 and abs(fb[4] - HOME_JOINTS[3]) <= 25
                      and abs(fb[5]) <= 30)
                good = good + 1 if ok else 0
                if good >= 5:
                    self.get_logger().info('go_home 到位：%s' % [round(v, 1) for v in fb])
                    return
            time.sleep(0.2)
        self.get_logger().warn('go_home 等归位超时（%.0fs），夹爪已保持，臂位姿未确认' % timeout)

    def _feedback(self, gh, phase, detail):
        fb = Grasp.Feedback()
        fb.phase, fb.detail = phase, detail[:200]
        gh.publish_feedback(fb)

    def _log_run(self, gh, line):
        self.get_logger().info(line)
        self._feedback(gh, 'run', line)

    def _set_driver_t_ms(self, value):
        """常驻驱动的 arm_t_ms。失败只告警（用驱动当前值也能跑，只是更"冲"）。"""
        from rcl_interfaces.srv import SetParameters
        name = self.get_parameter('driver_node').value
        cli = self.create_client(SetParameters, f'/{name}/set_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'{name} 的参数服务不在，arm_t_ms 没改')
                return False
            req = SetParameters.Request()
            req.parameters = [Parameter('arm_t_ms', value=int(value)).to_parameter_msg()]
            fut = cli.call_async(req)
            end = time.monotonic() + 2.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            return fut.done() and all(r.successful for r in fut.result().results)
        finally:
            self.destroy_client(cli)

    def _execute(self, gh):
        g = gh.request
        res = Grasp.Result()
        res.frame_id = 'arm_base'
        self._busy, self._abort = True, None
        host = None
        locked = False
        t_ms_changed = False
        try:
            want = [float(v) for v in g.box]
            if len(want) != 4 or not all(0.0 <= v <= 1.0 for v in want) \
                    or not (want[0] < want[2] and want[1] < want[3]):
                raise _Abort('BAD_GOAL', f'框不合法：{want}')
            alpha0 = g.alpha0 if g.alpha0 != 0.0 else self.get_parameter('alpha0_default').value
            lift = g.lift_m if g.lift_m > 0.0 else self.get_parameter('lift_m_default').value
            # z_plane_m 只是占位：require_depth=True 时 estimate_point 绝不使用它，
            # 缺深度走 NO_DEPTH / Refused，而不是拿旧 1.6cm 平面蒙一个 O。
            cfg = grasp.GraspConfig(
                z_plane_m=0.016, require_depth=True,
                alpha0=float(alpha0), lift_m=float(lift),
                joint_comp=float(self.get_parameter('joint_comp').value),
                bias_r_m=float(self.get_parameter('bias_r_mm').value) / 1000.0)

            # ① 锁框（臂还没动，失败可重试）
            self._feedback(gh, 'preflight', '锁框')
            try:
                host = k230_host_from_result()
            except Exception as e:     # noqa: BLE001
                raise _Abort('LOCK_FAILED', f'拿不到板子地址：{e}')
            rep_lock = {}
            for _ in range(3):
                rep_lock = observe.k230_cmd(host, {'box': want}, timeout=5.0)
                if rep_lock.get('ok'):
                    break
                time.sleep(0.5)
            if not rep_lock.get('ok'):
                raise _Abort('LOCK_FAILED', f'板子没锁上框：{rep_lock}')
            locked = True

            # ② 回读
            end = time.monotonic() + 3.0
            while not _stable(self.fb) and time.monotonic() < end:
                time.sleep(0.05)
            if not _stable(self.fb):
                raise _Abort('NO_FEEDBACK', '读不到可信的机械臂回读')
            fb = [float(v) for v in self.fb]
            trace = ltrace.Trace()
            trace.add(time.monotonic(), fb, fb)

            # ③ 一条观测 → 目标点 O
            self._feedback(gh, 'preflight', '等目标框和深度')
            obs = None
            end = time.monotonic() + OBS_WAIT_S
            while obs is None and time.monotonic() < end:
                self.check_abort()
                rec = self.latest_rec()
                b = observe.pick_box(rec, want) if rec else None
                if b is not None:
                    obs = self.make_obs(rec, b, self.tof, cfg.latency_s)
                else:
                    time.sleep(0.05)
            if obs is None:
                raise _Abort('NO_TARGET', '结果流里挑不出这个框（跟踪丢了？）')
            if obs.depth_mm is None:
                raise _Abort('NO_DEPTH', '没拿到目标深度（ToF 盲区 <150mm / tof_node 没起？）')
            try:
                O, _C, _d, _j, _t = grasp.estimate_point(
                    trace, obs, cfg.z_plane_m, cfg.latency_s,
                    require_depth=cfg.require_depth)
            except Exception as e:     # noqa: BLE001
                raise _Abort('NO_TARGET', f'算不出目标点：{e}')
            res.target_valid = True
            res.target_xyz = [float(v) for v in O]
            _dsrc, _dval, _dinfo = getattr(self, '_last_depth', ('?', None, {}))
            # 物体高 h = 桌面 − 顶面（同一帧 ToF 原始读数）⇒ 往顶面以下扎 d
            h_m = None
            if _dinfo.get('table_mm') is not None and _dinfo.get('raw_mm') is not None:
                h_m = (float(_dinfo['table_mm']) - float(_dinfo['raw_mm'])) / 1000.0
            frac = float(self.get_parameter('grip_depth_frac').value)
            d_m = grasp.grip_depth(h_m, frac=frac) if frac > 0.0 else 0.0
            # ★ 2026-10-31：固定再往下 3mm（用户：抓取总是偏高）。加在 clamp 之后 ——
            #   要的就是"多走"，不再让 grip_depth 的桌面余量约束把它吃掉。
            d_m += float(self.get_parameter('grip_extra_down_m').value)
            cfg = cfg._replace(grip_depth_m=d_m)
            O_g = (O[0], O[1], O[2] - d_m)
            lvl, tgt, msg = grasp.plan_verdict(O_g, cfg)
            self.get_logger().info(
                f'O={tuple(round(v, 4) for v in O)}  S0={lvl}：{msg}  '
                f'深度={_dval}mm 源={_dsrc} raw={_dinfo.get("raw_mm")} '
                f'offset={_dinfo.get("offset_mm")} dt={_dinfo.get("dt_s")} '
                f'区域={_dinfo.get("region")}  '
                f'桌面={_dinfo.get("table_mm") and round(_dinfo["table_mm"])}mm '
                f'h={"未知" if h_m is None else "%.0fmm" % (h_m * 1000)} '
                f'⇒ 下扎 d={d_m * 1000:.0f}mm（含固定 +{float(self.get_parameter("grip_extra_down_m").value) * 1000:.0f}mm，'
                f'合拢爪尖在顶面下 {(d_m + grasp.GRIP_EXTEND_M) * 1000:.0f}mm）')
            # S0 只做参考、不拦（用户 2026-10-21：防御判据导致的失败没必要）。
            # 真够不着时 aim 相会铺满 α 包络再试，还不行才以 OUT_OF_REACH 收场。
            if g.dry_run:
                res.ok, res.code, res.reason = True, 'OK', f'dry-run：可行（{lvl}：{msg}）'
                gh.succeed()
                return res

            # ④ 真跑
            t_ms_changed = self._set_driver_t_ms(self.get_parameter('grasp_arm_t_ms').value)
            link = ServerLink(self, want, self.tof, cfg.latency_s)
            self._feedback(gh, 'aim', '开始')
            rep = grasp.run(cfg, link, phase='all',
                            log=lambda *a: self._log_run(gh, ' '.join(str(x) for x in a)))
            res.ok = bool(rep.get('ok'))
            res.code = code_for(rep.get('stopped'), res.ok)
            done = [ph for ph, _r, _n in rep.get('phases') or []]
            if res.code == 'REFUSED' and 'aim' not in done:
                res.code = 'OUT_OF_REACH'      # 对准都没完成 ⇒ 这个点臂够不着（车臂联动看这个）
            res.reason = rep.get('stopped') or '超过最大拍数'
            # ★ 2026-10-30：下扎中途就地合爪、且爪尖离 s_stop 目标点 >12mm 时，
            #   后面 close/lift 仍会“成功”，但用户看到的是没抓到。不能报 OK。
            fb_err = rep.get('fallback_err_m')
            if rep.get('descend_fallback') and fb_err is not None and fb_err > 0.010:
                res.ok = False
                res.code = 'REFUSED'
                res.reason = ('下扎没到位：爪尖离 s_stop 目标点 %.1fmm（>10mm），'
                              '就地合爪位置可能偏，未确认抓住' % (fb_err * 1000.0))
            try:
                res.err_m = float(rep.get('err_m'))
            except (TypeError, ValueError):
                pass
            if rep.get('O_last') is not None:
                res.target_xyz = [float(v) for v in rep['O_last']]
            if res.ok:
                self._feedback(gh, 'home', '回复位位姿（保持夹爪）')
                self._go_home()
                gh.succeed()
            elif res.code == 'GRIP_EMPTY':
                # 夹空：抬起已经走完、臂在桌面上方 ⇒ 照常回复位，但如实报失败
                self.get_logger().warn(f'抓取结束：GRIP_EMPTY {res.reason}')
                self._feedback(gh, 'home', '夹空，回复位位姿')
                self._go_home()
                gh.abort()
            else:
                gh.abort()
            return res
        except _Abort as a:
            res.ok, res.code, res.reason = False, a.code, a.reason
            self.get_logger().warn(f'抓取结束：{a.code} {a.reason}')
            if a.code == 'CANCELED':
                gh.canceled()
            else:
                gh.abort()
            return res
        except Exception as e:         # noqa: BLE001
            self.get_logger().error(f'抓取异常：{e!r}')
            res.ok, res.code, res.reason = False, 'ERROR', repr(e)
            gh.abort()
            return res
        finally:
            if t_ms_changed:
                self._set_driver_t_ms(self.get_parameter('normal_arm_t_ms').value)
            if locked and host:
                observe.k230_cmd(host, {'cmd': 'stop'})
            self._busy = False

    # ================================================================ 语音动作（只加）

    def _init_motion(self, cb):
        # motion.yaml 路径；空 = share/arm_grasp/config/motion.yaml（没装就用源码树）
        self.declare_parameter('motion_config', '')
        self._motion_lock = threading.Lock()
        self._mfb = None                 # (fields, t) 最近一拍可信回读
        self._grip_cmd = None            # 最近一次 /arm/command 的夹爪指令（任何来源）
        self._estop_on = False
        self._gate = motion_exec.CmdGate()
        self._history = motion_exec.PoseHistory(maxlen=5)
        self._motion_err = None
        try:
            path = self.get_parameter('motion_config').value or None
            self._mcfg = motion_spec.load_config(path)
            self._ecfg, self._scfg = motion_exec.load_runtime_config(path)
        except Exception as e:         # noqa: BLE001 —— 配置坏了只废掉动作，不连累抓取
            self._motion_err = 'motion.yaml 读不了：%r' % (e,)
            self._mcfg, self._ecfg, self._scfg = None, None, motion_exec.StateConfig()
            self.get_logger().error(self._motion_err)
        self.create_subscription(JointState, 'arm/feedback', self._on_fb_motion, 10, callback_group=cb)
        self.create_subscription(JointState, 'arm/command', self._on_cmd_grip, 10, callback_group=cb)
        self.create_subscription(Bool, 'estop', self._on_estop_state, 10, callback_group=cb)
        self.create_subscription(Bool, 'arm/cmd_gate', self._on_cmd_gate, 10, callback_group=cb)
        self.create_service(Trigger, 'arm/describe', self._describe_srv, callback_group=cb)
        self._as_motion = ActionServer(self, ArmMotion, 'arm/motion',
                                       execute_callback=self._execute_motion,
                                       goal_callback=self._on_goal, cancel_callback=self._on_cancel,
                                       callback_group=cb)
        self.get_logger().info('arm_grasp_server 就绪：/arm/motion /arm/describe')

    def _on_fb_motion(self, msg):
        f = list(msg.position)
        if _stable(f):
            with self._lock:
                self._mfb = ([float(v) for v in f[:6]], time.monotonic())

    def _on_cmd_grip(self, msg):
        if len(msg.position) > 0:
            self._grip_cmd = float(msg.position[0])

    def _on_estop_state(self, msg):
        self._estop_on = bool(msg.data)

    def _on_cmd_gate(self, msg):
        self._gate.on_msg(msg.data, time.monotonic())

    def motion_fb(self):
        with self._lock:
            if self._mfb is None or time.monotonic() - self._mfb[1] > MOTION_FB_FRESH_S:
                return None
            return list(self._mfb[0])

    def _wait_motion_fb(self, timeout):
        end = time.monotonic() + timeout
        fb = self.motion_fb()
        while fb is None and time.monotonic() < end:
            time.sleep(0.05)
            fb = self.motion_fb()
        return fb

    def check_motion_abort(self):
        if self._abort is None and self._gate.dead(time.monotonic()):
            self._abort = ('ABORTED_BY_OPERATOR', '机械臂进入复位（回中），动作中止')
        if self._abort is not None:
            code, reason = self._abort
            raise motion_exec.Abort(code, _MOTION_ABORT_TEXT.get(reason, reason))

    def publish_motion(self, fields):
        """取消（CANCELED）时仍允许发冻结帧；人接管/急停/复位中一条都不发。"""
        try:
            self.check_motion_abort()
        except motion_exec.Abort as a:
            if a.code != 'CANCELED':
                raise
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'motion'
        m.name = list(JOINT_NAMES)
        m.position = [float(v) for v in fields]
        self.pub_arm.publish(m)

    def _state_json(self, fb=None):
        fb = fb if fb is not None else self.motion_fb()
        if fb is None:
            return ''
        return motion_exec.state_json(fb, self._scfg, self._grip_cmd)

    def _describe_srv(self, req, resp):
        fb = self._wait_motion_fb(0.5)
        if fb is None:
            resp.success, resp.message = False, '读不到可信的机械臂回读（驱动没起来/臂没上电/正在复位？）'
            return resp
        try:
            resp.success, resp.message = True, self._state_json(fb)
        except Exception as e:         # noqa: BLE001
            resp.success, resp.message = False, repr(e)
        return resp

    def _motion_feedback(self, gh, step, total, detail):
        fb = ArmMotion.Feedback()
        fb.step, fb.total, fb.detail = int(step), int(total), str(detail)[:200]
        gh.publish_feedback(fb)

    def _execute_motion(self, gh):
        g = gh.request
        res = ArmMotion.Result()
        with self._motion_lock:
            if self._busy:             # goal_callback 和 execute 之间被抓取抢了
                res.ok, res.code, res.reason = False, 'BUSY', '机械臂正忙（抓取或另一个动作在跑）'
                gh.abort()
                return res
            self._busy, self._abort = True, None
        try:
            if self._motion_err:
                raise _Abort('ERROR', self._motion_err)
            try:
                spec = motion_spec.parse_spec(g.spec_json)
            except motion_spec.SpecError as e:
                raise _Abort('BAD_GOAL', '动作描述不合法：%s' % e)
            summary = motion_exec.spec_summary(spec)
            fb = self._wait_motion_fb(1.0)
            if fb is None:
                raise _Abort('NO_FEEDBACK', '读不到可信的机械臂回读，没动')
            before = list(fb)
            before[0] = motion_exec.grip_hold_value(fb[0], self._grip_cmd, self._scfg)
            try:
                p = motion_plan.plan(spec, before, self._history.snapshot(), self._mcfg)
            except motion_spec.SpecError as e:
                raise _Abort('BAD_GOAL', '动作描述不合法：%s' % e)
            total_s = sum(k.t_ms for k in p.keyframes) / 1000.0
            self.get_logger().info('motion%s：%s ⇒ %d 帧 %.1fs partial=%s notes=%s  %s' % (
                '(dry)' if g.dry_run else '', summary, len(p.keyframes), total_s, p.partial,
                p.notes, ' | '.join('%s %s %dms' % (k.label, [round(v) for v in k.fields], k.t_ms)
                                    for k in p.keyframes)))
            if g.dry_run:
                res.code = motion_exec.final_code('OK', p.partial)
                res.ok = True
                frames = '；'.join('%s %dms' % (k.label, k.t_ms) for k in p.keyframes)
                res.reason = 'dry-run（没动）：' + motion_exec.compose_reason(
                    res.code, summary, p.notes,
                    '%d 帧约 %.1fs：%s' % (len(p.keyframes), total_s, frames))
                res.state_json = self._state_json(fb)
                gh.succeed()
                return res
            if self._gate.dead(time.monotonic()):
                raise _Abort('BUSY', '机械臂正在复位，等它结束再动')
            if self._estop_on:
                raise _Abort('ESTOP', '急停还按着，不能动')

            ecfg = dataclasses.replace(
                self._ecfg, normal_t_ms=int(self.get_parameter('normal_arm_t_ms').value))
            st = {}
            code, ereason = motion_exec.run_motion(
                p, MotionLink(self), ecfg, stats=st, scfg=self._scfg,
                progress=lambda i, n, d: self._motion_feedback(gh, i, n, d))
            code = motion_exec.final_code(code, p.partial)
            if code in motion_exec.OK_CODES:
                self._history.commit(p.history_used, before)
            res.ok = code in motion_exec.OK_CODES
            res.code = code
            res.reason = motion_exec.compose_reason(code, summary, p.notes, ereason)
            self.get_logger().info('motion 结束：%s %s（发 %d 帧、重发 %d、未到位 %s）' % (
                code, res.reason, st.get('published', 0), st.get('resends', 0), st.get('misses')))
            res.state_json = self._state_json()
            if res.ok:
                gh.succeed()
            elif code == 'CANCELED':
                gh.canceled()
            else:
                gh.abort()
            return res
        except _Abort as a:
            res.ok, res.code, res.reason = False, a.code, a.reason
            self.get_logger().warn('motion 结束：%s %s' % (a.code, a.reason))
            res.state_json = self._state_json()
            if a.code == 'CANCELED':
                gh.canceled()
            else:
                gh.abort()
            return res
        except Exception as e:         # noqa: BLE001
            self.get_logger().error('motion 异常：%r' % (e,))
            res.ok, res.code, res.reason = False, 'ERROR', repr(e)
            gh.abort()
            return res
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = GraspServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.tof is not None:
            node.tof.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
