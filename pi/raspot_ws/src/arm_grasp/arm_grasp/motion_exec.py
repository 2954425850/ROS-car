# -*- coding: utf-8 -*-
"""语音动作的逐帧执行 + 状态描述（设计 §4.2 §8）。纯逻辑，不 import rclpy。

    run_motion(plan, link, cfg) -> (code, reason)

I/O 全部经 `Link`（服务端注入 ROS 实现，测试注入假臂）：

    link.fb()          -> [p1..p6] 或 None（最近一拍**可信且新鲜**的回读；没有就 None）
    link.publish(f)    发一帧 /arm/command（6 个 field）；人接管/急停后必须抛 Abort，不许发
    link.set_t_ms(ms)  设驱动 arm_t_ms（驱动每帧现读）；返回 True/False
    link.spin(dt)      睡 dt 秒，期间查打断，有打断抛 Abort(code, reason)；dt=0 = 只查一次
    link.now()         单调时钟（秒）

执行规则（设计 §8）：
  * 每帧：先 set_t_ms(本帧 t_ms)，再发**一次**目标（顺 LX 舵机"目标恒定 + 一条长轨迹最丝滑"）；
  * 到位 = 已过 t_ms 且回读与目标差 ≤ 容差连续 settle_n 拍（关节/腕旋 tol_joint、底座 tol_base；
    **夹爪不判到位**——会被物体挡住，只等 t_ms）；超时 t_ms+timeout_extra_ms 不报错，记残差，最终 PARTIAL；
  * 发出 resend_after_ms 后回读完全没朝目标动 ⇒ 以剩余时间重发一次（防串口丢帧），每帧最多 max_resend 次；
  * 不加任何积分/补偿：到不了就报残差（记忆 integrator-compensation-can-crush）；
  * CANCELED ⇒ 冻结：以当前回读为目标、t_ms=freeze_t_ms 发一帧；
  * ABORTED_BY_OPERATOR / ESTOP / 门关（复位中）⇒ **零**发布；
  * finally：t_ms 还原成 normal_t_ms（设过才还原）。
"""
import json
import math
from dataclasses import dataclass

from arm_grasp.arm_kin import BASE_SCALE, FACTOR, fk, from_fields

# field 下标 → 中文名（p1 夹爪、p2 腕旋、p3 腕俯仰、p4 肘、p5 肩、p6 底座）
JOINT_CN = ('夹爪', '腕旋', '腕俯仰', '肘', '肩', '底座')
_CHECK_IDX = (1, 2, 3, 4, 5)        # 判到位的 field（夹爪 0 不判）

OK_CODES = ('OK', 'PARTIAL')


class Abort(Exception):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code, self.reason = code, reason


# --------------------------------------------------------------------------- 配置

@dataclass
class ExecConfig:
    tol_joint: float = 8.0          # p2..p5 到位容差（field）
    tol_base: float = 6.0           # 底座到位容差（field，1000/360 每度）
    settle_n: int = 3               # 连续几拍在容差内算到位
    timeout_extra_ms: int = 1500    # 每帧超时 = t_ms + 这个
    poll_s: float = 0.04            # 回读 25Hz
    resend_after_ms: int = 250      # 发出后这么久回读还完全没动 ⇒ 重发
    max_resend: int = 1
    moved_eps: float = 3.0          # 朝目标挪了不到这么多 field 视为"没动"（回读底噪 ±1）
    resend_min_t_ms: int = 100      # 重发时剩余时间的下限
    freeze_t_ms: int = 100          # 取消时冻结帧的 t_ms
    normal_t_ms: int = 40           # 结束还原成这个（服务端用参数 normal_arm_t_ms 覆盖）
    fb_lost_s: float = 1.0          # 执行中连续这么久读不到回读 ⇒ NO_FEEDBACK


@dataclass
class StateConfig:
    grip_open_max: float = 300.0    # 读回 ≤ 这个 = 张开（张开指令 240）
    grip_closed_min: float = 545.0  # 读回 ≥ 这个 = 合上（空爪合到底读回 559，指令 578）
    grip_holding_min: float = 400.0  # (holding_min, closed_min) 之间且指令是合上 = 夹着东西
    grip_hold_gap_max: float = 120.0  # 读回比指令小 0..这个 ⇒ 认为是被挡住，保持指令值（不松爪）


def load_runtime_config(path=None):
    """motion.yaml 里的 exec: / state: 两节 → (ExecConfig, StateConfig)。缺的键用默认值。"""
    import yaml
    from arm_grasp.motion_spec import default_config_path
    path = path or default_config_path()
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}

    def build(cls, sec):
        d = raw.get(sec) or {}
        known = cls.__dataclass_fields__
        extra = set(d) - set(known)
        if extra:
            raise ValueError('motion.yaml 的 %s: 有不认识的键 %s（%s）' % (sec, sorted(extra), path))
        kw = {k: type(known[k].default)(v) for k, v in d.items()}
        return cls(**kw)

    return build(ExecConfig, 'exec'), build(StateConfig, 'state')


# --------------------------------------------------------------------------- 门 / 夹爪

class CmdGate:
    """`/arm/cmd_gate` 三态（语义抄 raspot_teleop/arm_gate.py，跨包不 import）：

    'live' 见过 1、当前 1、新鲜；'dead' 见过 1、当前 0、新鲜 ⇒ 正在复位，不许动；
    'unknown' 没收到 / 过期 / 从没见过 1（老驱动）⇒ 放行。
    """

    def __init__(self, stale_s=1.0):
        self.stale_s = stale_s
        self.live = None
        self.t = 0.0
        self.ever = False

    def on_msg(self, live, now):
        self.live = bool(live)
        self.t = now
        if self.live:
            self.ever = True

    def state(self, now):
        if self.live is None or (now - self.t) > self.stale_s or not self.ever:
            return 'unknown'
        return 'live' if self.live else 'dead'

    def dead(self, now):
        return self.state(now) == 'dead'


def grip_hold_value(p1_read, grip_cmd, scfg):
    """规划/冻结用的夹爪目标。

    回读比最近一次夹爪指令小一点（0..grip_hold_gap_max）= 爪被物体挡住（或空爪合到底 559 vs 578）
    ⇒ 用**指令值**，否则拿回读当目标会把夹着的东西松掉。其它情况（没有指令记录 / 对不上）用回读。
    """
    if grip_cmd is None:
        return float(p1_read)
    gap = float(grip_cmd) - float(p1_read)
    if 0.0 <= gap <= scfg.grip_hold_gap_max:
        return float(grip_cmd)
    return float(p1_read)


def gripper_state(p1_read, grip_cmd, scfg):
    """夹爪 张开 / 合上 / 半开 / 夹着东西。grip_cmd = 最近一次夹爪指令（None = 不知道）。"""
    p = float(p1_read)
    if p >= scfg.grip_closed_min:
        return '合上'
    if p <= scfg.grip_open_max:
        return '张开'
    if p > scfg.grip_holding_min and (grip_cmd is None or float(grip_cmd) >= scfg.grip_closed_min):
        return '夹着东西'
    return '半开'


# --------------------------------------------------------------------------- 状态描述

def _deg(v):
    return int(round(abs(v)))


def describe_state(fields, scfg, grip_cmd=None):
    """回读 [p1..p6] → state dict（含给模型念的中文 `text`）。arm/describe 与 Result.state_json 共用。"""
    f = [float(v) for v in fields]
    j = from_fields(f)
    az = j['base']
    el = j['shoulder'] + j['elbow'] + j['wrist_pitch']
    (x, y, z), _ = fk(j)
    g = gripper_state(f[0], grip_cmd, scfg)

    if abs(az) < 3:
        s_az = '正前方'
    else:
        s_az = '朝%s %d°' % ('左' if az > 0 else '右', _deg(az))
        if abs(az) > 90:
            s_az += '（在车后方那一侧）'
    if abs(el) < 3:
        s_el = '水平'
    else:
        s_el = '向%s %d°' % ('上' if el > 0 else '下', _deg(el))
    parts = ['车前 %dcm' % _deg(x) if x >= 0 else '车后 %dcm' % _deg(x)]
    if abs(y) >= 1:
        parts.append('偏%s %dcm' % ('左' if y > 0 else '右', _deg(y)))
    parts.append('高 %dcm' % int(round(z)) if z >= 0 else '比臂座低 %dcm' % _deg(z))
    text = '视线%s、%s；爪尖在%s；夹爪%s' % (s_az, s_el, '、'.join(parts), g)
    return {
        'fields': [round(v, 1) for v in f],
        'joints_deg': {k: round(v, 1) for k, v in j.items()},
        'gaze_deg': {'az': round(az, 1), 'el': round(el, 1)},
        'tip_cm': {'x': round(x, 1), 'y': round(y, 1), 'z': round(z, 1)},
        'gripper': {'p1': round(f[0], 1), 'state': g},
        'text': text,
    }


def state_json(fields, scfg, grip_cmd=None):
    return json.dumps(describe_state(fields, scfg, grip_cmd), ensure_ascii=False)


# --------------------------------------------------------------------------- 执行

def _tol(i, cfg):
    return cfg.tol_base if i == 5 else cfg.tol_joint


def _to_deg(i, d_field):
    return d_field / BASE_SCALE if i == 5 else d_field / FACTOR


def residual(fb, target, cfg):
    """超出容差的关节 → {下标: target-fb（field）}。夹爪不算。"""
    return {i: target[i] - fb[i] for i in _CHECK_IDX if abs(target[i] - fb[i]) > _tol(i, cfg)}


def residual_text(res):
    return '、'.join('%s差 %.0f°' % (JOINT_CN[i], abs(_to_deg(i, d))) for i, d in sorted(res.items()))


def _not_moved(start, fb, target, cfg):
    """所有需要动的 field（含夹爪）都没朝目标挪过 moved_eps ⇒ True（多半这帧被串口丢了）。"""
    need = [i for i in range(6) if abs(target[i] - start[i]) > cfg.moved_eps]
    if not need:
        return False
    return all((fb[i] - start[i]) * math.copysign(1.0, target[i] - start[i]) <= cfg.moved_eps
               for i in need)


def run_motion(plan, link, cfg, progress=None, stats=None, scfg=None):
    """逐帧执行 plan.keyframes。返回 (code, reason)；code ∈ OK/PARTIAL/NO_FEEDBACK/CANCELED/
    ABORTED_BY_OPERATOR/ESTOP/BUSY/ERROR（规划阶段的截断由调用方并进来，这里只管执行）。

    stats（可选 dict）回填：published / resends / misses（[(帧号, 残差 dict)]）/ frames_done。
    progress（可选）：progress(step, total, detail)。scfg：StateConfig（冻结帧的夹爪保持判据）。
    """
    scfg = scfg or StateConfig()
    st = stats if stats is not None else {}
    st.update(published=0, resends=0, misses=[], frames_done=0)
    frames = plan.keyframes
    n = len(frames)
    t_set = None           # 已设给驱动的 t_ms（None = 还没动过）
    last_sent = None       # 最近一帧发出去的目标

    def set_t(ms):
        nonlocal t_set
        ms = int(ms)
        if t_set == ms:
            return True
        ok = link.set_t_ms(ms)
        if ok:
            t_set = ms
        return ok

    def send(fields):
        link.spin(0)                       # 发之前最后查一次打断
        link.publish(list(fields))
        st['published'] += 1

    try:
        for k, kf in enumerate(frames):
            target = [float(v) for v in kf.fields]
            start = link.fb()
            if start is None:
                if k == 0:
                    return 'NO_FEEDBACK', '读不到可信的机械臂回读，没动'
                return 'NO_FEEDBACK', '执行到第 %d/%d 帧时回读丢了' % (k + 1, n)
            if progress:
                progress(k + 1, n, kf.label)
            if not set_t(kf.t_ms):
                # 设不了 t_ms 就会按驱动默认 40ms 猛冲过去 —— 宁可不动
                return 'ERROR', '设不了驱动的 arm_t_ms（参数服务不在？），没敢动'
            t0 = link.now()
            send(target)
            last_sent = target
            resent = 0
            good = 0
            fb_t = t0
            t_ms = kf.t_ms
            deadline = t0 + (t_ms + cfg.timeout_extra_ms) / 1000.0
            while True:
                link.spin(cfg.poll_s)
                now = link.now()
                fb = link.fb()
                if fb is None:
                    if now - fb_t > cfg.fb_lost_s:
                        return 'NO_FEEDBACK', '执行到第 %d/%d 帧时回读丢了' % (k + 1, n)
                    continue
                fb_t = now
                el_ms = (now - t0) * 1000.0
                if (resent < cfg.max_resend and el_ms >= cfg.resend_after_ms
                        and _not_moved(start, fb, target, cfg)):
                    rest = max(cfg.resend_min_t_ms, int(t_ms - el_ms))
                    if not set_t(rest):
                        return 'ERROR', '重发前设不了驱动的 arm_t_ms，没敢再发'
                    send(target)
                    resent += 1
                    st['resends'] += 1
                    continue
                good = good + 1 if not residual(fb, target, cfg) else 0
                if good >= cfg.settle_n and el_ms >= t_ms:
                    break
                if now >= deadline:
                    st['misses'].append((k, residual(fb, target, cfg)))
                    break
            st['frames_done'] = k + 1
        if st['misses']:
            return 'PARTIAL', miss_text(st['misses'], n)
        return 'OK', ''
    except Abort as a:
        if a.code == 'CANCELED':
            fb = link.fb()
            if fb is not None and last_sent is not None:   # 一帧都没发过 ⇒ 臂本来就没动，不发
                frz = [float(v) for v in fb]
                # 夹爪：被物体挡住时保持指令值，别借冻结把东西松掉
                frz[0] = grip_hold_value(fb[0], last_sent[0], scfg)
                try:
                    link.set_t_ms(int(cfg.freeze_t_ms))
                    t_set = int(cfg.freeze_t_ms)
                    link.publish(frz)
                    st['published'] += 1
                    st['freeze'] = frz
                except Abort:
                    pass                   # 冻结时又来了人接管/急停 ⇒ 不发
            return 'CANCELED', a.reason or '取消了'
        return a.code, a.reason
    finally:
        if t_set is not None:
            try:
                link.set_t_ms(int(cfg.normal_t_ms))
            except Exception:               # noqa: BLE001 —— 还原失败只能放过
                pass


def miss_text(misses, n):
    """[(帧号, 残差)] → 中文。最后一帧没到 = 臂没停在要的位置，单独说。"""
    last = [r for k, r in misses if k == n - 1]
    mid = [k for k, _r in misses if k != n - 1]
    out = []
    if last and last[0]:
        out.append('没完全到位：' + residual_text(last[0]))
    elif last:
        out.append('最后一帧没等到稳定')
    if mid:
        out.append('中间有 %d 帧没走到位' % len(mid))
    return '；'.join(out)


# --------------------------------------------------------------------------- 文案

def _amt(m, unit):
    if m.number is not None:
        return '%g%s' % (m.number, unit)
    return m.amount or ''


def spec_summary(spec):
    """Spec → 一句中文动作摘要（给 reason 用）。"""
    steps = []
    for s in spec.steps:
        parts = []
        if s.toward:
            parts.append('回到上一个姿态' if s.toward == '上一个姿态' else '看向' + s.toward)
        for m in s.turn:
            parts.append('往%s转%s' % (m.dir, _amt(m, '°')))
        for m in s.shift:
            parts.append('往%s挪%s' % (m.dir, _amt(m, 'cm')))
        if s.approach:
            parts.append('%s%s' % (s.approach.dir, _amt(s.approach, 'cm')))
        if s.gripper:
            parts.append('夹爪' + s.gripper)
        if s.oscillate:
            o = s.oscillate
            word = {'上下': '点头', '左右': '摇头', '夹爪': '夹爪开合'}[o.axis]
            parts.append(word + ('×%d' % o.times if o.times else ''))
        steps.append('+'.join(parts))
    text = '，然后'.join(steps)
    if spec.repeat > 1:
        text += '（重复 %d 遍）' % spec.repeat
    return text


def compose_reason(code, summary, notes, exec_reason=''):
    """最终 reason：动作摘要 + 规划 notes + 执行情况。"""
    head = {
        'OK': '已完成：%s' % summary,
        'PARTIAL': '做了：%s' % summary,
        'CANCELED': '取消了（%s），臂停在当前位置' % summary,
        'ABORTED_BY_OPERATOR': '手柄接管了，动作中止（%s）' % summary,
        'ESTOP': '急停，动作中止（%s）' % summary,
    }.get(code, '没做成（%s）' % summary)
    tail = [t for t in list(notes) + [exec_reason] if t]
    return head + ('。' + '；'.join(tail) if tail else '')


def final_code(exec_code, plan_partial):
    """执行 OK 但规划有截断 ⇒ PARTIAL。"""
    if exec_code == 'OK' and plan_partial:
        return 'PARTIAL'
    return exec_code


class PoseHistory:
    """「动作前姿态」栈（最多 maxlen 个，最后一项最近）。按 Plan.history_used 弹出，
    动作成功（OK/PARTIAL）后压入动作前姿态。"""

    def __init__(self, maxlen=5):
        self.maxlen = maxlen
        self.items = []

    def snapshot(self):
        return [list(x) for x in self.items]

    def commit(self, used, before):
        for _ in range(min(int(used), len(self.items))):
            self.items.pop()
        self.items.append([float(v) for v in before])
        del self.items[:-self.maxlen]

