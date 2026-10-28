# -*- coding: utf-8 -*-
"""机械臂"现在能不能操作"的门：``/arm/cmd_gate`` 的判定。

驱动每拍发一个 Bool（1 = 现在可以给臂下 0xAC）。这一位已经由驱动合过两件事：
固件自报的 control-live（0x56 byte19，INIT_* 期间不消费 0xAC）和"本节点刚下发
过 re-home"的复位窗口（烧录版固件可能压根不报这个位）。本模块只做判定，不碰 ROS。

为什么需要它（2026-10-28 现场）：START=回中会在固件里跑一套和上电相同的
复位流程（五关节 → 复位位姿、底座重锚回中，约 5~15 秒）。这期间：
  · 固件**不消费** 0xAC（驱动也在丢弃 arm/command）⇒ 手部输入没反应；
  · ``/arm/feedback`` 是固件总线调试计数器（假值），不是关节角。
旧 teleop 对此一无所知：照样把摇杆积分进 targets、把假回读同步进 targets，
复位一结束第一帧就带着这些目标发出去 ⇒ **臂突然甩一下**。
门关着的时候 teleop 不积分、不发布；一旦重新"verified live"，就要"按真实回读
重设目标"，对应 ``step()`` 的 ``resumed``。

三态（宁可放行也不要把臂锁死）：
  · 'live'    收到过 1 且当前是 1、且话题新鲜   → 放行（verified）
  · 'dead'    收到过 1、当前是 0、且话题新鲜     → 门住
  · 'unknown' 没收到过 / 超过 stale_s 没更新 / 从没见过 1（旧驱动不发这个话题）
              → 放行，但**不算 verified**（老驱动行为和以前完全一样）
"""


class CmdGate:

    def __init__(self, stale_s=1.0):
        self.stale_s = stale_s
        self.live = None          # 最近一次收到的值（None = 还没收到过）
        self.t = 0.0              # 收到时刻（monotonic）
        self.ever = False         # 见过 1 吗
        self.prev_verified = None  # 上一次 step() 是否 verified（None = 还没跑过）

    def on_msg(self, live, now):
        self.live = bool(live)
        self.t = now
        if self.live:
            self.ever = True

    def state(self, now):
        if self.live is None or (now - self.t) > self.stale_s:
            return 'unknown'      # 没数据 / 驱动重启：不能判
        if not self.ever:
            return 'unknown'      # 老固件不报这一位：永远不要门
        return 'live' if self.live else 'dead'

    def step(self, now):
        """每拍调一次，返回 ``(allowed, resumed)``。

        allowed: 这一拍能不能积分/发布臂指令。
        resumed: 刚刚进入 verified live（含第一次 verified）——要按真实回读重设目标。
        """
        st = self.state(now)
        allowed = st != 'dead'
        verified = st == 'live'
        resumed = verified and not self.prev_verified
        self.prev_verified = verified
        return allowed, resumed
