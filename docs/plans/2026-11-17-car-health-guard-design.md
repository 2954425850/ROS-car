# 整车健康守护（carhealth）设计

> 2026-11-17 · 状态：已定稿（用户确认架构、契约、恢复阶梯、分期；
> 2026-11-17 按评审意见修订：故障分类库、直接证据归因、配置漂移、
> 迟滞、UNKNOWN 语义、RTC/写盘、覆盖缺口）
>
> 目标：把「各部分总是莫名其妙出故障、要靠人实时改」变成
> 「故障可判定、可归因、可对症自愈，人只处理真正的硬件/环境问题」。

---

## 1. 背景

现有 systemd 单元几乎都是 `Restart=always`，但它只覆盖「进程死了」这一种故障。
仓库里实际发生过的故障按**进程是否还活着**分类：

| 故障类别 | 实例 | `Restart=always` 可见 |
|---|---|---|
| 环境根因 | Pi 掉压 → USB 掉 → 臂/底盘串口掉线；ToF 重枚举后 ttyUSB 号漂移 | 只见下游症状 |
| 对端假死 | K230 `machine.reset()` 不发 FIN，TCP 假连 | 否 |
| 网络前提 | 换网段后 DDS 有发现但收不到数据；2.4G 高延迟 | 否 |
| 协议/应用 | 语音 API 400 后「卡死」；工具调用历史残缺 | 否 |
| 资源 | 内存/CPU/温度/磁盘 | 否 |
| 真实崩溃 | 节点抛异常退出 | 是 |

八类中只有最后一类被覆盖。因此现场表现为：**大多数故障在 systemd 眼里「没病」，
只能靠人在现场发现并手动修**。

仓库里已有若干正确的局部做法（`/tof/status` 的 JSON 契约、`k230pi.py` 的
静默超时丢连接、`wakeword/engine.py` 的退避重连、`k230_camera.sh` 的断流重连、
`k230ctl status` 的 mtime 新鲜度），但没有统一点位、没有跨组件归因、
没有安全边界。本设计把这些经验收敛成一个独立守护。

## 2. 目标与非目标

### 目标

1. **判定**：对每个组件区分 `OK / DEGRADED / STALE / ABSENT / BLOCKED / UNKNOWN`，
   判据是存活 + 新鲜度 + 正确性 + 环境前提，不只是进程状态。
2. **归因**：用直接证据找根因（掉压、板子离线、发布者数量为 0、DDS 绑着旧 IP），
   不靠「症状数量」推断；下游症状不各算各的故障。
3. **对症恢复**：先判故障类别，再选动作类别——重启单元 / 等条件恢复 /
   给人一句物理动作 / 登记为配置缺陷。分级阶梯只是「软件状态坏了」这一类的子集，
   **动作中一律不动手**。
4. **取证**：结构化故障日志，能回答「刚才为什么坏了、它做了什么、为什么没修好」。
5. **不成为新故障源**：只读探针、ROS 不可用时降级、自身被 systemd 看护。
6. **体检**：出车前一条命令拿到 PASS/WARN/FAIL 与可执行建议。
7. **已知故障库**：把历史故障的特征 → 结论 → 动作类别 → 给人的指令写成规则，
   `doctor/status/why` 直接输出「是什么、该做什么」，而不只是一个 `STALE`。

### 非目标（明确不做，防止日后被顺手加上）

- 自动重启 Pi、自动重启/断电 K230、重新配对 PS2、重刷固件。
- 自动执行抓取、自动发臂指令、自动移动底盘。
- 自动删除/覆盖文件、自动改配置。
- 替代硬件修复：掉压是供电问题、K230 固件卡死需板侧看门狗、PS2 首开失灵需重新配对。
  本系统负责**识别、隔离、取证**，不负责让硬件故障消失。
- 云端告警/手机推送（本期只做本地日志；架构预留，后续可加）。

## 3. 方案选型

| 方案 | 说明 | 结论 |
|---|---|---|
| A. 中央分诊守护 `carhealthd` | 独立进程，声明式契约采集 + 统一归因 + 分级恢复 | **终态** |
| B. 各节点就地自愈 + 中央只展示 | 无单点，但口径分散、跨层/环境根因无人管，且与现有分散逻辑重复 | 不采用；其重连经验被吸收 |
| C. 只做体检 + 取证 | 一两天可交付、零风险，但解决不了「不用人实时改」 | **作为第一期** |

**决策：以 A 为终态，按 C → A 分三期交付**。任何一期停下都不会产生自动动作误伤。

## 4. 架构与部署

```
pi/carhealth/
  carhealthd          # 常驻守护（系统级 systemd，User=cy，Restart=always）
  carhealthctl        # CLI：status / doctor / history / why <组件>
  contracts.py        # 组件清单（声明式）：探针、阈值、依赖、允许动作
  probes/
    systemd.py        # 批量 systemctl show ActiveState/SubState/NRestarts/MemoryCurrent/CPUUsageNSec
    ros_topic.py      # 只订阅 rclpy 节点；记录年龄、发布者数量；Image/IMU 用 raw=True
    file_status.py    # status JSON + mtime 新鲜度（沿用 k230ctl 语义）
    net.py            # TCP 探活 / ping / 端口，不拉流；地址未知时返回 UNKNOWN
    udev.py           # 稳定名存在性；优先用 udevadm 读 ID_USB_INTERFACE_NUM
    host.py           # loadavg、/proc/meminfo、磁盘、温度、wifi 信号
    power.py          # vcgencmd get_throttled（欠压/降频/软温度位）
    service_health.py # NRestarts 增量（崩溃循环）、内存占用
    usb_events.py     # 内核日志里的 USB disconnect 计数（按 vid:pid）
    dds.py            # ss -uan 里的本机绑定地址 vs 当前网卡地址（旧 IP 检测）
    config_drift.py   # ordering cycle、should-be-enabled、udev 规则、git HEAD、build 过期
  classify.py         # 状态机 + 迟滞 + 根因挂载（基于直接证据）
  faults.py           # 已知故障库：特征 → 结论 + 动作类别 + 给人的指令
  diagnose.py         # 把探针证据喂给故障库，产出 Diagnosis
  recovery.py         # 阶梯/白名单/联锁/熔断（二期启用）
  busy.py             # 动作互锁信号
  journal.py          # JSONL 故障日志 + 滚动（带 boot_id / monotonic）
```

部署约束：

- 系统级 systemd 单元以 `cy` 运行；**不通过 `systemctl --user` 读取用户单元**，
  voice 用心跳文件判活（避免权限与 XDG_RUNTIME_DIR 依赖）。
- 探针一律只读：不打开串口、不拉视频流、不起 `ros2 topic echo` 子进程。
- rclpy 不可用时只跑非 ROS 探针，相关组件标 `UNKNOWN`，进程不退出。
- 守护自身由 `Restart=always` 看护（二期加 `WatchdogSec`/sd_notify）；自重启写日志。
- **state.json 写在 tmpfs**：单元加 `RuntimeDirectory=carhealth`，状态文件在
  `/run/carhealth/state.json`，不磨损 SD 卡；journal 才落盘到
  `/home/cy/.local/state/carhealth/journal-YYYYMMDD.jsonl`，按天滚动保留 30 天。
- `doctor` 是只读一次性检查，永远用临时目录，绝不覆盖守护的 state/journal。
- **高频话题不进守护**：2026-11-18 Pi 实测，carhealth 直接订阅
  `/tof/depth`(30Hz)、`/imu`(50Hz)、`/arm/feedback`(25Hz)、`/wheel_odom`(50Hz)
  等话题时，rclpy 每条消息的固定开销让守护稳定吃 28% CPU；只去掉三个后仍有
  18%。最终由驱动节点新增 1Hz `/health/driver` 摘要（含臂/底盘/电池/命令/里程计
  的最近值与年龄），守护只订阅 `/health/driver`、`/tof/status`、
  `/health/arm_grasp`、`/k230/result`，实测 **1.0% CPU / 71MB RSS**。
  这也是 `/tof/status` 契约模式的推广：节点自己汇总，守护只读摘要。

## 5. 健康契约

每个组件按同一形状声明，示例（ToF）：

```
component  tof
probe      systemd: tof-a010.service ActiveState=active
probe      topic: /tof/status age<2s, connected=true, fps>0
probe      topic: /tof/depth age<2s（raw=True，只记到达时间）
probe      udev: /dev/a010 存在；ID_USB_INTERFACE_NUM=="00"
           若无 /dev/a010 但 by-id 的 *-if00-port0 在 → DEGRADED「udev 规则未装」
blocked_by power
recovery   restart tof-a010.service              # 静态、幂等、空闲可做
```

`/tof/status` 现有字段（`connected/connecting/frames/fps/reconnects/age_s/
device/port/last_error/frame_id`）即契约模板，新增心跳沿用同一形状。
**注意：`tof_a010` 重连成功后不清 `last_error`，所以它不能当判据，只进证据。**

### 组件清单与初值阈值（现场标定）

| 组件 | 存活 | 新鲜度/正确性 | 环境前提 | 互锁来源 | 允许恢复 |
|---|---|---|---|---|---|
| `power` | — | 当前欠压 / 进程内观察到 10 分钟内出现过欠压 / 开机以来历史位；bit1–3（降频、throttle、软温度） | — | — | 无 |
| `battery` | `/health/driver` <3s | `battery_voltage` <11.5V → DEGRADED；<11.2V → 规则提示「性能明显下降」 | — | — | 无 |
| `chassis` | ps2-teleop 单元 | `/health/driver` <2s；`telemetry_age_s` <1s；`chassis_faults` 非零 → DEGRADED；`/dev/input/js0` 存在 | `/dev/l150pro` | 底盘在动 | 空闲时重启 ps2-teleop |
| `arm` | 同上（同一串口） | `/health/driver` 的 `feedback_age_s` <0.5s；`arm_online`=true；`arm_estop`=false | `/dev/l150pro` | 臂在动 / 抓取在跑 | 空闲时重启；动作中只记录 |
| `tof` | tof-a010 单元 | `/tof/status` 契约；`/tof/depth` <2s（raw）；udev 接口号 00 | `/dev/a010` 或 by-id `if00` | 抓取中 | 重启单元 |
| `grasp` | arm-grasp 单元 | `/health/arm_grasp` <3s；心跳带 `busy`、`feedback_age_s`、`last_goal_wall` | — | 抓取在跑 | 空闲时重启 |
| `k230-link` | resultd/bridge 单元 | `resultd-status.json` mtime<7s；`/k230/result` <3s；resultd 的 `connected`/`peer` | 板子地址可知（否则 UNKNOWN） | — | 重启 resultd / bridge |
| `k230-video` | k230-framed 单元 | framed 状态文件 <3s；最新 IDR <5s | 板子在线 | — | 重启 framed |
| `rosbridge` | 单元 | TCP 127.0.0.1:9090 | — | — | 重启单元 |
| `voice` | 心跳文件 mtime<5s | `busy_for_s`<180；`wakeword_ready`；`wakeword_thread_alive`；`mic_open`；`asr_last_ok/err` 年龄 | 网络、cookie 未过期 | 工具执行中 | 空闲时重启 |
| `host` | — | 可用内存、swap、磁盘、SoC 温度、load1、CPU%、WiFi 信号 | — | — | 无 |

不进组件、只喂故障库的探针：`service_health`（NRestarts 增量、内存、单服务 CPU%）、
`usb_events`（USB disconnect 计数）、`dds`（旧 IP 绑定）、`config_drift`
（ordering cycle、enabled 但 inactive、udev 规则、git HEAD、build 过期）。

### 判定规则

- 迟滞：**连续 2 次坏才降级，连续 5 次好才恢复**；`engine.py` 必须实现，
  否则 1 秒轮询 + 0.5 秒阈值会在一次 CPU 尖峰（Pi 实测 load 17.8）里刷满日志。
- 状态语义：`BLOCKED` 不是故障，是「环境根因压住了，等条件恢复」。
- **BLOCKED 不得掩盖健康组件**：先看组件自己的判据。自己全部通过 → `OK`；
  自己失败或 `UNKNOWN` 且依赖项 `BLOCKED` → 才标 `BLOCKED` 并挂 `root_cause`。
  欠压期间底盘完全正常，就不该显示 BLOCKED。
- **UNKNOWN 不是故障**：探针失明（rclpy 不可用、无权限、字段读不到）时，
  只记一条限流的 `monitor_blind` 事件，不进入组件 incident 时间线。
- **根因用直接证据**：
  - 欠压：当前 `uv_now`；或进程**观察到** 10 分钟内出现过欠压。
    开机历史位只算 DEGRADED，不算 BLOCKED——医生每次新建探针，
    否则开机后任何一次掉压都会让 doctor 永远 FAIL。
  - ROS 图：对每个失败的话题查 `node.count_publishers(topic)`。
    `0` → 发现层故障（`ros_discovery`，通常要整条 ROS 栈重启 / 换网络）；
    `>0` 但无数据 → 数据通路/QoS/MTU（`ros_data_path`）。
  - DDS 旧 IP：`ss -uan` 里本机绑定地址不在当前网卡地址集合中 → `dds_stale_ip`。
  - 板子不可达：有地址才 ping；地址未知（resultd 无 peer 且无 `K230_HOST`）→ `UNKNOWN`，
    **禁止回退到任何写死的旧 IP**。
- 时间：年龄一律用 `time.monotonic()`；journal 每条同时记 `boot_id` 与 `mono`。
  Pi 无 RTC，墙钟在 NTP 对时前不可信，`history --since` 必须能用 boot/mono 校正。

## 6. 分诊、已知故障库与恢复阶梯

**先分类，再决定动作类别。** 大多数故障的正确出路不是重启：

| 故障类别 | 例子 | 正确出路 |
|---|---|---|
| 软件状态坏了 | 节点卡住、TCP 假连 | 重启；DDS 钉旧 IP 要整条 ROS 栈一起重启 |
| 环境前提不满足 | 欠压、板子离线、电池低 | 等条件恢复，不动手 |
| 只能人做的物理操作 | CP2102 USB 锁死、ToF 拖垮 K230 WiFi、换主控后 serial 变了 | 给一句明确指令（拔哪根线、换哪条规则） |
| 配置/设计缺陷 | ordering cycle、云端空闲 20Hz 发零、JWT 60 分钟过期、改了 .py 没 build | 运行时修不了，登记进待修清单，从根上修 |

### 已知故障库（一期就交付）

`faults.py` 把「特征 → 结论 + 动作类别 + 给人的指令」写成纯规则；
`diagnose.py` 用探针证据求值；`status/doctor/why` 直接展示结论与指令。
首批规则（全部来自本项目历史故障）：

| id | 特征 | 结论 / 动作类别 | 给人的指令 |
|---|---|---|---|
| `power.probe_unavailable` | `vcgencmd` 读不到 `/dev/vcio`（Pi 5 实测：cy 不在 video 组） | 配置 | `sudo usermod -aG video cy` 后重新登录并重启 carhealthd |
| `power.undervoltage` | `uv_now` 或进程观察到近期欠压 | 等 / 查供电 | 检查电源线、接插件、驱动板压降；换 5V/5A 线 |
| `usb.disconnect_storm` | 10 分钟内同一 `vid:pid` disconnect ≥3 次 | 物理 | 换线/换口/检查供电；K230 24h 13 次重枚举属此类 |
| `udev.l150pro_missing` | `/dev/l150pro` 不在，但其它 tty 有 `1a86:55d4` | 配置 | 装/改 `99-l150pro.rules`，确认 serial 匹配 |
| `udev.tof_byid_only` | `/dev/a010` 不在，by-id `*-if00-port0` 在 | 配置 | 装 `99-tof-a010.rules`（功能不受影响，只是短名丢了） |
| `udev.tof_channel_b` | 接口号 ≠ 00 | 物理/配置 | 插回通道 A；只按 vid/pid 匹配会抢名 |
| `dds.stale_ip` | `ss -uan` 有本机不存在的绑定 IP | 重启整栈 | `systemctl restart` ROS 相关单元；不要只重启一个 |
| `ros.discovery` | 失败话题 `count_publishers==0` 且单元 active | 重启整栈 / 查网络 | 先看 `ROS_DOMAIN_ID`、网卡、DDS 发现；再整栈重启 |
| `ros.data_path` | `count_publishers>0` 但无数据 | 查 QoS/MTU/网络 | 检查 QoS 兼容、MTU 分片、WiFi 质量 |
| `service.crash_loop` | 某单元 `NRestarts` 在 60s 内涨 ≥2 | 配置/物理 | 看 `journalctl -u`；这是被 `Restart=always` 掩盖的崩溃循环 |
| `service.ordering_cycle` | 本次开机 journal 有 ordering cycle | 配置 | 修 unit 的 `After/Wants`；`arm-grasp` inactive (dead) 常见于此 |
| `service.enabled_but_dead` | 应 enabled 的单元处于 inactive (dead) 且非人为 stop | 配置 | 查失败原因；`systemctl status` |
| `build.stale` | 源码 `package.xml/setup.py` 比 install 里的新 | 配置 | `colcon build --symlink-install`，再重启服务 |
| `k230.host_unknown` | 无 peer 且无 `K230_HOST` | 等 | 确认板子上电/连 WiFi；Pi 重启后 /tmp 清空属正常 |
| `k230.push_stall` | framed 连着但 IDR 年龄超阈值 | 物理/网络 | 看 K230 推流与 WiFi；K230 不能主动请求 IDR |
| `battery.low` | 电压 <11.2V | 等/换电 | 电压 11.2–12.3V 性能差近一倍，别误判成控制环路 |
| `voice.wakeword_lost` | `wakeword_ready=false` 或线程不在 | 物理 | 唤醒模块被拔/重枚举；按路径重连已实现，仍不行就换口 |
| `voice.mic_error` | 心跳带 `mic_last_error` | 物理 | USB 重枚举/设备被抢占；换口或重启语音助手 |
| `voice.api_stuck` | `busy_for_s`>180 | 软件 | 看 ASR/LLM 报错；这是「400 后卡死」的历史形态 |
| `voice.asr_error` | 心跳带 `asr_last_error` | 网络/软件 | 常见是 `status=44 request timeout`；先查网关 RTT/WiFi |
| `host.gateway_latency` | 网关 RTT 平均 >50ms 或最大 >200ms 或丢包 ≥20% | 物理/网络 | 信号满格也会发生；**先关 WiFi 省电**（`iwconfig wlan0 power off`），再换 5G/AP 或改有线 |
| `tof.last_error_stale` | `connected=true` 但 `last_error` 非空 | 证据 | 只提示，不判故障（该字段重连后不清零） |

规则是纯函数，输入是 `{probe_name: ProbeResult}` + 组件判定 + 当前时间；
单测用固定证据表。规则命中写进 `Diagnosis`，进 state/journal/CLI，
**不影响组件的六态判定**。

### 恢复阶梯（仅「软件状态坏了」这一类）

| 级别 | 动作 | 前提 | 适用 |
|---|---|---|---|
| L0 | 只观察 | 刚异常 / `BLOCKED` / 动作中 | 全部 |
| L1 | 重启静态节点 | 不持设备、不参与运动 | rosbridge、k230-resultd、k230-bridge、k230-framed |
| L2 | 重启持设备节点 | 互锁全清 + 设备节点在 | tof-a010、ps2-teleop、arm-grasp、voice |
| L3 | 按依赖顺序重启组 | L2 失败且组内多组件异常 | ps2-teleop → tof-a010 → arm-grasp |
| L4 | 升级给人 | 熔断打开 / 预算耗尽 | 写 `NEEDS_HUMAN` |

- 验证窗口：动作后 15 秒内未回 `OK` 则升一级；仍失败进熔断。
- 冷却：同组件同动作间隔 ≥30s；每 incident 每级最多 2 次。
- 全局限流：10 分钟内自动重启总数 ≤6。
- 开机后前 60 秒不执行任何恢复动作。
- **动作前先写日志、后执行**，保证动作本身导致二次故障时证据仍在。
- **「整条 ROS 栈重启」是独立动作类别**，不塞进 L1/L2：DDS 绑旧 IP 或发现层
  故障时只重启一个单元修不好。二期实现为 `restart_stack`，动作前同样检查互锁。

### 熔断

- 故障指纹 = `hash(组件 + 状态 + 根因 + 证据特征)`。
- 同一指纹 30 分钟内复发 ≥3 次 → 打开熔断：该组件 30 分钟内不再自动动作，
  标 `ESCALATED`，日志记 `NEEDS_HUMAN`。
- 稳定 30 分钟后自动复位。

## 7. 动作互锁

恢复权限的唯一开关。任一为真即只记录不动手：

| 信号 | 判据 |
|---|---|
| 臂在动 | `/health/driver` 的 `arm_cmd_age_s` <1.5s，或抓取在跑 |
| 底盘在动 | `/health/driver` 的 `cmd_vx/cmd_wz`（命令）或 `odom_vx/odom_wz`（实测）非零，且对应年龄新鲜 |
| 抓取在跑 | `/health/arm_grasp` 的 `busy=true`（由 `GraspServer._busy` 生成） |
| 语音在忙 | heartbeat 的 `busy` 字段（THINKING / SPEAKING / 工具执行中） |

缺信号的常驻节点在第一期补 1Hz 心跳，格式沿用 `/tof/status` 的 JSON 形状：

- `voice`：独立线程写文件，字段包括 `state/state_age_s/busy/busy_for_s`、
  `wakeword_ready`、`wakeword_thread_alive`、`mic_open`、`mic_last_error`、
  `asr_last_ok_wall`、`asr_last_err_wall`。
  只看线程心跳会漏掉「主流水线死锁但心跳线程还在跳」和「停在 IDLE、喊它没反应
  （USB 重枚举后麦克风/唤醒模块掉线）」；必须带上设备与引擎状态。
- `grasp`：`/health/arm_grasp` 带 `busy`、`feedback_age_s`、`last_goal_wall`。
  定时器心跳只能证明 executor 活着，不能证明 action 在推进；
  所以还要看 `/arm/feedback` 是否新鲜。
- `k230-framed`：`framed-status.json` 带 `connected/peer/bytes/idr_frames`，
  每秒写；断连时也写。

## 8. 故障日志与 CLI

只记状态变化与动作，不记每次轮询。示例：

```json
{"ts":"2026-11-17T14:02:11.2","boot_id":"3f2c...","mono":8412.5,
 "incident":"I-20261117-140211-tof","component":"tof",
 "from":"OK","to":"STALE","verdict":"STALE",
 "evidence":{"topic:/tof/status":{"age_s":4.2,"fps":0.0,"publishers":1,
                                 "reconnects":7,"last_error":"read timeout"},
             "udev:a010":{"exists":true,"interface":"00"}},
 "root_cause":null,"fingerprint":"tof:stale:timeout:a1c9",
 "diagnosis":[{"id":"usb.disconnect_storm","action_class":"human",
               "hint":"ToF 10 分钟内重枚举 3 次：换线/换口/查供电"}],
 "action":null,
 "interlock":{"arm":false,"chassis":false,"grasp":false,"voice":false}}
```

- 根因事件单独成条（`power`、`board`、`dds`、`config_drift`），下游 incident
  通过 `root_cause` 引用，避免一次掉压产生七条独立告警。
- **每条都带 `boot_id` 与 `mono`**：Pi 无 RTC，NTP 对时前墙钟可能错。
  `history --since` 对当前 boot 的事件用 `mono` 过滤，跨 boot 的事件才用墙钟。
- `state.json` 在 `/run/carhealth/`；journal 在 `~/.local/state/carhealth/`。

CLI：

```
carhealthctl status            # 表格：组件/状态/持续/根因/结论 + 指令
carhealthctl status --json
carhealthctl doctor            # 出车体检：PASS/WARN/FAIL + 人话指令
carhealthctl history --since 1h
carhealthctl why tof           # 最近一次异常的完整证据 + 命中的故障规则
```

`doctor` 分两段：
1. **运行时**：若 `carhealthd` 正在跑且 state.json <10s，直接读它的结果，
   不再自己起 ROS 节点（FastDDS 在这台 Pi 上发现慢，3 秒容易误判）；
   守护没在跑才自建探针，warmup 默认 8 秒。
2. **配置漂移**：ordering cycle、should-enabled 但 inactive、udev 规则是否安装、
   git 分支/HEAD、build 是否比源码旧、USB 重枚举计数、时间同步、磁盘。

`doctor` 永远用临时目录，不覆盖守护状态。

## 9. 分期与验收

### 第一期：只观察，不动手

交付：`carhealth` 包、全部探针、分类器与**迟滞**、已知故障库（`faults.py`）、
配置漂移体检、JSONL 日志（boot_id/mono）、CLI、三个节点的心跳、
以 `recovery=off` 常驻的 systemd 单元、单测与故障注入脚本。

覆盖除六态判定外还必须包含：主机 load/CPU/内存/温度/WiFi、各单元
`NRestarts`/内存/CPU、USB disconnect 计数、DDS 旧 IP、`/battery_voltage`、
`/chassis_faults`、`/arm/status`、PS2 `js0`、配置漂移。

验收（硬性）：

1. 跑 2–3 次真实使用，每次掉数据/掉连接都能在日志中找到带证据的 incident，
   且 `why` 能给出结论与指令，而不只是 STALE。
2. `doctor` 对人为故障全部判对：拔 ToF、`systemctl stop k230-bridge`、
   伪造欠压位、K230 IP 指向不可达、杀掉 voice、伪造旧 IP 绑定、
   制造 build 过期。
3. 欠压历史位**不得**让 doctor 永久 FAIL；欠压期间健康的底盘/臂**不得**显示 BLOCKED。
4. 单次坏采样不开 incident（迟滞生效）；UNKNOWN 不产生组件 incident。
5. 全程自动动作次数 = 0。

### 第二期：自愈层（2026-11-18 已实现）

交付（现场按真实故障收敛，而不是全量阶梯）：

- `recovery.py`：白名单动作 + 互锁 + 冷却（120–600s）+ 全局预算（10 分钟 3 次）
  + 动作后 30s 验证 + 连续失败 2 次熔断 30 分钟。
- `CARHEALTH_RECOVERY=on` 启用；`carhealthctl recovery off` 写
  `/run/carhealth/recovery-off` 作为 kill switch。
- 特权动作统一走 root-owned `/usr/local/sbin/carhealth-recover`，
  sudoers 只放行这一条命令；守护本身仍以 `cy` 运行。
- 首批动作（全部来自现场故障）：
  1. `wifi_powersave_off`：WiFi 省电 on → `iwconfig wlan0 power off`（不重启，忙时也允许）
  2. `ros_daemon_stop`：仅 DDS 旧 IP socket 残留、组件健康 → 停 `ros2 daemon`
  3. `restart_stack`：换网/IP 变化或 ROS 图故障导致组件 BLOCKED → 停旧 daemon +
     按依赖顺序重启 `ps2-teleop/tof-a010/arm-grasp/k230-*/rosbridge` + voice；
     **互锁全清**才执行
  4. `restart_voice`：`voice.wakeword_lost` 持续 ≥60s 且语音不忙
- 网络变化检测：`dds` 探针的 `current_ips` 变化写 `network_changed`，
  180s 内作为 `restart_stack` 的触发条件。

明确不做：单点停服务不自动重启（避免掩盖崩溃循环）、欠压/板子离线不重启、
不重启 Pi/K230、不写配置文件/不改 K230 SD、不运行抓取/臂指令。

验收：重开 WiFi 省电 → 自动关闭；换网段 → `network_changed` + 互锁清空后
`restart_stack` 且组件恢复；臂运动中触发 → `recovery_deferred`；连续失败 →
熔断；`recovery off` 后零动作。

### 第三期：编排与长期

交付：L3 组重启、根因规则细化、每日摘要、门限按真实数据标定；nav2/SLAM
会话型任务的独立体检；可选告警通道（本地日志之外）。

验收：一周内「可自愈类」故障零人工介入；每次升级都有明确指纹与证据链。

## 10. 测试策略

- 探针单测：假 status 文件、假 systemd 输出、假 udev/udevadm、假 ss/journalctl、
  假 monotonic 时钟。
- 分类器真值表：每个组件 × 每种探针组合 → 期望状态与根因；重点覆盖
  「字段缺失 ≠ 合格」「BLOCKED 不压健康组件」「UNKNOWN 不计故障」。
- 迟滞真值表：1 坏仍 OK、2 坏才 STALE、4 好仍 STALE、5 好才 OK。
- 故障库规则表：每条规则一组命中/不命中证据。
- 变异自检：写完关键判据后，故意改坏实现（去掉字段缺失检查、去掉迟滞、
  让 BLOCKED 短路），确认对应测试变红，再改回。只靠
  `ModuleNotFoundError` 的「红」不算验证。
- 恢复策略单测（二期）：冷却、预算、验证升级、熔断开合、互锁拦截。
- 现场故障注入：`tools/inject_fault.sh`（停单元、停心跳、伪造欠压、伪 IP、
  伪造旧 DDS 绑定、touch 源码制造 build 过期）。
- CI 不依赖硬件。

## 11. 风险与对策

| 风险 | 对策 |
|---|---|
| 话题 QoS 不匹配导致误判 | 每话题声明 QoS；sensor 类用 BEST_EFFORT |
| DDS 图故障导致全体 UNKNOWN | `count_publishers` 直接区分发现层/数据通路，不逐个报故障 |
| 守护自己也在图的另一半里 | 自己收不到数据时，优先查 `ss` 旧 IP 与发布者数量，不把「我也看不见」当成「组件坏了」 |
| 时钟跳变 / Pi 无 RTC | 年龄用 monotonic；journal 带 boot_id+mono；`history --since` 用 boot 校正 |
| 日志膨胀 | 按天滚动，保留 30 天 |
| 每秒写 SD 卡 | state.json 在 `/run/carhealth`（tmpfs），磁盘只留 journal |
| 守护自己挂掉 | systemd `Restart=always`；二期加 sd_notify |
| 白名单误伤运动 | 互锁为唯一开关；ps2-teleop/arm-grasp 二期只观察 |
| 重启循环 | 指纹熔断 + 全局限流 + 开机静默期 |
| CPU 尖峰造成抖动噪音 | 连续 2 坏才降级 / 5 好才恢复；`doctor` 不自己抢 ROS 图 |
| 规则误报 | 规则只给建议、不改六态；每条规则有命中/不命中单测 |

## 12. 未决事项

- 告警通道：本期只做本地日志；云端/语音/手机推送在第三期按需加，不影响架构。
- nav2 / SLAM / 建图会话的健康模型在第三期单独设计（它们不是 systemd 单元）。
- 阈值初值需第一期真实数据标定后才固化；IDR 5s 阈值需现场确认 GOP 间隔。
- 云端链路（frp 是否在线、rosbridge 客户端数）第一期为可选探针，不阻塞验收。
- `arm-grasp` 的 `/health/arm_grasp` 定时器只能证明 executor 活着；
  要证明 action 在推进，必须同时看 `/arm/feedback` 新鲜度（已列入契约）。
- 若 Pi 上没有 journal 读权限（`adm`/`systemd-journal` 组），
  USB/ordering-cycle 探针会返回 UNKNOWN，不会误报；需在安装步骤里确认权限。

## 修订记录

- 2026-11-17 评审后修订：
  - 恢复动作按故障类别拆分，新增已知故障库；一期交付 `faults.py`。
  - 根因改为直接证据（`count_publishers`、`ss` 旧 IP），去掉「≥3 症状」推断。
  - 新增配置漂移体检与 USB / DDS / 服务资源 / 电池 / 底盘故障位 / load/WiFi 覆盖。
  - 欠压历史位不再让 doctor 永久 FAIL；欠压期间健康组件不再被 BLOCKED 掩盖。
  - 补迟滞；UNKNOWN 不再进 incident。
  - ToF 接口号改用 `udevadm`，并支持 by-id 兜底。
  - K230 地址未知时判 UNKNOWN，删除写死的旧回退 IP。
  - state.json 移到 `/run`；journal 带 boot_id/mono。
  - `doctor` 优先复用守护状态，warmup 8s；临时目录隔离。
  - 语音/抓取心跳补设备与引擎状态，不再只证明线程活着。
