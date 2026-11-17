# 整车健康守护（carhealth）设计

> 2026-11-17 · 状态：已定稿（用户确认架构、契约、恢复阶梯、分期）
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
2. **归因**：先找根因（掉压、板子离线、网络不可达、DDS 图故障），
   再决定动不动手；下游症状不各算各的故障。
3. **对症恢复**：分级阶梯 + 白名单 + 冷却/预算/验证窗口，**动作中一律不动手**。
4. **取证**：结构化故障日志，能回答「刚才为什么坏了、它做了什么、为什么没修好」。
5. **不成为新故障源**：只读探针、ROS 不可用时降级、自身被 systemd 看护。
6. **体检**：出车前一条命令拿到 PASS/WARN/FAIL 与可执行建议。

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
  carhealthd          # 常驻守护（系统级 systemd，User=cy，Restart=always，WatchdogSec）
  carhealthctl        # CLI：status / doctor / history / why <组件>
  contracts.py        # 组件清单（声明式）：探针、阈值、依赖、允许动作
  probes/
    systemd.py        # systemctl show ActiveState/SubState/NRestarts
    ros_topic.py      # 单个只订阅 rclpy 节点，按话题统计年龄与内容
    file_status.py    # status JSON + mtime 新鲜度（沿用 k230ctl 语义）
    net.py            # TCP 探活 / ping / 端口，不拉流
    serial.py         # udev 节点存在性、接口号、设备身份
    host.py           # vcgencmd get_throttled、/proc/meminfo、磁盘、温度、wifi
  classify.py         # 状态机 + 迟滞 + 根因挂载
  recovery.py         # 阶梯/白名单/联锁/熔断（二期启用）
  busy.py             # 动作互锁信号
  journal.py          # JSONL 故障日志 + 滚动
```

部署约束：

- 系统级 systemd 单元以 `cy` 运行；**不通过 `systemctl --user` 读取用户单元**，
  voice 用心跳文件判活（避免权限与 XDG_RUNTIME_DIR 依赖）。
- 探针一律只读：不打开串口、不拉视频流、不起 `ros2 topic echo` 子进程。
- rclpy 不可用时只跑非 ROS 探针，相关组件标 `UNKNOWN`，进程不退出。
- 守护自身由 `WatchdogSec` + `Restart=always` 看护；自重启也会写日志。
- 日志写 `/home/cy/.local/state/carhealth/journal-YYYYMMDD.jsonl`，按天滚动保留 30 天。

## 5. 健康契约

每个组件按同一形状声明，示例（ToF）：

```
component  tof
probe      systemd: tof-a010.service ActiveState=active
probe      topic: /tof/status age<2s, fps>0, last_error==null
probe      topic: /tof/depth age<2s              # 与 status 交叉验证
probe      udev: /dev/a010 存在，USB 接口号为 00 # 接口 01 是死通道，见到也算错
blocked_by power
recovery   restart tof-a010.service              # 静态、幂等、空闲可做
```

`/tof/status` 现有字段（`connected/connecting/frames/fps/reconnects/age_s/
device/port/last_error/frame_id`）即契约模板，新增心跳沿用同一形状。

### 组件清单与初值阈值（现场标定）

| 组件 | 存活 | 新鲜度/正确性 | 环境前提 | 互锁来源 | 允许恢复 |
|---|---|---|---|---|---|
| `power` | — | `vcgencmd get_throttled` 当前/历史欠压位 | — | — | 无 |
| `chassis` | ps2-teleop 单元 | `/wheel_odom`、`/imu/data_raw` <1s；`/cmd_vel_mux` 有流量 | `/dev/l150pro` | 底盘在动 | 空闲时重启 ps2-teleop |
| `arm` | 同上（同一串口） | `/arm/feedback` <0.5s；`/arm/online`=true；`/arm/status` 无故障位 | `/dev/l150pro` | 臂在动 / 抓取在跑 | 空闲时重启；动作中只记录 |
| `tof` | tof-a010 单元 | `/tof/status` 契约；`/tof/depth` <2s | `/dev/a010` 接口 00 | 抓取中 | 重启单元 |
| `k230-link` | 板子 ping | 8556 可连；`resultd-status.json` mtime<3s；`/k230/result` <3s | 板子在线、网络可达 | — | 重启 resultd / bridge |
| `k230-video` | k230-framed 单元 | 8554 可连；最新 IDR 年龄<5s | 板子在线 | — | 重启 framed |
| `rosbridge` | 单元 | TCP 127.0.0.1:9090 | — | — | 重启单元 |
| `voice` | user 单元 / 心跳 mtime | 心跳<5s；busy 字段；LLM 接口可达（空闲时 60s 一次） | 网络、cookie 未过期 | 工具执行中 | 空闲时重启 |
| `host` | — | 可用内存、swap、磁盘、SoC 温度、wifi | — | — | 无 |

### 判定规则

- 迟滞：连续 2 次坏才降级，连续 5 次好才恢复。
- 状态语义：`BLOCKED` 不是故障，是「环境根因压住了，等条件恢复」。
- **根因挂载**：`power` 欠压（当前或近 10 分钟有历史位）时，`chassis/arm/tof`
  的 STALE 统一挂 `root_cause=power`；`k230*` 在板子不可达时统一挂
  `root_cause=board_offline`；≥3 个话题同时 STALE 且单元均 active 时，
  先判 `root_cause=ros_graph`（DDS/网络），不逐个报组件故障。
- 时间：年龄一律用 `time.monotonic()`；墙钟只用于日志与展示（Pi 无 RTC）。

## 6. 分诊与恢复阶梯

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

### 熔断

- 故障指纹 = `hash(组件 + 状态 + 根因 + 证据特征)`。
- 同一指纹 30 分钟内复发 ≥3 次 → 打开熔断：该组件 30 分钟内不再自动动作，
  标 `ESCALATED`，日志记 `NEEDS_HUMAN`。
- 稳定 30 分钟后自动复位。

## 7. 动作互锁

恢复权限的唯一开关。任一为真即只记录不动手：

| 信号 | 判据 |
|---|---|
| 臂在动 | `/arm/command` 近期有流量，或 `/arm/feedback` 位姿在近期有变化 |
| 底盘在动 | `/cmd_vel_mux` 非零，或 `/wheel_odom` 速度非零 |
| 抓取在跑 | `/arm/grasp` action 活跃 |
| 语音在忙 | heartbeat 文件 `busy` 字段（工具执行中） |

缺信号的常驻节点在第一期补 1Hz 心跳与 busy 字段：`voice`、`arm-grasp`、
`k230-framed`。心跳格式沿用 `/tof/status` 的 JSON 形状。

## 8. 故障日志与 CLI

只记状态变化与动作，不记每次轮询。示例：

```json
{"ts":"2026-11-17T14:02:11.2","incident":"I-0412","component":"tof",
 "from":"OK","to":"STALE","verdict":"STALE",
 "evidence":{"age_s":4.2,"fps":0.0,"reconnects":7,"last_error":"read timeout","dev":"/dev/a010"},
 "root_cause":null,"fingerprint":"tof:stale:timeout:a1c9",
 "action":{"type":"restart_unit","target":"tof-a010.service","result":"ok","took_s":3.1},
 "interlock":{"arm_busy":false,"chassis_busy":false,"grasp":false,"voice_busy":false}}
```

根因事件单独成条（`power`、`board_offline`、`net_unreachable`、`ros_graph`），
下游 incident 通过 `root_cause` 引用，避免一次掉压产生七条独立告警。

CLI：

```
carhealthctl status            # 表格：组件/状态/年龄/根因/最近动作
carhealthctl status --json
carhealthctl doctor            # 出车体检：PASS/WARN/FAIL + 人话建议
carhealthctl history --since 1h
carhealthctl why tof           # 最近一次异常的完整证据链
```

`doctor` 检查项：udev 节点、开机以来欠压次数、ROS 图连通性、K230 IP 可达、
API key 与 cookie 寿命、磁盘、时间同步、必要端口。

## 9. 分期与验收

### 第一期：只观察，不动手

交付：`carhealth` 包、全部探针、分类器、日志、CLI、三个节点的心跳/busy 字段、
以 `recovery=off` 常驻的 systemd 单元、单测与故障注入脚本。

验收（硬性）：

1. 跑 2–3 次真实使用，每次掉数据/掉连接都能在日志中找到带证据的 incident。
2. `doctor` 对人为故障全部判对：拔 ToF、`systemctl stop k230-bridge`、
   伪造欠压位、K230 IP 指向不可达、杀掉 voice。
3. 全程自动动作次数 = 0。

### 第二期：连接层与单节点恢复

交付：`recovery.py`、白名单、冷却/预算/验证窗口、互锁、熔断；dry-run 先跑一段时间。
首批白名单：rosbridge、k230-resultd、k230-bridge、k230-framed、tof-a010、voice。
`ps2-teleop`、`arm-grasp` 仍只观察，现场验证联锁后再加。

验收：逐个 `kill -9` 白名单单元 → 20s 内自愈；伪造「臂在动」→ 零动作；
反复注入同指纹 → 熔断打开并落 `NEEDS_HUMAN`；伪造欠压 → 零动作。

### 第三期：编排与长期

交付：L3 组重启、根因规则细化、每日摘要、门限按真实数据标定；nav2/SLAM
会话型任务的独立体检；可选告警通道（本地日志之外）。

验收：一周内「可自愈类」故障零人工介入；每次升级都有明确指纹与证据链。

## 10. 测试策略

- 探针单测：假 status 文件、假 systemd 输出、假 udev 路径、假 monotonic 时钟。
- 分类器真值表：每个组件 × 每种探针组合 → 期望状态与根因。
- 恢复策略单测：冷却、预算、验证升级、熔断开合、互锁拦截。
- 现场故障注入：`tools/inject_fault.sh`（停单元、停心跳、伪造欠压、伪 IP）。
- CI 不依赖硬件。

## 11. 风险与对策

| 风险 | 对策 |
|---|---|
| 话题 QoS 不匹配导致误判 | 每话题声明 QoS；sensor 类用 BEST_EFFORT |
| DDS 图故障导致全体 UNKNOWN | 先判 `ros_graph` 根因，不逐个报故障 |
| 时钟跳变 | 年龄用 monotonic；doctor 检查时间同步 |
| 日志膨胀 | 按天滚动，保留 30 天 |
| 守护自己挂掉 | systemd `Restart=always` + `WatchdogSec` |
| 白名单误伤运动 | 互锁为唯一开关；ps2-teleop/arm-grasp 二期只观察 |
| 重启循环 | 指纹熔断 + 全局限流 + 开机静默期 |

## 12. 未决事项

- 告警通道：本期只做本地日志；云端/语音/手机推送在第三期按需加，不影响架构。
- nav2 / SLAM / 建图会话的健康模型在第三期单独设计（它们不是 systemd 单元）。
- 阈值初值需第一期真实数据标定后才固化。
