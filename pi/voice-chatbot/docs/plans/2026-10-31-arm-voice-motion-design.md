# 机械臂语音调用（AI 现场给姿态）设计

2026-10-31。对话里已与用户逐条确认，本文把结论落盘。实现计划见同目录 `2026-10-31-arm-voice-motion-plan.md`。

## 1. 目标与非目标

**目标**：语音助手（deepseek-v4-flash）能用口语直接、可预期地动机械臂——看某个方向、低头抬头、
点头摇头、伸出去收回来、开合夹爪——**姿态由 AI 现场决定，不靠预设姿态库**。
同一套接口以后给"视觉辅助 SLAM 语义"复用（看向地图里的某个点）。

**非目标（本轮不做）**：
- 车身/雷达禁区（用户定：暂不设，只保留关节软限位）；
- 臂的 TF 发布、`frame="map"` 的看向点（P3，接口先留）；
- 档位表的"按用户纠正自动调整"（P3；本轮档位表放配置文件，可手调）；
- 改驱动 `l150pro_driver_node.py`（冻结清单里的文件，见记忆 raspot-freeze-chassis-base）。

**已定约束**（用户 2026-10-31 确认）：
- 允许动作中开合夹爪；允许车在走的时候动臂（不做底盘/臂互锁）。
- 不设笛卡尔禁区。
- 用户说话是模糊的（「低下头」「左边看看」「再低点」），不会说厘米/度数，但偶尔会说。

## 2. 核心思路：模糊 → 具体 由代码做，不由模型做

模型只负责**听懂意图**（往哪、大概多少、要不要来回），**不算几何**。

| 口语 | 模型输出 | 代码做的事 |
|---|---|---|
| 低一点 / 再低点 | `turn 下 一点` | 在**当前**视线上往下 8°（档位表） |
| 左边看看 | `toward 左边` | 命名方向 → (方位, 俯仰) → 视线求解器现场解关节 |
| 往前伸一些 | `shift 前 一些` | 爪尖沿车头方向 +5cm，俯仰不变，IK |
| 凑近点看 | `approach 靠近 一点` | 沿视线方向 +2cm |
| 点点头 | `oscillate 上下 一些 ×3` | 展开成 去-回-去-回…-回原位 的关键帧 |
| 左转 30 度看 | `turn 左 degrees=30` | 给了数就原样用，不走档位 |
| 张开爪子 | `gripper 张开` | p1=240 |

要点：
- **同一个程度词 = 同一个增量**（不是同一个终点）。终点取决于当前姿态。
- 方向/目标类说法由求解器现场算，与档位表无关。
- 时长不让模型填：`duration = max(关节变化量)/角速度档`（慢/正常/快）。
- 越限一律**截到极限并如实说明**（"已经低到最低了"、"左边最多转到 270°，还差 30°，要的话车可以转"），不报错让模型重试。

## 3. 坐标与约定（全部沿用 `arm_kin.py`，不新造）

- 关节角：`base / shoulder / elbow / wrist_pitch`（度），`arm_kin.to_fields/from_fields` 与舵机 field 互转。
- 车体系：x 前、y 左、z 上，cm，原点在臂底座安装面（`arm_kin` 的 z 参考面）。
- **视线** = 夹爪轴（相机光轴与之平行，`fk()` 的 `axis`）：
  - 方位 `az` = 底座角，**左正**、0=车头；
  - 俯仰 `el` = `alpha = shoulder+elbow+wrist_pitch`，**下负**、0=水平。
- 关节软限位：p3/p4/p5 field ∈ [125, 875]（`FIELD_LO/HI`）；底座 `|base| ≤ base_limit_deg`（配置，**±270°（用户 2026-10-31 确认）**，
  ）；夹爪 p1 ∈ [0, 600]，张开 240、合上 578、半开 400；腕旋 p2 原样保持。

## 4. 架构

```
deepseek-v4-flash
  │  原生工具（tools/arm.py，group="arm"）——形状由 §6 的评测定稿
  ▼
car/arm_client.py           ActionClient /arm/motion + Trigger /arm/describe（仿 grasp_client.py）
  ▼  spec_json（结构化意图，未量化）
Pi: arm_grasp_server（现有进程，扩展）
  ├─ motion_spec.py   档位表/命名方向/速度档（纯数据+解析，读 YAML）
  ├─ gaze.py          视线求解：(az, el) + 当前关节 → 关节（纯函数）
  ├─ motion_plan.py   spec + 当前关节 → [Keyframe(fields, t_ms)] + notes（纯函数，含限位截断）
  ├─ motion_exec.py   逐帧执行 + 回读确认 + 打断（依赖一个 Link 抽象，可用假 Link 测）
  └─ grasp_server.py  加 ArmMotion action、arm/describe 服务；与 Grasp 共用 _busy/_abort
  ▼
/arm/command (frame_id='motion') → 驱动 → 固件 → LX 舵机
```

**为什么塞进 grasp_server 而不是新节点**：`/arm/command` 上 AI 侧只该有一个发布者；抓取和自由动作
共用"忙/打断/手柄让权/急停"逻辑（已经在 grasp_server 里且实车验证过）。两者互斥：一个在跑另一个回 `BUSY`。

**为什么 spec 用 JSON 字符串而不是展开成 .action 字段**：意图结构会随评测改；字段一改就要重编接口包、
两端一起部署。JSON 里的结构由 `motion_spec.py` 的解析器校验（不合法回 `BAD_GOAL` + 原因）。

### 4.1 `raspot_interfaces/action/ArmMotion.action`

```
string spec_json      # 见 §5
bool dry_run          # true = 只规划不动臂，返回规划结果（评测/调试用）
---
bool ok
string code           # OK | PARTIAL | BAD_GOAL | BUSY | NO_FEEDBACK | CANCELED | ABORTED_BY_OPERATOR | ESTOP | TIMEOUT | ERROR
string reason         # 人话（中文）：做了什么 + 截断/没到位的说明
string state_json     # 结束时状态（同 arm/describe）
---
int32 step
int32 total
string detail
```

`PARTIAL` = 做完了但有截断（到了限位）——对用户不是错误，reason 里说清楚。

### 4.2 `arm/describe`（std_srvs/Trigger）

`message` = state JSON：关节角、视线 (az, el)、爪尖 (x, y, z)、夹爪 开/合/半开/夹着东西、
以及一句 `text`（给模型念的人话，例如「视线朝左 20°、向下 35°；爪尖在车前 16cm、高 8cm；夹爪张开」）。
回读不稳定/没有时 success=false。

## 5. spec（模型意图的结构化形式，未量化）

```jsonc
{
  "steps": [                       // 依次执行；单步也用数组
    {
      "toward":   "左边",          // 命名方向（§5.1）或 "上一个姿态"
      "turn":     [{"dir": "下", "amount": "一点"}, {"dir": "左", "degrees": 30}],
      "shift":    [{"dir": "前", "amount": "一些"}, {"dir": "上", "cm": 3}],
      "approach": {"dir": "靠近", "amount": "一点"},
      "gripper":  "张开",          // 张开 | 合上 | 半开
      "oscillate": {"axis": "上下", "amount": "一些", "times": 3}   // 上下 | 左右 | 夹爪
    }
  ],
  "repeat": 1,
  "speed": "正常"                  // 慢 | 正常 | 快
}
```

一个 step 里多个字段的合成顺序固定：`toward` → `turn` → `shift` → `approach` → `gripper`，
合成为**一个**目标姿态（一帧）；`oscillate` 单独展开成多帧且不与其它字段同 step。

### 5.1 档位表与命名方向（`config/motion.yaml`，可手调）

| | 一点 | 一些 | 很多 | 到底 |
|---|---|---|---|---|
| turn（度） | 8 | 20 | 40 | 到限位 |
| shift / approach（cm） | 2 | 5 | 10 | 到可达边界 |
| oscillate 幅度（度） | 8 | 15 | 25 | — |

速度档（关节空间最大角速度）：慢 30°/s、正常 60°/s、快 120°/s；每帧最短 250ms。oscillate 默认"快"。

命名方向（az, el）：前方 (0, −15)、左边 (90, −15)、右边 (−90, −15)、左前 (45, −15)、右前 (−45, −15)、
上方 (0, 30)、地上/脚下 (0, −75)、地上远处 (0, −40)、后方 (180, −15；±180 两个解都在 ±270 内，取离当前底座角近的那个，避免绕远)。

## 6. 工具形状：已定稿 B（合一、单步平铺）

> 2026-10-31 Task 1 评测定稿。schema 源：`tools/arm_schema.py` 的 `SCHEMAS_B`；语料 `tests/arm_eval/corpus.yaml`（78 条）；
> 脚本 `scripts/eval_arm_tools.py`（连同现有 22 个工具一起发给 deepseek-v4-flash，T=0.7，每条 3 次）。

**定稿工具（3 个）**：
- `arm_act`：参数就是 §5 的 spec，但**单步时字段直接平铺在顶层**（`toward/turn/shift/approach/gripper/oscillate`
  + `speed`/`repeat`），**一句话里有先后**（「先低头再张开爪子」）才用 `steps`（每步同样这些字段）。required 为空。
  handler 还原：`steps = [顶层字段组成的一步（若非空）] + steps`（`arm_schema.to_spec`）。
- `arm_state`：无参，调 `arm/describe`。
- `arm_home`：无参，调现有 `arm/go_home`（夹爪保持）。

**为什么平铺**：最初的 B 是 `{"steps":[...]}` 必填嵌套，多轮里（「再低点」「还不够」「再来一次」）约 5%
（10 次×13 条中 6 次）会把 `dir/amount/cm` 拍平写到步上（`{"steps":[{"dir":"下","amount":"一些","turn":[…]}]}`），
平铺后同一组 130/130。
**为什么不选 A**：A 要求模型先在 `arm_look`/`arm_move` 之间二选一，跨两组的组合（「低头凑近点看」）
反复把 `turn` 塞进 `arm_move`（专项 10 次里 4 次，description 写明"没有 turn"也压不住）；B 一个调用就能表达。

**评测结果**（最终版 description，整份语料 ×3）：

| | 总通过 | 车/臂混淆 | 单步模糊 | 带数 | 多轮相对 | 组合 | 混淆对照 | 状态/复位 |
|---|---|---|---|---|---|---|---|---|
| A | 96.2%（225/234） | 0 | 98.9% | 100% | 100% | 87.5% | 89.6% | 100% |
| **B（定稿）** | **97.0%（227/234）** | **0** | 100% | 100% | 100% | 87.5% | 91.7% | 100% |

B 剩下的失败**全部**落在期望待用户裁定的条目：c01「左转」/c02「往前走」模型用 `ask_user` 追问角度/距离
（现有车工具的行为，不是臂的问题）；k08「张开爪子 往前伸一点」模型把两件事合成同一步（同时到位）
而期望写的是先后两步。去掉这些条目 B 为 100%。

**description 的关键写法**（system_prompt 一字未改）：
- 开头说清"摄像头在爪子上，臂指哪镜头就看哪"，并列举口语（低下头/往右瞄一眼/再低点/回到刚才那样…）；
- 分界写在动作工具描述末尾：单说「左转/右转/掉个头/往前走/后退」是车（car_turn/car_move），
  **话里带「看/瞄/瞅/头/伸/爪子」才是臂**（「左转 30 度看看」是臂、「左转 30 度」是车）；抓用 grasp_object，
  盯/跟用 follow_person/track_object，问画面用 look；
- toward（绝对方向）与 turn（当前视线上的相对量，「再…一点」用它）分开讲；
- amount 的口语映射（一点/稍微/一下/一丢丢=一点 …），"说了度数/厘米才填，不要自己换算"。

评判与通过线（保留）：整体 ≥ 90%，且**车/臂混淆为 0**（混淆＝该动臂却开车或反之，安全相关）。

每个动作工具返回 `reason` + 状态人话，模型据此回答，也据此理解下一句「再……一点」。
⚠️ 评测里的工具结果是脚本编的（按调用内容生成"做了什么"，不带姿态读数）；真实返回若带读数，必须与动作自洽，
否则模型会以为没做成而重调（评测初版用固定读数时就出现过）。

## 7. 规划（motion_plan.py）

输入：spec、当前关节（回读）、当前夹爪、配置、历史姿态栈。输出：`[Keyframe(fields[6], t_ms, label)]` + `notes[]`。

- **turn / toward → 视线求解**（gaze.py）：
  1. `base = az`，截到 ±base_limit（截了 ⇒ note「左边最多转到 X°，还差 Y°」）；
  2. 先试"只动腕"：肩、肘不变，`wrist = el − shoulder − elbow`，field 在限内就用（像转脖子，动作最小）；
  3. 不行则在肩×肘网格（1°）里找满足 field 限位、且 `|Δshoulder|+|Δelbow|` 最小的解；
  4. 都不行（el 本身超出可达）⇒ 取最接近的可达 el，note「已经低到最低了」。
- **shift**：爪尖目标 = 当前爪尖 + Δ（车体系），保持 el；`ikine` 两个肘支取离当前关节近的；
  不可达 ⇒ 沿 Δ 二分找最大可行比例，note「只往前挪了 3cm，再远够不着」。
  注意：左右平移会改变底座角，从而视线方位跟着变——这是物理事实，不额外补偿。
- **approach**：Δ = d · 视线单位向量，其余同 shift。
- **oscillate**：上下＝腕俯仰 ±A（不动肩肘）；左右＝底座 ±A；夹爪＝开/合交替。
  展开 `times` 个来回，最后一帧回到起始姿态。
- **上一个姿态**：执行器维护最近 5 个"动作前姿态"栈；`toward:"上一个姿态"` 弹出栈顶。
- **时长**：每帧 `t_ms = max(250, max|Δ关节°| / 档速 × 1000)`。
- **截断报告**：所有截断/不可达汇总进 notes，最终 code=PARTIAL。

## 8. 执行（motion_exec.py）

- 前置：`cmd_gate` 不能是 dead；回读必须稳定（`_stable`），否则 `NO_FEEDBACK`。
- 每帧：把驱动 `arm_t_ms` 设成本帧 t_ms（grasp_server 已有 `_set_driver_t_ms`），发**一次**目标
  （frame_id='motion'，顺 LX 舵机"目标恒定+一条长轨迹最丝滑"的特性，记忆 lx-servo-smoothness），
  等 `t_ms + 稳定`（回读与目标差 ≤ 容差连续 3 拍，或超时 t_ms+1.5s）。
  250ms 后回读仍完全没动 ⇒ 以剩余时间重发一次（防串口丢帧），最多 1 次。
- 结束（任何路径）`finally`：`arm_t_ms` 设回 `normal_arm_t_ms`。
- **打断**：
  - 语音唤醒词 ⇒ 客户端 cancel ⇒ `CANCELED`，并**冻结**：以当前回读为目标、t_ms=100 发一帧（硬停，不续——
    记忆 voice-interrupt-is-resumable 的"运动例外"）；
  - 手柄 frame_id='teleop' ⇒ `ABORTED_BY_OPERATOR`，**不再发任何帧**（人接管）；
  - `/estop` ⇒ `ESTOP`。
- **不加任何积分/补偿**：到不了就报告残差（记忆 integrator-compensation-can-crush）。
- 底座是固件闭环（非 LX 位置舵机），`t_ms` 对它可能不生效 ⇒ 实车验证点（Task 4）。

## 9. 语音侧

- `car/arm_client.py`：仿 `grasp_client.py`（自建节点 + 自己的 SingleThreadedExecutor + daemon 线程，
  起不来只降级）。`act(spec) -> str`、`describe() -> str`、`home() -> str`、`cancel()`。
- `tools/arm.py`：按评测胜出的形状注册（group="arm"），handler 把参数整理成 spec_json。
- `core/conversation.py`：注册；`_wake_fast_action()` 里在 `self._car.stop_all()` 旁加 `arm.cancel()`
  （这条路径绝不抛异常）。抓取的打断行为不在本轮范围内，不改。
- 日志：每次调用（参数、code、reason、耗时）追加一行到 `logs/arm_motion.jsonl`（不入库），
  给 P3"高频动作升级为预设 / 档位表按反馈调整"攒数据。

## 10. 风险与验证点

| 风险 | 处理 |
|---|---|
| flash 把「往左看」当 `car_turn` | 评测通过线要求车/臂混淆为 0；不过就改 description/形状 |
| 单发一帧被串口丢 | 250ms 无动作重发一次；实车统计 |
| 底座不吃 t_ms、速度不可控 | Task 4 实测；必要时底座大角度拆成多帧 |
| `fk` 与实物有偏差 | 视线控制只依赖角度（az/el），对连杆误差不敏感；平移类误差 ~1cm 级可接受 |
| 动作结束 PS2 接手时臂跳 | teleop 空闲时镜像回读进目标（已有）；Task 4 验证"AI 动完立刻推摇杆不跳" |
| 底座走线缠绕 | `base_limit_deg` = 270（用户确认）；绝对方向（toward）在等价角里取离当前最近的，避免累计绕圈 |

## 11. 分期

- **P1（本计划）**：评测定形 → 纯函数规划 → action/执行 → 实车 → 语音接入 → 端到端。
- **P2**：`look_at(point, frame)`（相机偏置 12.3cm，`cam_model`）+ 臂 TF 发布。
- **P3**：`frame="map"`（SLAM 语义）；档位表按用户纠正自动调；按日志把高频动作升级为预设。

## 12. Task 1 遗留条目的用户裁决（2026-10-31）

- 「往前走」「左转」不带量时模型用 `ask_user` 追问距离/角度：**认可**，车工具不改。
- 「张开爪子 往前伸一点」合成一步同时到位：**认可**（原话没有先后就允许同一步）。
- 底座限位 **±270°**。
