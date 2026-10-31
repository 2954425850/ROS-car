# 机械臂语音调用 实现计划

> 执行方式：**子代理逐任务**——每个任务派一个干净的 subagent，任务之间停下来人工审查再放行（用户约定）。
> 设计：`2026-10-31-arm-voice-motion-design.md`（下称"设计"）。每个任务开工前先读设计对应章节。

**Goal**：语音助手能用模糊口语现场驱动机械臂（看向/低抬头/点头/伸缩/开合夹爪），不靠预设姿态。

**Branch**：`feat/arm-voice-motion`（从 `feat/k230-a010-tof` 切出；Pi 当前在 `feat/k230-a010-tof`，部署时再切）。

## Global Constraints

- **Pi 是权威、本地写 → push → Pi pull**。Windows 工作副本 `C:\Users\29544\ROS-car`，Pi `/home/cy/ROS-car`（主机 `nepu.local`，走 ssh-mcp；账号见本机记忆，不入库）。
- **不改 `system_prompt`**（`config.yaml` 的 `llm.system_prompt`）。工具何时调用全靠 description。
- **不改驱动 `raspot_teleop/l150pro_driver_node.py` 与底盘链路**（冻结）。`ps2_teleop_node.py` 本计划也不改。
- **不改抓取流程 `grasp.py` 的行为**；`grasp_server.py` 只做"加"（新 action/服务），Grasp action 行为逐字不变。
- `raspot_ws` 改任何 `.py` 都要 `colcon build`（服务 import 的是 `build/` 副本）；新增 `.action` 要重编 `raspot_interfaces`。
- 纯逻辑模块（`motion_spec.py`/`gaze.py`/`motion_plan.py`/`motion_exec.py`）**不许 import rclpy**，能在 Windows/无 ROS 环境跑 pytest。
- **新测试必须做变异自检**：故意改坏被测代码一处、确认测试变红、再改回（记忆 test-must-be-able-to-fail）。在任务报告里写出做了哪几个变异。
- **公开仓库**：api key 一律不进库；`logs/`、评测输出 JSON 不入库。
- 行尾 LF（仓库 `.gitattributes` 已钉）。
- 重启 `ps2-teleop` 会让臂归位——本计划**不需要**重启它；需要重启的只有 `arm-grasp`（不归位）和用户级 `voice-chatbot`。
- 动臂的实车步骤（Task 4、Task 6）**必须用户在场**，subagent 不得自行发动臂指令。

---

## Task 1：工具形状评测（定稿 A 或 B）

**目的**：在写任何执行代码前，用真实口语验证 deepseek-v4-flash 能否把模糊说法稳定地映射到工具参数，并选定形状。

**Files**
- Create `pi/voice-chatbot/tools/arm_schema.py`：纯数据。`SCHEMAS_A`（arm_look/arm_move/arm_state/arm_home）、`SCHEMAS_B`（arm_act/arm_state/arm_home），OpenAI tools 格式，description 中文、口语化、写清"何时调用/何时不要（车/抓取/跟随）"。字段与枚举严格按设计 §5/§5.1。
- Create `pi/voice-chatbot/tests/arm_eval/corpus.yaml`：≥ 50 条。每条 `{id, history?, say, expect}`；`expect` 写**语义**断言：`tool`（或 `tools_any`）、关键字段（如 `turn[0].dir == 下`、`amount in [一点, 一些]`）。必须覆盖：
  - 单步模糊：低下头/抬头/左边看看/往右瞄一下/看看地上/看前面/往前伸点/收回来/举高点/张开爪子/合上/点点头/摇摇头；
  - 带数：左转 30 度看看、往前伸 5 厘米；
  - **多轮相对**（`history` 里放上一轮的用户话 + 工具调用 + 工具结果）：再低点/再往左一点/回到刚才那样/还不够；
  - **混淆对照**（期望是车/别的工具或不调用）：左转、往前走、掉个头、抓那个杯子、盯着那个人、还有多少电、今天天气；
  - 组合：往左下看、先低头再张开爪子。
- Create `pi/voice-chatbot/scripts/eval_arm_tools.py`：读 `config.yaml` 的 llm（base_url/model/api_key/system_prompt），**把现有全部已注册工具的 schema + 候选臂工具一起**发给模型（现有工具 schema 通过构造 registry 获取；构造失败就从各 `register_*` 里抽 schema，不得只发臂工具），`temperature` 用服务里的实际值；每条跑 3 次；输出每条的调用与判定，汇总：总通过率、车/臂混淆数、按类别通过率。`--variant A|B`，结果写 `logs/arm_eval-<variant>-<ts>.json`。
- Create `pi/voice-chatbot/tests/test_arm_schema.py`：schema 结构合法（OpenAI 格式、枚举与设计一致、required 正确）；corpus 每条 expect 引用的工具/字段在对应 schema 里存在。

**Steps**
1. 写 schema + corpus + 结构测试（先红后绿 + 变异自检）。
2. 在 Pi 上跑 `python3 scripts/eval_arm_tools.py --variant A` 与 `--variant B`（key 在 Pi 的 `config.yaml`，不得打印或回传 key）。
3. 对失败条目迭代 **description 措辞/字段形状**（不改 system_prompt、不改 corpus 的期望来"凑通过"；若认为某条期望本身不合理，列出来交给用户判）。
4. 报告：两种形状的通过率表、混淆明细、最终推荐及理由；把定稿形状写回设计 §6（只改 §6）。

**验收**：推荐形状整体 ≥ 90%、车/臂混淆 = 0。**任务完成后停下等用户审查**（用户可能要补自己的口语）。

---

## Task 2：纯函数规划层（Pi 侧，无 ROS）

**Files**（`pi/raspot_ws/src/arm_grasp/`）
- Create `config/motion.yaml`：档位表、速度档、命名方向、夹爪值、`base_limit_deg: 150`、最短帧 250ms（设计 §3/§5.1/§7）。在 `setup.py` 的 `data_files` 装上。
- Create `arm_grasp/motion_spec.py`：`load_config(path)`；`parse_spec(json_str) -> Spec`（dataclass），不合法抛 `SpecError(中文原因)`；`resolve_amount(kind, amount|None, number|None, cfg) -> float|"max"`。
- Create `arm_grasp/gaze.py`：`gaze_of(joints) -> (az, el)`；`solve_gaze(az, el, cur_joints, cfg) -> (joints, notes)`，算法按设计 §7（先只动腕 → 肩肘网格最小改动 → 截断到最近可达 el）。
- Create `arm_grasp/motion_plan.py`：`plan(spec, cur_fields, history, cfg) -> Plan(keyframes, notes, partial)`；`Keyframe(fields: list[6], t_ms: int, label: str)`。实现 toward/turn/shift/approach/gripper/oscillate/上一个姿态/repeat/speed 与时长公式。
- Create `test/test_motion_spec.py`、`test/test_gaze.py`、`test/test_motion_plan.py`。

**必须有的测试**（每条都做变异自检）
- gaze：`gaze_of(solve_gaze(az,el,…))` 回到 (az,el)（容差 0.5°），覆盖"只动腕可行"和"需要动肩肘"两类；所有输出 field 在 [125,875]；底座超限截断且 note 含"还差"。
- turn 相对量：同一 `一点` 在两个不同起始姿态下，**增量相同、终点不同**。
- shift：爪尖位移 = 期望 Δ（fk 验证，容差 0.2cm），el 不变；不可达时按比例缩短且 note 说明实际挪了多少。
- oscillate：帧数 = 2×times+1，最后一帧 == 起始姿态。
- 时长：慢 > 正常 > 快；≥ 250ms。
- 解析：未知方向/未知档位/oscillate 与其它字段同 step → `SpecError`。

**验收**：`cd pi/raspot_ws/src/arm_grasp && python3 -m pytest test/test_motion_*.py test/test_gaze.py -v` 全绿（Windows 与 Pi 都跑）；原有测试不受影响。完成后停下审查。

---

## Task 3：ArmMotion action + 执行器 + describe（Pi 侧）

**Files**
- Create `pi/raspot_ws/src/raspot_interfaces/action/ArmMotion.action`（设计 §4.1），CMakeLists 登记。
- Create `arm_grasp/motion_exec.py`：`run_motion(plan, link, cfg) -> (code, reason)`；`Link` 协议：`fb()`、`publish(fields)`、`set_t_ms(ms)`、`spin(dt)`（内部查打断，抛 `Abort(code, reason)`）、`now()`。实现设计 §8：逐帧发一次、等稳定、250ms 无动作重发一次、`finally` 还原 t_ms、cancel 时冻结帧。
- Modify `arm_grasp/grasp_server.py`（只加）：`ArmMotion` ActionServer `arm/motion`（与 Grasp 共用 `_busy`/`_abort`；frame_id='motion'；teleop 打断沿用 `_on_cmd`）；`arm/describe` Trigger；维护"动作前姿态"栈（5）。Grasp 的代码路径不改。
- Create `test/test_motion_exec.py`：用假 Link（模拟回读按 t_ms 线性逼近目标、可注入丢帧/teleop/estop/cancel）。

**必须有的测试**：正常多帧全部到位；第一帧被"丢"时重发一次后到位；cancel ⇒ 发出一帧冻结（目标=当前回读）且 code=CANCELED；teleop ⇒ ABORTED_BY_OPERATOR 且之后**零**发布；任意路径 t_ms 都被还原；到不了目标 ⇒ 报残差、不加压（发布次数有上限）。

**部署（只构建不动臂）**：push → Pi `git fetch && git switch feat/arm-voice-motion` → `colcon build --packages-select raspot_interfaces arm_grasp` → `systemctl restart arm-grasp`（不归位）→ `ros2 action list | grep arm/motion`、`ros2 service call /arm/describe std_srvs/srv/Trigger` 返回合理状态。可以跑 **dry_run=true** 的 goal 验证规划；**不许发 dry_run=false**。完成后停下审查。

---

## Task 4：实车验证（用户在场）

由主会话与用户一起做，不派 subagent。逐项、每项先说清会怎么动再发：
1. `turn 下 一点` / `toward 左边` / `toward 地上` 单帧；目测方向 + describe 读数。
2. `oscillate 上下 一些 ×3`（点头）、`左右`（摇头，底座）——**看底座是否遵守 t_ms**。
3. `shift 前 一些`、`approach 靠近 一点`，尺子量位移。
4. `gripper 张开/合上/半开`。
5. 动作中喊唤醒词（或 `ros2 action` cancel）⇒ 冻结；动作中推手柄 ⇒ 让权；动作完立即推摇杆 ⇒ 不跳。
6. 统计重发次数（丢帧率）。
结果记进设计 §10，必要时回 Task 2/3 修。

---

## Task 5：语音侧接入

**Files**（`pi/voice-chatbot/`）
- Create `car/arm_client.py`（仿 `grasp_client.py`）：`start()`、`act(spec: dict) -> str`、`describe() -> str`、`home() -> str`、`cancel()`（绝不抛）。code → 人话映射；返回 `reason` + 状态 `text`。
- Create `tools/arm.py`：用 `tools/arm_schema.py` 里**定稿**的 schema 注册（group="arm"），handler 把参数转 spec；每次调用写一行 `logs/arm_motion.jsonl`。
- Modify `core/conversation.py`：注册（起不来只降级，工具照注册、调用时说明原因）；`_wake_fast_action()` 里 `self._car.stop_all()` 后加 arm cancel（try/except 吞掉一切）。
- Create `tests/test_arm_tools.py`：假 client；参数 → spec 映射；client 为 None 时的降级文案；唤醒路径调用了 cancel 且异常不外泄。

**验收**：`python3 -m pytest -v` 全绿；`git diff` 证明 `config.yaml.example` 的 system_prompt 逐字未变。完成后停下审查。

---

## Task 6：端到端（用户在场）

Pi `git pull` → 重启用户级 `voice-chatbot` → 用户实际说 corpus 里的话 + 自由发挥；记录不符合预期的说法，回 Task 1 迭代 description。确认后合并方案与用户商量（`feat/k230-a010-tof` 与 `main` 是分叉的，合并前先问）。
