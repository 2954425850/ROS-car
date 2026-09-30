# 小车语音控制（car tools）设计

日期：2026-09-17
状态：已确认，待写实现计划

## 1. 目标与范围

给 nepu 上的语音助手（JARVIS，`~/voice-chatbot`）加上**用嘴控制 L150Pro 小车**的能力。

**本次范围**：新增小车工具 + 支撑它们所必需的 `CarController` + 驱动侧一处小改动。

> 勘误（2026-09-17 13:50）：本文写作时「工具执行期间全静默」是现状描述，但用户在 12:50 已自行实现 `_say_before_tools`（调工具前那句「我看一下…」立刻出声）。下面凡提到「攒到最后一次性 TTS」的地方都是**写作时的旧状态**，现已不成立。小车部分不受影响。

**明确不在范围内**：
- 「工具执行期间没有反馈」的改造（用户单独处理）
- 长时任务（跟随 / 巡线）—— 未来
- SLAM 建图 / 路径规划工具 —— 未来
- 任何对 `mcp_host/` 的改动

## 2. 核心设计判断：三层，`set_velocity` 不是工具

```
语音意图层    "往前走两米" / "转九十度" / "云台往左"      ← 模型在这一层
    ↓  Tool（粗粒度、口语化、5 个）
能力层        CarController 的 Python API                ← 程序在这一层
    ↓  （未来的 nav / slam 也调这一层，不再新增模型可见工具）
硬件层        ROS2 cmd_vel / servo_cmd / wheel_odom      ← 驱动在这一层
```

**`set_velocity(vx, wz)` 是 Python API，不暴露给模型。** 两个理由：

1. **语音带宽极低。** 用户说「去厨房」，不会说「设 vx 为 0.3」。速度/坐标换算是模型做不好的事，把它做成工具只会增加选错的机会。
2. **一份底层，多个消费者。** `car_move` 用它、未来的 `nav_goto` 用它、SLAM 算法也用它。它们全都调同一个 Python 方法，谁都不需要模型在中间翻译。

这条判断决定了**未来叠加功能时，工具数量不会跟着功能数量一起涨**——导航、建图作为「意图」才变成工具（`nav_goto`、`slam_start`），底层原语永远不变成工具。

### 关于工具数量稀释

当前 9 个工具，本次新增 5 个 = **14 个**。经验上 15 个以内模型选择准确率稳定，20+ 开始明显下滑。因此本次**只预留、不实现**分组能力：

- `Tool` dataclass 加 `group: str = "core"` 字段（有默认值，不破坏现有代码）
- `ToolRegistry.schemas()` 加可选 `groups=` 过滤参数

将来工具过 20 个时，让 `LLMClient` 的 `extra_schemas` 回调按需返回子集即可，不用改 LLM 层。

## 3. 工具清单（5 个）

```python
# ---- 运动 ----
car_move(direction, distance, speed=None)
    direction: forward | backward
             | forward_left | forward_right | backward_left | backward_right
    distance:  米。走完就停（里程计闭环）。
    speed:     m/s，留空用 config 默认值。

car_turn(angle_deg, speed=None)
    angle_deg: 度，正 = 逆时针（REP-103）。
    speed:     度/秒，留空用 config 默认值。

car_stop()
    立即归零。

# ---- 云台 ----
car_camera(direction, degrees=None)
    direction: left | right | up | down | center
    degrees:   留空 = 固定步进（config.car.camera_step_deg）；给了 = 按角度转。

# ---- 状态 ----
car_status()
    电压、故障码、四轮轮速、当前是否在动。
```

**`car_move` 为什么没有 `left` / `right`。** 差速车没有横移，「往左」要么是原地转（归 `car_turn`），要么是弧线（`forward_left`）。而「往左走两米」里的「两米」对原地转毫无意义——把两者塞进同一个工具的 enum，只会让模型在一个它本来就不擅长的判断上多想一层。

**统一模式：「必填参数 + 可选精度」。** `speed` / `degrees` 留空就用默认值，用户说了「快点」「抬 30 度」才传。**何时该传由 tool description 承担——不改 `system_prompt`。**

> 参数名 `speed` 在 `car_move` 是 m/s、在 `car_turn` 是度/秒。单位由各自 description 说明，不靠参数名区分。

### 有意不做的工具

- **`car_get_pose()`** —— 当前里程计未标定、IMU yaw 是纯陀螺积分会漂。一个会自信报错位置的工具有害无益：用户问「车在哪」，模型念一个错数字，比说「我还不知道自己在哪」糟得多。等标定完成再加。
- **云台分轴工具**（`car_camera_pan` / `_tilt`）—— 语音场景下「往左看看」是单一意图，拆轴只会让模型多想一层。

## 4. `CarController`

新文件 `car/controller.py`。照抄 `MusicController` 已验证的模式（独立线程 + 在 `_on_wake_word` 里被第一时间叫停）。

```python
class CarController:
    def start(self)                                  # 起 ROS node + spin 线程
    def stop_all(self)                               # 急停：目标归零 + 补发零速帧
    def set_velocity(self, vx, wz)                   # ← 原语，程序内部用
    def move_distance(self, direction, meters, ...)  # 闭环：盯 wheel_odom 到位停
    def turn_by(self, angle_deg, ...)                # 闭环：积分 IMU gz 到位停
    def set_camera(self, direction, degrees)         # 云台
    def read_state(self) -> dict                     # 线程安全快照
    def close(self)
```

**三条内部规矩：**

- **安静 pump**：25Hz 发帧线程**只在有非零目标时**才发；目标归零就停止发帧，让底盘 200ms 看门狗自己停。这样语音助手在不动车时完全不占用 `cmd_vel`，前后端 / teleop / 其他控制器随时能接管，不会出现多发布者抢占抖动。
- **超限要报出来，不静默截断**：驱动的 `max_vx = 0.5` 会静默截断。工具自己先截断，并把这件事写进返回结果（「已按上限 0.5 m/s 执行」），否则模型会以为车真的按 0.8 在跑。
- **闭环运动跑在独立线程上，工具立刻返回**（★ 2026-09-17 实车后订正）。

  初版把 `move_distance` / `turn_by` 写成**阻塞直到到位**，理由是「符合同步串行的
  架构」。**那是错的，而且是危险的错。** 唤醒模块的串口监听和整条对话流水线
  **共用一个线程**——见 `wakeword/engine.py` 的 `_listen_loop → _handle_detection`，
  它是**内联**调回调的，那个函数自己的注释就写着「回调里同步跑完了整条流水线」。

  所以只要在工具里等车走完，串口就没人读，用户喊唤醒词进不来，**急停失效**。
  实车复现：「他在往前走的时候，喊他没用」。

  改法与 `MusicController` 完全同构（`tools/music.py` 的模块文档第一段讲的就是
  这件事，**我读过却没套用到小车上**）：运动起一个 `car-motion` 线程，工具只
  返回「已开始…，走完自动停」。

  停下来有三条路，都不经过工具：唤醒词 → `stop_all()`；`car_stop` 工具；
  新指令 → `_preempt()` **顶掉**旧指令（而不是回「正忙」——语音场景下用户改主意
  是常态）。

  代价：工具拿不到最终结果，只能说「已开始」。**结局记在 `last_outcome` 里**，
  `car_status` 会带上它（「上一次动作：前进 1 米 完成」）。

### 唤醒词 = 急停

`_on_wake_word` 现在第一件事是 `self._music.stop()`，加一句 `self._car.stop_all()` 即同构。

- **唤醒词路径**：硬件串口触发，**不经 ASR / LLM**，几十毫秒。这是急停。
  ⚠️ 它的前提是**运动不占唤醒线程**（见上一条）——初版阻塞实现下这条路径是死的。
- **`car_stop` 工具**：走 LLM 往返，约 1~2 秒。这是「停一下」这类自然表达。

两条都要——一条快一条自然，语义不同。

## 5. 驱动改动：电压上 ROS

**不需要碰 STM32 固件，不需要改协议。** 电压在链路上已经通到解析层了：

| 层 | 状态 |
|---|---|
| STM32 固件 | ✅ 已在发（上行帧第 10 字段，毫伏） |
| `protocol.py` | ✅ 已解出（`Uplink.voltage`，V） |
| `monitor.py` | ✅ 已在打印 |
| `driver_node.py` | ❌ **到这中断**：`_publish_fault()` 只发 `[fault, crc_err, head_err, dropped]` |

改动：`_publish_state()` 里多发一条 `std_msgs/Float32` 到 `battery_voltage`。约 5 行。

**该话题不受 `publish_diagnostics` 参数控制**——电压是核心健康指标，应始终发布。

## 6. 配置

新增 `config.yaml` 的 `car:` 段，不把数值写死在代码里：

```yaml
car:
  default_speed: 0.3      # car_move 未指定 speed 时用，m/s
  max_speed: 0.5          # 驱动限幅 max_vx
  camera_step_deg: 15     # car_camera 固定步进
  turn_speed_dps: 60      # car_turn 未指定 speed 时用，度/秒
  max_turn_speed_dps: 120
  cmd_rate_hz: 25         # pump 发帧频率（驱动 cmd_rate_hz=20，留余量）
```

理由：记忆里那些数值（`0.15` 保守速度、`turn_gain=4.0`、`track_eff=1.000`）全都因为硬化在代码里，每次调整都要改代码重部署。将来标定完成或供电改善，应当只改 yaml。

## 7. 已知坑

1. **systemd 用户单元不加载 ROS 环境。** `rclpy` 需要先 source `/opt/ros/jazzy/setup.bash` 和 `~/l150pro_ws/install/setup.bash`。这是记忆里「PATH 不含 `~/.local/bin` 导致 MCP server 静默降级」的同类坑——**失败会静默**。因此 `CarController.start()` 要做显式检查并大声报错，不能等 `ImportError` 自己冒出来。

2. **闭环精度依赖尚未完成的前提。** `move_distance` 依赖里程计标定、`turn_by` 依赖 IMU gz 积分。按「理想情况」设计接口，但**实车验证前，这两条路径的精度不可依赖**。

3. **`rclpy.init()` 是进程级、只能调一次。** 本次不受影响（前后端是独立进程）。但如果将来把 web API 合并进同一个进程，必须共用同一个 `CarController` 实例，不能各建一个。

4. **多发布者抢占。** 若语音助手和 teleop 同时对 `cmd_vel` 发帧，驱动是「最后到达的赢」，车会抖。「安静 pump」是本次的对策。

## 8. 验收标准

- [ ] `battery_voltage` 话题在 `ros2 topic echo` 下能读到与现实相符的电压
- [ ] 5 个工具出现在发给 LLM 的 `tools=` 数组里（总数 14）
- [ ] `car_status()` 能返回电压与故障码
- [ ] 云台三个方向 + 回中 + 指定角度，实车动作正确（ch5 水平 / ch6 俯仰）
- [ ] `car_move(forward, 2)` 实车走约 2 米后停
- [ ] `car_turn(90)` 实车转约 90 度
- [ ] 车在动时喊唤醒词，车立即停（几十毫秒级，不经过 ASR/LLM）
- [ ] **`system_prompt` 与改动前逐字一致**
- [ ] 车静止时 `ros2 topic hz /cmd_vel` 无输出（安静 pump）
