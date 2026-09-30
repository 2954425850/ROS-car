# 唤醒握手重构 + K230 拍照上云工具 实现计划

设计：`docs/plans/2026-09-18-wake-handoff-and-vision-design.md`（先读它）
日期：2026-09-18
开工快照：`/home/cy/voice-chatbot.bak-20260918-084329.tar.gz`

---

## 0. 每个任务都必须遵守的约定（写给子代理）

1. **本项目不上 git。不要生成 `git commit` / `git add` 之类的步骤。**
2. **任何情况下不得删除、覆盖、移动已有的 `*.bak-*` / `*.tar.gz` 快照，只能新建。**
   「我刚建的」「已被取代的」「里面装着已被推翻的结论的」**都不是删除理由** ——
   没有 git，快照是唯一回滚路径，而装着误判路径的中间态恰恰最该留档。
3. **改任何已有文件之前**，先留一份时间戳副本：`<文件名>.bak-<YYYYMMDD-HHMMSS>`。
4. 每个任务末尾做一次**开工快照**（**新建**，不覆盖）：
   ```bash
   cd /home/cy && tar czf "voice-chatbot.bak-T<N>-$(date +%Y%m%d-%H%M%S).tar.gz" \
       --exclude='voice-chatbot/logs' --exclude='__pycache__' voice-chatbot
   ```
5. 测试一律 `cd ~/voice-chatbot && python3 -m pytest`（`pytest.ini` 已配 `pythonpath = .`）。
6. **任务之间停下来，等人工审查再放行下一个。**

---

## T1 — 唤醒引擎：修订契约（docstring + 特征化测试）

**为什么**：引擎**代码不用改** —— 它本来就只是「读字节 → 调回调」。有问题的是它的
**契约文档**：`start()` 的 Args 里写着「在监听线程上同步执行 —— 这正是我们想要的」，
`_handle_detection` 的 `finally` 注释也建立在「回调里同步跑完了整条流水线」之上。
这两处现在是**主动误导**，必须反过来说。

**改**
- `wakeword/engine.py`：只动注释/docstring，**不动代码**
  - `start()` 的 Args：回调**必须立刻返回**；防重入由调用方负责
  - `_handle_detection()` 的 `finally` 注释：`_drain()` 的语义改为「清掉 stop 期间（~320ms）的陈旧帧」
  - 顶部模块 docstring 补一句：本引擎不负责并发，回调里做什么是调用方的事

**测**：`tests/fakes.py` 加 `FakeSerial`（可预置字节流、可并发喂帧），
新建 `tests/test_wakeword_engine.py`：
- `test_slow_callback_does_not_block_listening` —— 预置两帧，第一次回调阻塞 0.3s，
  断言第二次回调仍被触发（证明监听循环没被回调拖住）。
  ※ 这条**今天就会通过** —— 它是**特征化测试**，用来钉住「引擎侧不阻塞」这个事实，
  防止后人「顺手加把锁」把阻塞加回来。**要写进测试 docstring。**
- `test_drain_discards_frames_accumulated_during_callback` —— 钉住丢帧是有意的
- `test_cooldown_swallows_rapid_second_wake`

**验收**：`pytest` 全绿；`git diff` 语义上只有注释变化（用 `diff` 比对着看）。

---

## T2 — 握手单元 `core/wake_handoff.py`（TDD）

**为什么单独抽一个单元**：`ConversationManager.__init__` 会把麦克风 / 扬声器 / ASR /
TTS / ROS 全起起来，没法单测。而「快慢分离 + 防重入」这段**并发语义**恰恰最需要测试，
所以把它抽成一个不依赖任何硬件的纯 Python 单元（照 `car/kinematics.py` 的做法）。

**新文件** `core/wake_handoff.py`

```python
class WakeHandoff:
    """把一次唤醒拆成「快动作」和「慢流水线」。

    on_wake() 在监听线程上被调用，必须立刻返回。
    快动作（急停/停音乐）同步跑完；流水线丢到后台线程，且同一时刻只允许一条。
    """

    def __init__(self, fast_action, pipeline, *, name="conversation"): ...
    def on_wake(self) -> None: ...          # 监听线程调
    def shutdown(self, timeout: float = 5.0) -> None: ...   # join 流水线线程
    @property
    def is_busy(self) -> bool: ...
```

**语义（写进 docstring）**
- `on_wake()`：先跑 `fast_action()`（不捕获异常？→ **捕获并记日志**，不能让监听线程死）；
  再在锁内判断 busy；空闲 → 起线程跑 `pipeline()`；忙 → 只记一条 info 日志，**不起第二条**。
- `pipeline()` 的 `finally` 里清 busy —— **异常也必须清**，否则助手永久卡在「忙」。
- busy 标志用 `threading.Lock` 保护（`on_wake` 在监听线程、清理在 pipeline 线程）。

**测**：`tests/test_wake_handoff.py`
- 慢 pipeline 不阻塞 `on_wake` 返回（计时断言）
- busy 期间再 `on_wake`：`fast_action` 又被调了一次、`pipeline` 调用计数**不变**
- pipeline 正常结束后，再 `on_wake` 能起第二条
- pipeline **抛异常**后，busy 被清（再 `on_wake` 能起第二条）+ 异常不炸掉线程
- `fast_action` 抛异常时，`on_wake` 不向上抛，且流水线**仍然**会起
- `shutdown()` 能 join 掉正在跑的 pipeline

**验收**：`pytest tests/test_wake_handoff.py -v` 全绿；**无 `time.sleep` 长等待**（用
`threading.Event` 同步，避免 flaky）。

---

## T3 — 接线 `ConversationManager`

**改** `core/conversation.py`（改前先备份）

- `__init__`：建 `self._handoff = WakeHandoff(fast_action=self._wake_fast_action,
  pipeline=self._run_pipeline, name="conversation")`
- 新增 `_wake_fast_action()`：**只**放今天 `_on_wake_word` 的前两行
  （`self._music.stop()` / `self._car.stop_all()`）
- 新增 `_run_pipeline()`：今天 `_on_wake_word` 的**其余全部**搬过来
  （闲置检查 / reset / 提示音 / sleep(0.5) / `transition(LISTENING)`）
- `_on_wake_word` → 改为一行 `self._handoff.on_wake()`（**保留这个名字**，
  它是 `wakeword.start(on_detected=...)` 的入口，也是 `music.py`/`controller.py`
  注释里引用的锚点）
- `shutdown()`：顺序改为 **先 `self._handoff.shutdown()`，再 `self._wakeword.stop()`**
  （否则监听线程可能正在起新流水线）
- 模块 docstring 的「整条流水线是同步串行的……改动时请勿破坏」**必须重写**：
  串行**保住了**（busy 保证单条），但**不再跑在监听线程上**。把这条写清楚，
  否则下一个人又要重新推一遍。

**测**：本任务**不写单测**（manager 起整套硬件）。靠 T2 的单测 + T6 的端到端。
**但要**：`pytest` 全绿（确认没碰坏别的）。

**验收**：`python3 -c "from core.conversation import ConversationManager"` 能导入；
`pytest` 全绿。

---

## T4 — 视觉客户端 `vision/k230_look.py`（TDD）

照 `car/ros_bridge.py` 的形状：把外部依赖（子进程、HTTP）作为**可注入的构造参数**，
测试里换假的。

**新文件** `vision/__init__.py`、`vision/k230_look.py`

```python
class LookError(Exception): ...

class K230Vision:
    def __init__(self, config, *, runner=subprocess.run, http=None): ...
    def snap(self) -> str:          # 返回 JPEG 路径
    def describe(self, question: str) -> str:   # 返回要念的话
    def look(self, question: str) -> str:       # snap + describe
```

**要点**
- `snap`：跑 `config.vision.snap_cmd`，取 **stdout 第一行**（`k230ctl snap` 默认只回一行路径），
  带 `snap_timeout_sec`；退出码非 0 → 抛 `LookError(stderr 首行)`
- 读文件 → **0 字节也当失败**（设计 §4.5）
- `describe`：base64 → `data:image/jpeg;base64,...` → `chat/completions`
  - ★ `max_tokens` 取 `config`（默认 1500）—— **给小了正文就是空的**（设计 §4.3）
  - **正文为空要单独判**：记 `logger.error`（提示 max_tokens / reasoning 吃光），
    返回「没看清」而不是空串
  - `detail` 从 config 取，默认 `low`
- 所有失败 → 抛 `LookError`，**由工具层**转成人话（保持客户端纯粹）

**测**：`tests/test_k230_look.py`（假 runner + 假 http）
- snap 成功 → 返回路径（含 stdout 带尾随空行的情况）
- snap 退出码非 0 → `LookError` 且带 stderr 内容
- snap 超时（runner 抛 `TimeoutExpired`）→ `LookError`
- 文件 0 字节 / 不存在 → `LookError`
- describe 正常 → 文本
- **正文为空 → `LookError`**（而不是返回 ""）
- HTTP 错误 / 超时 → `LookError`
- 断言发出去的 payload 里：`max_tokens` 来自 config、`detail` 正确、图片是 data URL

**验收**：`pytest tests/test_k230_look.py -v` 全绿。
**真机冒烟**（可选但推荐）：用 `/tmp/k230/latest.jpg` 直接调一次，确认返回人话。

---

## T5 — 工具 `tools/vision.py` + config

**新文件** `tools/vision.py`：`register_vision_tools(registry, vision)`

```python
look(question: str) -> str
```

**description（承担「何时使用」，不动 system_prompt）**，草稿：

> 让 K230 摄像头拍一张照片，交给云端视觉模型看，返回看到的内容。
> 用户问「前面有什么」「这是什么」「看看那边」「帮我看看」「上面写的什么」
> 这类**需要看画面才能回答**的问题时调用。把用户的问题原话写进 question。
> 注意：这个操作要花 4~10 秒。

参数 `question`：`{"type": "string"}`，必填。

**接线 `config.yaml`**：加设计 §5 的 `vision:` 段（改动前 `config.yaml.bak-...`）。
`vision.enabled` 为 false 时**不注册**工具（照 `llm.tools_enabled` 的做法）。

**测**：`tests/test_vision_tools.py`
- 工具出现在 `registry.schemas()` 里，名字/参数形状对
- handler 正常 → 返回文本
- `LookError` → 返回**人话**（不是抛异常、不是空串）—— 断言含「没拍成/看失败」之类字样
- **未登记的工具名不会静默成功**（沿用 registry 的既有保证，顺带回归）
- `enabled: false` → 不注册

**验收**：`pytest` 全绿。

---

## T6 — 接线 `ConversationManager` + 端到端验收

- `core/conversation.py`：建 `K230Vision(config)`，`register_vision_tools(self._registry, vision)`
  （位置同 `register_car_tools`；**vision 起不来不该拖垮助手**，照 car 的做法只记错误）
- 重启：`systemctl --user restart voice-chatbot`，看 `journalctl --user -u voice-chatbot -n 50`

**端到端验收（对着设计 §6 逐条过）**
- [ ] 日志显示 `look` 在 `tools=` 数组里（工具总数 +1）
- [ ] 对助手说「前面有什么」→ 它调 `look` → 念出画面内容
- [ ] **★ 核心回归**：说「往前走一米」，在它走/拍照期间喊唤醒词 → 车**立刻停**；
      日志可见「忙，只急停」而**没有**第二条流水线的记录
- [ ] 盲窗内喊唤醒词不再被丢弃
- [ ] 拔掉 K230（或 `snap_cmd` 指向不存在的东西）→ 助手说人话兜底，不崩不空转
- [ ] 说完话后正常唤醒 → 对话流程与改动前一致
- [ ] `system_prompt` 与改动前**逐字一致**（`diff` 备份确认）

---

## 任务依赖

```
T1 ─┐
    ├─→ T3 ─→ T6
T2 ─┘         ↑
T4 ─→ T5 ─────┘
```

T1/T2 可并行；T4/T5 可并行；**T3 依赖 T2**；**T6 依赖全部**。
