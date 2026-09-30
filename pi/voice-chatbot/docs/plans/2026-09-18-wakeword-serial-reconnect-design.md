# 唤醒模块串口掉线自动重连 设计

日期：2026-09-18
状态：用户 2026-09-18 拍板「根治」
开工快照：`/home/cy/voice-chatbot.bak-20260918-163635.tar.gz`

---

## 1. 背景：一次真实的「假活」故障

2026-09-18 16:03，JARVIS 突然喊不应了。现场证据（全部来自 `journalctl`）：

```
15:43:38  WakeWord: listening on /dev/myspeech @ 115200      ← 开机后一切正常
16:02:52  usb 2-1: can't set config #1, error -71            ← K230 庐山派板子开始报错
16:03:25  usb 2-2.3 / 2-2.4: USB disconnect                  ← 鼠标 + 无线接收器掉线
16:03:29  usb 4-1.2 / 4-1.3: USB disconnect                  ← USB 声卡 + 唤醒模块掉线
16:03:29  ch341-uart ttyUSB0: ch341-uart converter now disconnected from ttyUSB0
16:03:29  ERROR  WakeWord: 串口读取失败，停止监听:
                  device reports readiness to read but returned no data
                  (device disconnected or multiple access on port?)
16:03:31  usb 4-1.3: ch341-uart converter now attached to ttyUSB1   ← 回来了，但换了号
```

而**服务状态一直是绿的**：

```
● voice-chatbot.service - JARVIS Voice Chatbot (语音助手)
     Active: active (running) since Fri 2026-09-18 15:43:22 CST; 50min ago
       Main PID: 1498 (python3)
```

进程里的证据（`/proc/1498/fd`）：

```
85 -> /dev/ttyUSB0 (deleted)
```

### 1.1 因果链

1. **触发**：USB 总线发生全局重枚举（源头是 K230 板子反复 `error -71`，24 小时内
   触发了 13 次 disconnect）。
2. **换号**：CH340 唤醒模块（`1a86:7522`）从 `ttyUSB0` 掉线，回来时被分配成 `ttyUSB1`。
   `/dev/myspeech` 这个 udev 软链**被正确地**改指向了 ttyUSB1 —— 但进程手里攥的是
   **旧 fd**，不是路径，所以软链更新对它毫无意义。
3. **一次异常**：旧 fd 对应的设备节点已被删除，`read()` 抛 `SerialException`。
4. **线程自杀**：`_listen_loop` 在那个异常上 `break`，监听线程当场退出。
5. **systemd 看不见**：`Restart=on-failure` 只在**进程退出**时触发。死的是一个 daemon
   线程，主进程健在，于是重启策略永远不会启动。

结果就是最难受的一种故障：**任何指标都是绿的，但设备是聋的**。

### 1.2 为什么「重启一下就好」不算修好

`/dev/myspeech` 现在正确指向 ttyUSB1，`systemctl --user restart voice-chatbot` 立刻
恢复。但 §1.1 第 1 步已经证明 USB 会**反复**抖（K230 是长期存在的硬件因素），
所以只重启的话下次还会聋 —— 而且同样是在「服务显示正常」的情况下静默聋掉。

---

## 2. 目标与非目标

**目标**：串口掉线 / 换号 / 短暂消失后，监听**自动恢复**，不需要人工重启。

**非目标**（明确不修，避免顺手扩大爆炸半径）：

- **不修 K230 的 `error -71`**。那是独立的硬件/线材问题，换线换口是正解。
  本设计只保证「它抖它的，唤醒词不受影响」。
- **不改 `start()` 打不开就抛错的行为**。那是**故意**的（模块 docstring 第 3 条：
  「原版串口打不开就静默空转，用户完全不知道」）。开机时模块没枚举出来 → 抛错 →
  服务退出 → systemd 10 秒后重启 → 自愈。这条路径本来就对，别动。
- **不给引擎加锁**（见 `tests/test_wakeword_engine.py` 的警告）。内联回调是设计。

---

## 3. 设计

### 3.1 核心：读失败 = 重连，不是终止

`_listen_loop` 的循环条件从「读到异常就 break」改成「只要 `_running` 就继续」：

```
while self._running:
    没有句柄 → _reconnect()（退避重试），失败就继续等
    有句柄   → 读；抛异常 → 关掉坏句柄（_ser = None）→ 下一圈重连
```

**按路径重开**，不是按 fd：`/dev/myspeech` 是 udev 软链，`open()` 每次都会重新解析，
所以 ttyUSB0 变成 ttyUSB1 能自动跟上。**这点是本次修复成立的关键** —— 如果 config
里写死了 `/dev/ttyUSB0`，重连也会连到错的设备上。config 默认值本来就是
`/dev/myspeech`，所以无需改动。

### 3.2 退避：不在失败路径上忙转

- 首次重连等待 `reconnect_initial_sec`（默认 0.5s），之后每次开失败翻倍，
  封顶 `reconnect_max_sec`（默认 5s）。
- 为什么首次也要等：`open()` 成功但 `read()` 立刻失败（比如「端口被别的进程占用」
  那一支）会变成热循环 —— 每圈都开→读→炸。加一个下限就不可能有热循环。
- 为什么 0.5s 不嫌长：USB 重枚举实测要 1~2 秒，首次 0.5s ≈ 白等一小会儿。

### 3.3 等待必须可中断（`_stop_event`）

退避的等待用 `self._stop_event.wait(delay)`，**不是** `time.sleep(delay)`。

因为 `stop()` 里是 `join(timeout=2.0)`：如果监听线程正卡在一个 5 秒的 `sleep` 里，
join 会白等 2 秒然后超时返回，而线程还活着 —— `stop()` 就变成「没关干净」。
用 Event 的话 `stop()` 一 `set()`，线程立刻醒来退出。

### 3.4 句柄的生命周期归监听线程

`serial.Serial` 的 open/close **只在监听线程里发生**（`start()` 里那次 open 是例外，
那发生在起线程之前）。`stop()` 只负责「让它停」，真正 `close()` 在线程的 `finally` 里。

这样就不会出现「`stop()` 刚关掉句柄、监听线程下一微秒又把它打开」的竞态。
`_close_serial()` 是幂等的（置 None 后再调是空操作）。

### 3.5 日志分级：一次故障一条 warning

掉线可能持续很久（用户把模块拔了）。每 5 秒一条 warning 会把 journal 刷满。

- **一次故障的第一条**读失败 → `warning`（带原始异常，用户看得见）；
- **同一故障的后续**失败（重开失败 / 又读到异常）→ `debug`；
- **恢复** → `info`（带第几次尝试成功的）。

`logged_disconnect` 标志在**一次成功的读**之后才复位 —— 不是重开成功就复位。
因为「开成功但读立刻炸」那条路径如果重开就复位，会退化成每 0.5 秒一条 warning。

### 3.6 新增 config 键

```yaml
wakeword:
  reconnect_initial_sec: 0.5   # 掉线后首次重连的等待；之后每次失败翻倍
  reconnect_max_sec: 5.0       # 退避上限
```

两个键都走 `config.get(..., 默认值)`，**老的 config.yaml 不加也能跑**。
存在的意义是测试要把它压到毫秒级（不然每个重连用例都要真等 0.5 秒）。

---

## 4. 故意不做的事（免得后人补测试）

- **不测「重连后解析器状态复位」**。`_reconnect()` 里有 `self._step = _SYNC1`，
  但 `_feed()` 的设计是「0xAA 可以从任意状态重新起帧」，所以**去掉那行也测不出来**。
  写一个声称提供保护、实际提供不了的测试，比没有更糟
  （同 `tests/test_wakeword_engine.py` 里删掉 `test_slow_callback_...` 的理由）。
  那行保留是表达意图，不假装有测试覆盖。
- **不测 `start()` 打不开时的报错文案**。既有行为，本次没动。

---

## 5. 验收

1. `pytest` 全绿，且新增用例**逐个做变异自检**（把修复改回去必须变红）。
2. 部署后 `systemctl --user restart voice-chatbot`，确认监听线程活着。
3. **真机复现验收**：拔掉唤醒模块 USB → 等几秒 → 插回（会换设备号）→
   喊唤醒词必须能唤醒，且全程**不重启服务**。这是本次修复唯一算数的证据。

---

## 6. 验收结果（2026-09-18 真机，已通过）

用户手动拔掉唤醒模块 USB → 等 5 秒 → 插回（换了 USB 口）。全程**没有重启任何服务**。

### 6.1 证据

内核侧（`journalctl -k`）：

```
16:39:52  usb 4-1: USB disconnect, device number 7
16:39:52  usb 4-1.3: ch341_read_int_callback - usb_submit_urb failed: -19
16:39:52  ch341-uart ttyUSB1: ch341-uart converter now disconnected from ttyUSB1
16:39:59  usb 4-1.3: New USB device found, idVendor=1a86, idProduct=7522
16:39:59  usb 4-1.3: ch341-uart converter now attached to ttyUSB0      ← 换了号！
```

应用侧（`journalctl --user -u voice-chatbot`）：

```
16:39:52  WARNING | WakeWord: 串口读失败，进入重连: device reports readiness to
                     read but returned no data (device disconnected or multiple
                     access on port?)              ← 与事故现场一字不差
16:40:00  INFO    | WakeWord: 串口已重连 /dev/myspeech @ 115200（第 4 次尝试）
```

进程与句柄：

```
主进程 PID = 7990，启动时间仍是 16:38:53        ← 没重启，真自愈
修复前 fd: 85 -> /dev/ttyUSB1 (deleted)
修复后 fd:    -> /dev/ttyUSB0                   ← 跟上了新设备号
```

### 6.2 退避时序核对

断开时刻 t=0（16:39:52），设备在 t≈7s 回来，第 4 次尝试成功：

| 尝试 | 时刻 | 累计等待 |
|---|---|---|
| 1 | t≈0.5s | 0.5 |
| 2 | t≈1.5s | 0.5+1.0 |
| 3 | t≈3.5s | +2.0 |
| 4 | t≈7.5s | +4.0 ← 成功 |

与 `reconnect_initial_sec=0.5` 起、每次翻倍的预期完全一致。

### 6.3 结论

1. 设备换号（ttyUSB1 → ttyUSB0）后，引擎**按路径重开**跟上了新设备 —— §3.1 的核心假设成立。
2. 全程进程没退出，所以**不依赖 systemd 的 `Restart=on-failure`** —— 这正是修之前缺失的那一环。
3. 单测（含变异自检）证明了逻辑；本节证明了「真串口 + 真 udev 软链」这条集成路径。

### 6.4 尚未验收的一小步

重连成功后**还没有在真机上喊过唤醒词**（验收当时只做了拔插）。所以「新句柄能真正
收到唤醒帧」这一环，目前由单测
`test_read_error_reopens_serial_and_listening_survives` 覆盖，真机未复核。
下次有人在机器旁边时补喊一声、确认日志出现 `WakeWord: detected` 即可。
