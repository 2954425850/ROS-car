# carhealth — 整车健康守护（一期）

设计见 `docs/plans/2026-11-17-car-health-guard-design.md`。
一期**只观察、只记录，绝不执行任何恢复动作**。

## 安装（Pi 上）

```bash
sudo cp ~/ROS-car/pi/systemd/system/carhealthd.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now carhealthd
sudo ln -sf ~/ROS-car/pi/carhealth/carhealthctl /usr/local/bin/carhealthctl
```

## 命令

```bash
carhealthctl status            # 表格：组件/状态/持续/根因/原因
carhealthctl status --json
carhealthctl doctor            # 出车体检：PASS/WARN/FAIL，退出码 0/1/2
carhealthctl history --since 1h
carhealthctl why tof           # 最近 5 条该组件的完整证据
```

## 一期验收

| 注入的故障 | 期望 |
|---|---|
| `sudo systemctl stop k230-bridge` | `k230_link` 变 `ABSENT`，原因含「k230-bridge 单元不在运行」；**没有任何东西被自动重启** |
| `carhealthctl doctor --no-daemon --fake-throttled 0x1` | `power` FAIL；**底盘/臂/ToF 若本身健康，仍显示 PASS，不被 BLOCKED 掩盖** |
| `carhealthctl doctor --no-daemon --fake-throttled 0x50000` | `power` 只 WARN（历史位，时间未知），**不是 FAIL** |
| `carhealthctl doctor --no-daemon --board-host 203.0.113.1` | `board` FAIL，`k230_link/k230_video` 为 `BLOCKED`，根因 `board` |
| 撤掉 `K230_HOST` 且 `/tmp/k230/resultd-status.json` 无 peer | `board` 为 WARN/UNKNOWN，规则给 `k230.host_unknown`；**不得去 ping 任何写死的旧 IP** |
| `XDG_RUNTIME_DIR=/run/user/$(id -u) systemctl --user stop voice-chatbot` | 6 秒后 `voice` 变 `STALE`，原因含「心跳超过 5s」 |
| 拔掉唤醒模块 USB | `voice` 的 `wakeword_ready=false`，规则给 `voice.wakeword_lost` |
| 拔掉 ToF USB | `tof` 为 `ABSENT`，规则给 `udev`/`usb` 相关结论 |
| `touch pi/raspot_ws/src/arm_grasp/setup.py` 后 `doctor --no-daemon` | 出现 `build.stale` 规则结论 |
| 制造一次 `systemctl restart tof-a010` 并等 60s | `service.crash_loop` 不误报（只涨 1 次不改判据） |
| 臂/底盘动作时看 `status` | `busy` 对应字段为 true；`history` 的 incident 里 `interlock` 记录一致 |
| `ls /run/carhealth/state.json` 且看 SD 卡 IO | 状态文件在 tmpfs；`history` 里的 journal 才在 `~/.local/state/carhealth` |

每条验收后都要确认：`carhealthctl history --since 10m` 里有对应的
`incident_open`，且 `action` 为 `null`；`UNKNOWN` 只产生 `monitor_blind`，
不产生组件 incident。
