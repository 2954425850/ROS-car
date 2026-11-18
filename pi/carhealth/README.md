# carhealth — 整车健康守护（一期观察 + 二期自愈）

设计见 `docs/plans/2026-11-17-car-health-guard-design.md`。
默认只观察；`CARHEALTH_RECOVERY=on` 时启用二期自愈（白名单 + 互锁 + 冷却/预算 + 验证）。

## 安装（Pi 上）

```bash
# 1) 特权动作脚本（root-owned，sudoers 只放行这一条）
sudo install -m 0755 ~/ROS-car/pi/carhealth/tools/carhealth-recover /usr/local/sbin/carhealth-recover
sudo install -m 0440 ~/ROS-car/pi/systemd/carhealth-sudoers /etc/sudoers.d/carhealth
sudo visudo -cf /etc/sudoers.d/carhealth

# 2) 守护
sudo cp ~/ROS-car/pi/systemd/system/carhealthd.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now carhealthd
sudo ln -sf ~/ROS-car/pi/carhealth/carhealthctl /usr/local/bin/carhealthctl
```

## 命令

```bash
carhealthctl status            # 组件/状态/持续/根因/原因 + 自动恢复状态
carhealthctl status --json
carhealthctl doctor            # 出车体检：PASS/WARN/FAIL，退出码 0/1/2
carhealthctl history --since 1h
carhealthctl why tof           # 最近 5 条该组件的完整证据
carhealthctl recovery off      # 暂停所有自动动作（kill switch）
carhealthctl recovery on       # 恢复自动动作
```

## 自动恢复动作（二期）

| 触发 | 动作 | 安全边界 |
|---|---|---|
| `host.wifi_powersave`（WiFi 省电 on） | `iwconfig wlan0 power off` | 不重启任何服务，忙时也允许 |
| 换网/`ros.discovery`/`ros.data_path` 且 ≥1 组件 BLOCKED | 停旧 `ros2 daemon` + 按依赖顺序重启 `ps2-teleop/tof-a010/arm-grasp/k230-*/rosbridge` + 重启 voice | **互锁全清**（臂/底盘/抓取/语音都不忙）才执行 |
| 仅 `dds.stale_ip`（组件健康） | 只停 `ros2 daemon` | 不重启服务 |
| `voice.wakeword_lost` 持续 ≥60s | 重启 voice | 语音不忙时 |

- 冷却：同一动作 120–600s；全局 10 分钟最多 3 次。
- 动作后 30s 验证；连续失败 2 次 → 熔断 30 分钟，并写 `recovery` 结果。
- 启停静默：守护启动后 60s 内不动作。
- 所有动作先写 journal 再执行：`recovery_attempt` / `recovery_result` / `recovery_verify`。
- 不做：重启 Pi/K230、断电、改配置文件、写 K230 SD、运行抓取/臂指令。

## 一期验收

| 注入的故障 | 期望 |
|---|---|
| `sudo systemctl stop k230-bridge` | `k230_link` 变 `ABSENT`，原因含「k230-bridge 单元不在运行」 |
| `carhealthctl doctor --no-daemon --fake-throttled 0x1` | `power` FAIL；底盘/臂/ToF 若健康仍 PASS，不被 BLOCKED 掩盖 |
| `carhealthctl doctor --no-daemon --fake-throttled 0x50000` | `power` 只 WARN，不是 FAIL |
| `carhealthctl doctor --no-daemon --board-host 203.0.113.1` | `board` FAIL，`k230_link/k230_video` BLOCKED，根因 board |
| 撤掉 `K230_HOST` 且 resultd 无 peer | `board` UNKNOWN + `k230.host_unknown`；不 ping 写死旧 IP |
| 停 voice-chatbot | 6 秒后 `voice` STALE，原因含「心跳超过 5s」 |
| 拔唤醒模块 USB | `wakeword_ready=false` + `voice.wakeword_lost` |
| 拔 ToF USB | `tof` ABSENT + udev/usb 结论 |
| `touch .../arm_grasp/setup.py` 后 doctor | `build.stale` 规则命中 |
| 臂/底盘动作时看 status | busy 字段 true；incident 的 interlock 一致 |
| `ls /run/carhealth/state.json` | 状态在 tmpfs；journal 在 `~/.local/state/carhealth` |

## 二期验收

| 注入 | 期望 |
|---|---|
| `sudo iwconfig wlan0 power on` | 10s 内出现 `host.wifi_powersave`；守护自动关省电；`recovery_verify ok` |
| 换 WiFi/网段（IP 变化） | journal `network_changed`；互锁清空后自动 `restart_stack`；DDS/组件恢复 |
| `sudo systemctl stop tof-a010` 等单点停服务 | **一期不会自动重启单点**；整栈恢复只在换网/图故障时触发 |
| 臂运动中触发图故障 | `recovery_deferred`（interlock），不动手 |
| 连续失败 | 熔断写入 `recovery.blocked_until`，不再重试 |
| `carhealthctl recovery off` | 所有自动动作停止，只记录 |
