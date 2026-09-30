# 归档区：留档但不要安装

这里的文件**不是给 systemd 用的**，只是留个底，免得 Pi 重装/挂了之后这些历史版本就没了。
内容都是已经被取代的旧版，**别 `cp` 回 `/etc/systemd/system/`**。

## voice-chatbot-system-level.service (+ .bak)

2026-09-09 的**系统级**单元，已被**用户级**单元取代（现行版见 `../user/voice-chatbot.service`）。

它坏在两点，启用也会挂：

1. **直连 ALSA**（不是 PipeWire）—— 声卡被别的进程占住就永远起不来。
   现行版注释里那句「不像直连 ALSA 那版，一旦被别的进程占住就永远起不来」说的就是它。
2. **不 source ROS 2** —— `ExecStart` 是裸的 `python3 main.py`，`CarController` 会
   `ModuleNotFoundError: No module named 'rclpy'`，**小车功能静默失效**。

⚠️ **踩坑预警**：系统级和用户级**同名**。`sudo systemctl enable --now voice-chatbot`
会启到这个旧的，而且会和用户级那份**同时再跑一份**，两份抢唤醒模块串口。
**启动语音助手请用：`systemctl --user restart voice-chatbot`（记得带 `--user`）。**

## voice-chatbot-user.service.bak-car-0917-134335

用户级单元 2026-09-17 加小车工具之前的备份。
