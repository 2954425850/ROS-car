# ROS-car

L150Pro 小车 + 机械臂 + K230 视觉 + 语音助手，nepu 树莓派上跑的全部代码。

## 开发方式（2026-09-30 起）

**本地写代码 → push → 树莓派 `git pull`。不再直接在 Pi 上改代码，也不再做 tar 开工快照。**

- 用 git 历史回滚，用分支隔离改动。
- **Pi 上跑的是最新版**；本仓库首个提交就是从 Pi 的当前状态抓下来的。
  如果仓库里的东西和 Pi 上不一致，**以 Pi 为准**。

## 目录

```
pi/                     # 树莓派（nepu）上的东西
├── raspot_ws/          # = ~/raspot_ws    机械臂抓取 (arm_grasp) + PS2 遥控 (raspot_teleop)
├── l150pro_ws/         # = ~/l150pro_ws   L150Pro 底盘 ROS 驱动（含陀螺偏航闭环）
├── k230-vision/        # = ~/k230-vision  K230 视觉链路的 Pi 侧
├── k230-board/         # = ~/_work        K230 板子侧代码（app.py / vision.py / config.py）
├── ros2_lidar/         # = ~/ros2_lidar   RPLIDAR 建图
├── nepu_handheld/      # = ~/nepu_handheld
├── voice-chatbot/      # = ~/voice-chatbot 语音助手 JARVIS（含控车工具）
├── scripts/            # 散在 ~ 下的小车相关脚本
└── systemd/            # systemd 单元 + udev 规则（部署要点）
```

## 不入库的东西

- `voice-chatbot/config.yaml*` —— 里面是 LLM 的 `api_key` 和 `qweather_key`，**这个仓库是公开的，别往里放**。
- `logs/`、`build/`、`install/`、`log/`、`__pycache__/`、`*.tar.gz`
- `*.raw` / `*.bin` —— 抓图原始数据

## 部署（Pi 侧）

Pi 的仓库路径：`/home/cy/ROS-car`。改了代码后：

```bash
cd /home/cy/ROS-car && git pull
```

ROS 工作区需要用 `rosdep`/`colcon` 在 `raspot_ws` / `l150pro_ws` 下构建后再起服务。
运行中的服务：`l150pro-driver`、`ps2-teleop`、`voice-chatbot`、`k230-resultd`、`rosbridge`。
