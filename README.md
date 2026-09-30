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

## 什么入库、什么不入库

**入库**：代码、人写的文档（`docs/*.md`）、配置、素材、systemd 单元、测试。

**不入库**（见 `.gitignore`）：**工具跑出来的东西** ——
- `arm_grasp/docs/*.json` —— `collect.py` / `geom_probe.py` / `jacobian.py` 的实测数据
- `raspot_ws/docs/shots/*.jpg` —— `rec_sample.py` 拍的照
- `k230-vision/*.png` —— `fov_analyze.py` / `t_camcalib.py` 画的分析图
- 日志、`build/`、`install/`、`*.tar.gz`、`*.raw`、`*.bin`
- `voice-chatbot/config.yaml`（含 `api_key` 等密钥，只入库 `config.yaml.example`）

**判据**：这文件是脚本「写」的，还是人「写」的？脚本写的就不入库。
（`people.json` 虽是 .json，但它是人脸库配置、由人维护，**入库**。）

**这样做的两个好处**：跑采集/标定**不会弄脏工作区**；而且
`git checkout -- .` / `git stash` **永远不会冲掉你的测量数据**（untracked 文件 git 不碰）。

### 想固化某次测量结果

产物默认不进库。要留住某一次结果，**显式**加进去（`-f` 是必须的，文件被 .gitignore 挡着）：

```bash
git add -f pi/raspot_ws/src/arm_grasp/docs/samples-2026xxxx-xxxxxx.json
git commit -m "data: <这次测了什么>"
```
