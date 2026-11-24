# 建图/导航可视化入口 — 设计（临时件）

> **状态**：2026-11-24 定稿。**第一层（纯可视化）先做**；第二层（起停/存图）待第一层
> 实车看完再定；语音单独做，不在本文范围。
> **定位**：给「看建图」一个最轻的入口。后续 App 会做正式的可视化，因此本件必须
> **一个目录装完、不加系统服务、不碰现有任何服务，删掉即净**。

---

## 0. 约束（用户拍板）

| # | 决定 |
|---|---|
| D1 | 形态：Pi 上起一个网页，Windows 笔记本/手机浏览器直接看 |
| D2 | 必须轻、简单，**无连带影响**；后续整体移除要干净 |
| D3 | 语音建图/导航是长期件，**本次不做**，用户后续单独做 |
| D4 | 先做可视化；建图仍按现状 `sudo systemctl start slam` |

## 1. 选型对比

| 方案 | 结论 |
|---|---|
| 静态页 + rosbridge | ❌ 依赖 rosbridge 绑 `0.0.0.0`（共享服务，仓库里仍是 `127.0.0.1`）；地图走 JSON 量大；无法扩展控制 |
| **自带 stdlib 服务 + 单页** | ✅ **采用**：自有端口，数据在服务端按 TF 算好再推；不加系统服务；删除即目录 |
| RViz2 | ❌ 不是入口，Windows 端还要装 ROS |

## 2. 目录与安装面

```
pi/ros2_lidar/upper/
├── run.sh        # source ROS 环境后 exec server.py（唯一入口）
├── server.py     # rclpy 节点（后台线程）+ stdlib HTTP/SSE（主线程）
└── index.html    # 单页，零外部依赖（无 CDN / npm / roslibjs）
```

- 端口：**8850**（已与现有服务核对：8080 后端、8555–8559 K230、9090 rosbridge 均不冲突）。
- 启动：`bash ~/ROS-car/pi/ros2_lidar/upper/run.sh`，浏览器开 `http://<Pi>:8850`。
- 删除：`rm -rf pi/ros2_lidar/upper`（Pi 上无 symlink、无 systemd、无 sudoers）。
- **不改**：`slam.service` / `nav.service` / `rosbridge.service` / `slam_params.yaml` /
  `nav2_params.yaml` / APP / cloud-upper。

## 3. 第一层（本次实现）：纯看

### 3.1 服务端数据流

一个 rclpy 节点跑在后台线程（`SingleThreadedExecutor`），HTTP 走主线程：

| 输入 | 来源 | 处理 | 推送 |
|---|---|---|---|
| 地图 | `/map` `OccupancyGrid`（transient_local） | 原样保留；`data` 转 base64(int8) | 变化时（slam 的 `map_update_interval=5 s`），1 Hz 上限 |
| 雷达 | `/scan_filtered` `LaserScan` | 取有效 range，按 `map→laser` 变换到 map 系 | 7 Hz，float32 点对 base64 |
| 位姿 | TF `map→base_footprint` | 取 x/y/yaw | 10 Hz |
| 状态 | 内部 | 模式判定、地图统计、客户端数 | 1 Hz |

- **不订阅 `/pose`**：建图位姿由 `slam_toolbox` 的 TF 给出，导航位姿由 AMCL 的 TF 给出，
  两者都经 `map→base_footprint` 统一，少一个话题依赖。
- 模式判定（只读）：`systemctl is-active slam|nav` + `pgrep -f slam_toolbox|nav2`，
  仅用于状态条显示「建图中 / 导航中 / 空闲 / 外部实例」，**不启停任何进程**。
- TF 缺失时：位姿/雷达帧跳过，页面保留上一帧并标黄「等待 TF」。

### 3.2 HTTP 接口

| 方法 | 路径 | 作用 |
|---|---|---|
| GET | `/` | `index.html` |
| GET | `/events` | SSE 长连接：`map` / `scan` / `pose` / `status` 四类事件 |
| GET | `/api/health` | 便于 curl 排障：ROS 线程、订阅计数、TF 是否可用 |

第一层**没有 POST**，页面只读。

### 3.3 SSE 帧格式

```
event: map
data: {"w":553,"h":139,"res":0.05,"ox":-1.2,"oy":-0.3,"data":"<base64 int8>"}

event: scan
data: {"n":360,"pts":"<base64 float32 x,y 交错>"}

event: pose
data: {"x":1.23,"y":0.45,"yaw":1.57}

event: status
data: {"mode":"mapping","map_w":553,"map_h":139,"clients":1,"tf_ok":true,"note":""}
```

- SSE 用 HTTP/1.1 分块写；每个客户端一个线程（ThreadingHTTPServer），预期 1–3 个客户端。
- 页面断线自动重连（EventSource 自带 + 状态条提示）。

### 3.4 页面

全屏 canvas + 顶部状态条：

- 灰度地图（未知 = 深灰 / 空闲 = 白 / 占用 = 黑），0.05 m/格按设备像素比绘制
- 雷达点叠加（红），机器人箭头（位姿 + 朝向）
- 交互：滚轮缩放、拖拽平移、双击/按钮 fit
- 状态条：连接状态、模式、地图尺寸、分辨率、已建面积占比
- 无外部资源；深色主题；手机横屏可用

## 4. 第二层（暂不做，写完第一层实车看完再定）

- `POST /api/mode {slam|nav|idle}`：以独立进程组启停 `mapping.launch.py` / `nav.launch.py`；
  先查 `systemctl is-active` 与 `pgrep`，有外部实例就拒绝；SIGINT 整组 → 超时 SIGKILL。
- `POST /api/map/save {name}`：复用 `tools/save_map_snapshot.py`。
- `GET /api/maps`；给 `nav.launch.py` 加 `map:=` 参数（默认 `nav_20261124.yaml` 不变）。
- 导航模式点图发目标（`NavigateToPose` action client）。
- **这些是唯一碰进程管理的地方**；如果不做，入口仍然干净。

## 5. 失效与降级

| 情况 | 行为 |
|---|---|
| ROS 线程起不来 | 服务仍起，`/api/health` 和页面显示原因，不静默 |
| 没起建图/导航 | 页面正常，显示「空闲」，没有地图 |
| 没 TF | 位姿/雷达停更，状态条标黄 |
| 浏览器断开 | 服务端自动移除该 SSE 客户端，不积压 |
| 地图过大 | 单帧 base64 上限保护（> 4 MB 时降频并提示） |

## 6. 验收

1. 手动起 `run.sh`，浏览器连上，SSE 稳定不断线
2. `sudo systemctl start slam` 后：地图随车增长，雷达点与地图墙线对齐，箭头朝向正确
3. 刷新页面能立即拿到完整地图（`/map` transient_local）
4. `sudo systemctl start nav`（在 map 上）：显示该地图、AMCL 位姿与雷达
5. 停掉 slam/nav：页面不崩，状态回到空闲
6. 杀掉 `run.sh`：Pi 上无残留进程、无新增 systemd 单元、无端口占用（8850 释放）
7. **对照**：不跑本服务时，现有 slam/nav/语音行为完全不变

## 7. 已知取舍

- `map_update_interval` 仍为 5 s，建图画面 5 s 一跳；如实车觉得慢，再单独议（属主系统参数）。
- 页面不做历史轨迹、不做代价地图、不做目标点（第二层）。
- 地图缩略图/下载不做；存图仍用现有 `save_map.sh`。
