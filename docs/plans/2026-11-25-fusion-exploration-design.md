# 融合建图导航（边建边航）+ 自主探索 — 设计

> **状态**：2026-11-25 定稿；纯算法/后端/页面已在 Windows 实现并通过单测
> （35 个融合测试 + 22 个 upper 测试），**fusion.launch.py / fusion_nav.py 待 Pi 实车验收**。
> **前置**：`pi/ros2_lidar/NAV.md`（nav2 + AMCL 现状）、
> `docs/plans/2026-11-24-upper-visualization-design.md`（网页入口）。
> **定位**：在现有建图/导航之上增加第三种模式 `fusion`：
> slam_toolbox 提供 `map→odom`，nav2 只跑规划/控制（不含 AMCL/map_server）；
> 人点和探索器共用同一条"分步追目标"执行链。
> 本文只定设计与边界，实现步骤见同日期的 plan。

---

## 0. 目标

1. **边建边航**：建图过程中在网页上点一个点，机器人走过去，同时继续建图。
   点在未知区域也能逐步推进，不要求用户先建好再点。
2. **自主探索**：机器人自己选前沿目标，把可通行区域建完；可随时被人工点图打断。
3. **边界线**：在长走廊等场景画一条"不许越过"的线，探索目标和人工点击都受它约束。

非目标见 §14。

---

## 1. 拍板决定

| # | 决定 | 来由 |
|---|---|---|
| D1 | 真融合：slam_toolbox + nav2 navigation（不含 localization） | nav2 官方建图模式；假融合无法满足"过程中持续建图" |
| D2 | 不靠 `allow_unknown: true` 实现"点未知区"；用**分步追目标器** | 用户指出原 M1 验收把核心需求降级；Hybrid-A* 穿墙重规划不可接受 |
| D3 | 导航模式保持硬禁；**融合/探索模式允许慢速原地转**（`allow_slow_pivot=true`、0.35 rad/s） | 2026-11-25 实车：硬禁下 U 型调头在窄处死锁（guard clamp 231、pivot 清零 40 次仍不前进） |
| D4 | 边界线 L1（探索器过滤 + 拒绝线外点击）进主线；L2（KeepoutFilter）以后单独做 | 长走廊场景 L1 已覆盖九成 |
| D5 | 融合模式全局代价图**去掉 obstacle_layer**，只留 static + inflation | 回环修正后全局层旧障碍点会变成幽灵障碍 |
| D6 | 融合模式下存图时同时写当前位姿 | 切回 nav 模式可直接用，不拿旧图位姿初始化 AMCL |
| D7 | 分步执行 + 前沿选点放**独立节点** `fusion_nav`；upper 只管起停/转发/显示 | 语音和脚本可复用，测试面干净 |
| D8 | 长走廊结束条件不能只靠"没有前沿" | 12 m 量程外永远有前沿；玻璃/反光造假前沿 |
| D9 | M1 验收 = 建图过程中点未知区能逐步走过去 | 用户修正 |
| D10 | 工期按原估翻倍 | DDS/雷达历史踩坑 |

---

## 2. 现状与缺口

| 能力 | 现状 | 缺口 |
|---|---|---|
| 建图 | `slam.service` → `mapping.launch.py`（slam_toolbox + 雷达 + TF） | 无导航 |
| 导航 | `nav.service` → `nav.launch.py`（map_server + AMCL + nav2） | 无建图 |
| 网页入口 | `pi/ros2_lidar/upper/`：8850，看 map/scan/pose，起停建图/导航，存图/选图，导航模式点图发目标 | 无融合模式；建图中不能点图；无探索；无边界 |
| 模式探测 | `upper/server.py::detect_mode()` 先匹配 `slam_toolbox` 就返回 mapping | 两者同时跑时会被判成 mapping，需要三态 |
| 速度链路 | `cmd_vel_raw → smoother → no_pivot_guard → cmd_vel_guard → collision_monitor → cmd_vel_nav → cmd_mux` | 融合模式原样复用 |
| 存图 | `tools/save_map_snapshot.py` 写 pgm/yaml/png | 不写位姿 |
| 位姿文件 | `<地图名>_last_pose.json`，格式 `{"x","y","yaw"}`（`tools/nav_pose_file.py`） | 融合模式无人写 |

**关键事实**：AMCL 与 slam_toolbox 都发 `map→odom`，所以 `slam.service` / `nav.service`
用 `Conflicts=` 互斥。但 nav2 的规划/控制不依赖 AMCL，只依赖 map 系和代价地图，
因此"融合"不是把两个服务硬拼，而是新增第三种启动组合。

---

## 3. 三种模式与启动组合

| 模式 | 启动内容 | map→odom 来源 | 定位 |
|---|---|---|---|
| `mapping` | `mapping.launch.py` | slam_toolbox | 纯建图 |
| `navigating` | `nav.launch.py` | AMCL | 静态地图导航 |
| `fusion`（新） | `fusion.launch.py` | slam_toolbox | 边建边航 + 探索 |

`fusion.launch.py` 组合：

```
lidar_reset → rplidar_composition → scan_self_filter → /scan_filtered
robot_state_publisher（base_footprint → base_link → laser）
slam_toolbox（online_async，slam_params_fusion.yaml）
nav2_navigation_launch.py（nav2_params_fusion.yaml，use_composition:=False）
no_pivot_guard（原样）
fusion_nav（新节点）
FastDDS：FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

**不启动**：map_server、AMCL、`pose_persist`、`auto_localize`。
速度链路、`collision_monitor`、`cmd_mux`、`/cmd_vel_nav` 全部不动。

三种模式互斥：`slam.service` / `nav.service` 已用 `Conflicts=`；新增的 fusion
由 upper 的 `ModeController` 以子进程组管理，同样先检查外部实例再拒绝。

### 3.1 模式探测

`detect_mode()` 改为三态：

```
slam_toolbox 在 且 nav2_controller 在  → fusion
只有 slam_toolbox                       → mapping
只有 nav2_controller                    → navigating
都没有                                   → idle
```

`_external()` 也要把 `fusion` 视为外部实例；`ModeController.start("fusion")`、
`save_map()`、`send_goal()` 的模式门都要加 fusion。

---

## 4. 组件架构

```
┌──────────────────────────── upper（8850，网页 + 进程管理）────────────────────────────┐
│  index.html：地图/雷达/位姿/目标/腿/探索状态/边界线                                      │
│  server.py：起停 fusion.launch.py；转发人点；显示 /fusion/status                          │
└───────────────┬───────────────────────────────────────────────────────────────────────┘
                │ /fusion/target（人点）      /fusion/explore（开关）   /fusion/boundary
                ▼
┌──────────────────────────── fusion_nav（独立 rclpy 节点）──────────────────────────────┐
│  目标来源 A：人点                                目标来源 B：frontier explorer            │
│        └──────────────┬──────────────────────────────────┘                            │
│                 staged_goal 执行器                                                      │
│  订阅 /map、TF map→base_footprint；action client NavigateToPose；发布 /fusion/status、    │
│  /fusion/leg                                                                           │
└───────────────────────────────────────┬───────────────────────────────────────────────┘
                                        │ NavigateToPose
                                        ▼
                            nav2（planner / controller / bt / collision_monitor）
```

纯算法拆到 `fusion_plan.py`，不 import ROS，可在 Windows 上用合成 OccupancyGrid 单测：

- 前沿检测、聚类、评分
- 中间目标选择、预算/迟滞
- 边界线段相交判定
- 点击格判定（free / occupied / unknown）

### 4.1 接口

| 方向 | 话题/服务 | 类型 | 说明 |
|---|---|---|---|
| in | `/fusion/target` | `geometry_msgs/PoseStamped` | 人点的最终目标；覆盖当前探索目标 |
| in | `/fusion/cancel` | `std_srvs/Trigger` | 取消当前腿和最终目标 |
| in | `/fusion/explore` | `std_srvs/SetBool` | 开/关自动探索；关闭不清除最终目标 |
| in | `/fusion/boundary` | `std_msgs/String`（JSON，transient_local） | `{"anchor":[x,y],"segments":[[x1,y1,x2,y2],...]}` |
| in | `/map` | `nav_msgs/OccupancyGrid`（transient_local） | 建图 |
| in | TF `map→base_footprint` | tf2 | 当前位姿 |
| out | `/fusion/status` | `std_msgs/String`（JSON，1~2 Hz） | 状态、原因、前沿数、进度、当前腿 |
| out | `/fusion/leg` | `geometry_msgs/PoseStamped`（transient_local） | 当前中间目标，供页面画 |
| out | `NavigateToPose` | action | 每腿一个目标 |

状态 JSON 示例：

```json
{"state":"following","explore":true,"final":[3.2,0.5],
 "leg":[2.1,0.5],"legs":3,"reason":"",
 "frontiers":7,"covered_m2":18.4,"elapsed_s":312}
```

`state`：`idle` / `computing` / `following` / `exploring` / `blocked` / `done`。

---

## 5. 分步追目标器

### 5.1 判定

输入最终目标 `G`（人点或探索器产生的终点），当前位姿 `P`：

1. `G` 在边界线外 → 拒绝，`reason=outside_boundary`。
2. `G` 在 occupied / 未知且不可达 → 走分步。
3. `G` 在已知 free 且与 `P` 同侧 → 一条腿直接去。

### 5.2 选中间目标

在已知 free 且可通行（膨胀后代价可接受）的格子里，选满足以下条件的最远点 `L`：

- 从 `P` 到 `L` 的直线不被边界线截断；
- `|P-L| >= L_min`（0.8~1.0 m）；
- `L` 到最近障碍的间隙 >= 0.6 m（与 `inflation_radius: 0.7` 对齐）；
- `L` 朝 `G` 的方向（夹角 <= 60°）。

找不到 → `reason=no_reachable_frontier`，停止并提示。

### 5.3 迟滞与预算

- **迟滞**：新腿必须比当前腿更接近 `G` 至少 0.3 m，或当前腿已完成/失效，才换腿。
  否则地图每 1~2 s 更新一次会原地抖。
- **预算**：累计路程 <= `max(3 × |P0-G|, 10 m)`，腿数 <= 20。
  超预算 → `reason=unreachable`（点进了封闭房间/被墙挡住）。
- **终止**：`G` 所在格变成 occupied；连续 3 腿没有实质推进（距 `G` 减少 < 0.2 m）；
  人工取消。

### 5.4 目标朝向

每条腿的 goal yaw = `atan2(G.y - P.y, G.x - P.x)`（人点和探索共用）。
融合参数里 `yaw_goal_tolerance` 放宽到 3.15（π，只到位置、不管朝向），`xy_goal_tolerance` 0.2~0.25 m，
避免终点原地找角度。

### 5.5 执行

- 一条腿一个 `NavigateToPose`；同一时间只有一个活动目标。
- 腿成功 → 回到 §5.1 重算；腿失败/ABORT → 当前腿进黑名单，重试一次，再失败则终止。
- 人点到来时：取消当前腿，清探索的最终目标；若 `explore` 仍为开，人点目标完成后恢复探索。
- 取消：取消 action + 清最终目标 + 清中间腿。

---

## 6. 前沿探索器

### 6.1 前沿定义

- 读 `/map`：`free` 为 0..free_thresh，`occupied` >= 65，`unknown` = -1。
- **前沿格** = free 且 8 邻域至少一个 unknown。
- 连通域聚类（4 邻域 BFS）。
- 过滤：
  - 簇小于 `min_cluster_cells`（默认 20 格 = 0.05 m²）；
  - 簇内格到车距离 > `max_frontier_dist`（默认 8 m，量程 12 m 留余量）；
  - 簇中心在边界线外；
  - 黑名单 0.5 m 内（失败目标，保留到地图在该处更新或 10 min）。

### 6.2 评分与选点

```
score = w_gain * min(cluster_size, gain_cap)
      - w_dist * distance
      + w_clear * min(clearance, 0.6)
```

默认 `w_gain=1.0, w_dist=0.6, w_clear=1.5, gain_cap=200`。
选分数最高的簇；**不能直接选前沿边缘**（unknown 的膨胀会把边缘点压到
master 253，Smac 直接 `no valid path`）。从簇质心朝机器人退 0.6~1.2 m，
取第一个 unknown 间隙 >= 0.6 m、障碍间隙 >= 0.6 m、且 `line_free` 的
free 格作为 `G`（2026-11-25 实车修正）。
`G` 的 yaw 用 §5.4。选不到就换下一个簇。

### 6.3 节奏

- 0.5~1 Hz 计算一次；只在没有活动腿时产生新目标。
- 腿成功后立即重算；腿失败先黑名单再重算。
- 不预占、不排队；永远只有一个目标。

### 6.4 结束条件（不能只看"没有前沿"）

满足任一即结束（`state=done`，默认原地停）：

- 连续 3 次计算都没有可用前沿，且地图在最近 30 s 没有增长；
- 已探索面积增长停滞：最近 60 s 新增 free 面积 < 0.5 m²；
- 运行时间超过 `max_explore_time`（默认 30 min）；
- 累计路程超过 `max_explore_distance`（默认 200 m）；
- 电量低（见 §11）；
- 人工关闭探索。

结束不自动返航（M4 再加）。

### 6.5 进度指标

- 已探索面积：free 格数 × resolution²；
- 前沿数；
- 运行时间/路程；
- 当前目标与腿数。
由 `fusion_nav` 统计，经 `/fusion/status` 给页面。

---

## 7. 边界线

### 7.1 L1（本次）

- 表示：一组线段 + 一个锚点（画线时机器人的位置）。
  `{"anchor":[x,y],"segments":[[x1,y1,x2,y2],...]}`，全部在 map 系。
- 判定"线外"：从锚点到候选点的直线与任一线段相交 → 拒绝。
  这样一条横跨走廊的线就是半个平面，不需要用户画闭合多边形。
- 应用点：
  - 人点最终目标：`fusion_nav` 拒绝并回报 `outside_boundary`；
  - 中间目标：选点时排除线外；
  - 探索前沿：聚类中心或候选格在线外则丢弃。
- 页面：新增"画线"模式，点若干点、双击结束；画完可清除。
  线宽只为显示，判定用线段。
- 持久化：存在 `pi/ros2_lidar/upper/boundary.json`（随目录删除即净），
  upper 启动时以 transient_local 重发 `/fusion/boundary`。
- 锚点失效：机器人被搬动/换图后锚点无意义；换图或清除边界时一并清掉。
  第一版不做自动提醒。

### 7.2 L2（以后）

`nav2_costmap_filters::KeepoutFilter` + 独立掩码 OccupancyGrid：

- 掩码有自己的 origin/分辨率，按 map 系查；**不需要**随 slam 地图增长重画；
- 固定覆盖 40×40 m（足以覆盖活动范围）；
- 线宽 >= 0.1 m，叠加现有 0.7 m 膨胀；
- 加到全局和局部 costmap 的 `filters`，不放进 `plugins`；
- 需确认 Pi 上 `nav2_costmap_filters` 已安装（Jazzy 有）。
- 掩码值语义实现时按 nav2 源码核对，不靠记忆。

---

## 8. 参数集

**不改** `config/nav2_params.yaml`、`config/slam_params.yaml`、`slam.service`、`nav.service`。

### 8.1 `config/slam_params_fusion.yaml`

以 `slam_params.yaml` 为底，只改：

| 参数 | 原值 | 融合值 | 原因 |
|---|---|---|---|
| `map_update_interval` | 5.0 | 1.0（先试 1.0，CPU 高再 2.0） | 规划基于旧图会"看不见"新区域 |
| `minimum_time_interval` | 0.5 | 0.3 | 探索需要更快的扫描匹配；先保守 |

其余（`max_laser_range=9.0`、`scan_topic=/scan_filtered`、loop closure 参数）不动。

### 8.2 `config/nav2_params_fusion.yaml`

以 `nav2_params.yaml` 为底，只改：

| 项 | 原值 | 融合值 | 原因 |
|---|---|---|---|
| 全局 `plugins` | static + obstacle + inflation | **static + inflation** | 回环后全局旧障碍点变幽灵 |
| 局部 costmap 尺寸 | 3×3 m | 4×4 m | 去掉全局动态层后给局部更多反应距离 |
| `xy_goal_tolerance` | 0.15 | 0.2~0.25 | 分步腿短，避免终点磨蹭 |
| `yaw_goal_tolerance` | 原值 | 3.15（π） | 融合模式只到位置、不管朝向；窄处对角度会反复磨 |
| `allow_unknown` | false | **保持 false** | 未知区由分步器推进，不靠穿墙规划 |
| 其余（Smac/RPP/collision_monitor/no_pivot） | — | 不变 | 安全层不动 |

> **2026-11-27 更新（用户决定）**：融合 global costmap 的
> `track_unknown_space` 从 `true` 改为 **`false`**（未知当自由），解决起点
> 足迹压未知格时 Smac `Start occupied`。`allow_unknown` 仍保持 `false`；
> `fusion_plan` 仍用 `/map` 的 -1 做选点。真实障碍交给 local costmap /
> collision_monitor，后续由臂上相机 + 100×100 ToF（1 m~20 cm）融合补 3D/
> 低矮障碍。详见 `pi/ros2_lidar/NAV.md` 的「未知格策略」。

`inflation_radius` 先保持 0.7；门洞规划失败再按要求降到 0.5，不主动改。

### 8.3 待实测确认

- **2026-11-25 实测：StaticLayer 正常**。日志持续出现
  `StaticLayer: Resizing costmap to 295 X 159`；`/global_costmap/get_static_layer`
  服务返回的内部数据是 295×159、原点与 `/map` 一致，robot/leg2 都是 0（free）。
  之前从 `costmap` / `static_layer` **话题**读到 284×144 是 transient_local
  缓存旧帧，不代表层失效。
- **真正的坑**：分步选点的 `clearance` 用了 0.35，而设计 §5.2 要求 0.6。
  选出的“free”中间点在 master 上是 253（内切区），Smac 返回
  `no valid path found`（error_code 208）。已改为 0.6。
- 融合模式 CPU/内存：nav2 约 1.25 核、slam 约 0.7 核，合计预计 < 2.5 核 / 4 核。
- 局部图 4×4 与 `collision_monitor` 1 m 减速区是否重叠过多导致频繁停车。

---

## 9. 网页改动（`upper/`）

| 改动 | 说明 |
|---|---|
| 第三模式按钮 | "边建边航"；启动 `fusion.launch.py`；与建图/导航互斥 |
| 建图中点图 | 融合模式下点击即发 `/fusion/target`；未知区允许点 |
| 中间腿显示 | 读 `/fusion/leg`，画当前腿；状态条显示"正在推进到中间点" |
| 规划路径 | 订阅 `/plan`（可选，M1 后） |
| 探索开关 | "自动探索"开/关；显示前沿数、已探索面积、时间、当前目标 |
| 失败原因 | 显示 `outside_boundary` / `no_reachable_frontier` / `unreachable` / `blocked` |
| 边界线 | "画线"模式，双击结束；显示禁区侧或线本身；可清除 |
| 存图 | 融合模式下可用；同时写 `<地图名>_last_pose.json` |
| 地图推送节流 | 融合模式地图 1 s 更新；页面推送上限 1~2 s，避免 WiFi 上每帧几百 KB |

页面保持零外部依赖；不引入 CDN。

---

## 10. 存图与位姿

融合模式下 `pose_persist` 不启动（没有 `/amcl_pose`），由 upper 在存图时写位姿：

1. `save_map_snapshot.py` 写 `<name>.pgm/.yaml/.png`；
2. upper 从当前 `map→base_footprint` 取 `(x,y,yaw)`；
3. 写 `<maps_dir>/<name>_last_pose.json`，格式与 `tools/nav_pose_file.py` 一致：
   `{"x":..., "y":..., "yaw":...}`，原子写（tmp + `os.replace`）；
4. 之后切到 nav 模式选同一张图，`pose_persist` 直接恢复，不拿旧图位姿。

失败时存图仍算成功，但状态里标注"位姿未写入"，不静默。

---

## 11. 失效与兜底

| 情况 | 行为 |
|---|---|
| `map→base_footprint` 缺失 | 停止发新目标；当前腿取消；状态 `blocked` + 原因 |
| `/map` 超过 3 s 没更新 | 不再选新前沿；当前腿继续，collision_monitor 仍保护 |
| 腿 ABORT | 重试一次；再失败进黑名单；连续 3 次终止 |
| 回环导致位姿跳变 | 当前腿可能 ABORT，按上一条重试；不视为故障 |
| 雷达 USB 锁死 | `/scan` 超时 → collision_monitor 停车；`carhealthd` 观测；fusion_nav 无进展 30 s → 取消目标、`state=blocked` |
| 电量低 | 融合模式启动时读电压；< 11.2 V 不开始探索；探索中 < 11.0 V 停止并原地停（M4 加返航） |
| 玻璃/反光 | 雷达可能看穿或造假前沿；低速 + 边界线 + `collision_monitor` 前向停止区；不承诺完全避免 |
| 外部实例 | `ModeController` 检测到 systemd slam/nav 或别处 fusion 就拒绝，不抢不杀 |
| 页面/服务崩溃 | 子进程组随 `upper` 停止；单次探索建议不超过 30 min |

---

## 12. 测试

### 12.1 纯算法（无 ROS，Windows 可跑）

- 合成 OccupancyGrid：前沿检测、聚类、过滤（小簇/远距/黑名单）、评分确定性。
- 中间目标：未知区推进、`L_min`、0.6 m 间隙、迟滞、预算耗尽、目标变 occupied。
- 边界：线段相交、锚点侧判定、线外点击拒绝。
- **变异自检**：故意改坏一个判据，确认对应测试会红（遵守
  `test-must-be-able-to-fail`）。

### 12.2 服务端

- `detect_mode()` 三态（两个 pgrep 都命中 → fusion）。
- `ModeController.start("fusion")` 参数校验、外部实例拒绝。
- `save_map` 写位姿成功/失败两条路径。
- 新 HTTP 接口：`/api/explore`、`/api/boundary`、`/api/goal` 在 fusion 下的路由。

### 12.3 实车验收

| 阶段 | 验收 |
|---|---|
| M0 | 融合 launch 起来；手动 `NavigateToPose` 到 3 个已知 free 点都到；`/map` 持续增长；旧 nav 模式 20 次往返 ≥ 18/20、原地转 0 不回归 |
| M1 | 建图过程中点未知区：分步推进到能到的最远处，地图跟着长；页面显示中间腿与原因；`outside_boundary` 正确拒绝 |
| M2 | 27×7 场地自动探索一轮：记录覆盖率、耗时、里程、卡住次数、假前沿次数；结束条件触发正确 |
| M3 | 画一条横跨走廊的线：探索不越线、点线外被拒、点线内正常；清除后恢复 |
| 回归 | 每次动 `upper`/参数后跑一遍 `tools/nav_accept.py --rounds 20`（nav 模式） |

---

## 13. 里程碑

工期按历史踩坑翻倍。

| 阶段 | 内容 | 粗估 |
|---|---|---|
| M0 | `fusion.launch.py` + 两套参数 + `fusion_nav` 骨架（分步执行，无探索）；纯算法单测 | 2~3 天 |
| M1 | upper 三模式 + 建图中点图 + 中间腿/原因显示 + 存图写位姿 | 2~3 天 |
| M2 | frontier 探索器 + 开关 + 进度 + 结束条件 + 黑名单 | 3~4 天 |
| M3 | 边界线 L1（画线、拒绝、探索过滤） | 1~2 天 |
| M4 | posegraph 续建、低电停车/返航、L2 KeepoutFilter、局部图调参 | 另议 |

M0 必须先在 headless 下验证，不碰 UI。

---

## 14. 非目标

- 语义视觉辅助（`2026-11-24-semantic-vision-nav-design.md` 的内容）不接。
- App、云端上位机（`cloud-upper`）不接；网页的 HTTP API 保持可复用即可。
- 自动充电、自动回桩。
- 多层/3D、电梯。
- 覆盖式（boustrophedon）补扫：先不做，前沿探索不够再议。
- L2 KeepoutFilter 不在主线。

---

## 15. 已知风险

1. **玻璃**：雷达可能直接看穿，不只是假前沿，是真会撞；没有物理护栏时不承诺。
2. **倒车链路**：RPP 负速度要端到端穿过 smoother、`no_pivot_guard`、collision_monitor、
   `cmd_mux` 和驱动；任何一环截掉，D3 的策略就落空。M0 必须实测倒车转弯方向；2026-11-25 实车确认硬禁下 U 型调头会死锁，融合模式已改为允许慢速原地转。
3. **回环幽灵障碍**：已用 D5 规避全局层；局部层仍可能有短暂错位，属可接受。
4. **长走廊永远建不完**：靠 D8 的时间/里程/停滞条件收口，不追求"建满"。
5. **雷达 USB 锁死**：已知硬件坑，长时间探索要有无进展停车兜底。
6. **composition 不稳**：融合 launch 默认 `use_composition:=False`。
7. **地图推送带宽**：融合模式地图更新变快，页面推送必须节流，否则 WiFi 拥塞。
8. **实测未知**：局部图 4×4 是否够、1 s 地图间隔的 CPU、`fusion_nav` 在 640k 格
   地图上的耗时，还要用数据回答。StaticLayer 已在 2026-11-25 实测正常，不再是风险。
