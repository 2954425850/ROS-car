# 2D 导航（nav2 + AMCL）— 2026-11-18 方案已实现，待实车验收

> **状态**：2026-11-17 第一版（NavFn + DWB）实车跑过：去程 5.1 m SUCCEEDED，
> 停车误差 ~5 cm；返程因 DDS 心跳延迟 ABORTED。
> **2026-11-18 按《导航方案最终规划》修订版重写**：Smac Hybrid-A* + RPP、
> 禁止原地转、不优先贴边、composition、位姿持久化。
> **本版还没有上过车**——第一次上车的验收步骤见文末。

## 启动

```
ros2 launch /home/cy/ros2_lidar/launch/nav.launch.py use_composition:=False
# composition 目前不稳（container load_node 超时），默认走多进程     # 排障回退（多进程）

# 换地图（默认仍是 maps/nav_20261124.yaml）：
ros2 launch /home/cy/ros2_lidar/launch/nav.launch.py \
    map:=/home/cy/ros2_lidar/maps/map_20261124_upper.yaml use_composition:=False
```

SLAM 与导航**不能同时跑**（都发 `map->odom`）。`nav.service` / `slam.service`
用 `Conflicts=` 互斥。

## 启动了什么

| 节点 | 作用 |
|---|---|
| `tools/lidar_reset.py`（前置） | STOP + RESET 雷达；避免重启后卡在 `getDeviceInfo` |
| `rplidar_composition` | 雷达 7 Hz；`auto_standby=false`、by-id 串口 |
| `tools/scan_self_filter.py` | `/scan` → `/scan_filtered`，滤掉车体 footprint 内的机械臂回波 |
| `scan-gate.service`（systemd 常驻） | `tools/scan_motion_gate.py`：`/scan_filtered` → `/scan_motion_guard`，只保留当前运动方向扫过的车体；喂 collision_monitor。**不在 launch 里** |
| `robot_state_publisher` | `base_footprint -> base_link -> laser` |
| nav2 `localization_launch.py` | `map_server` + `amcl`，composition 容器 |
| `nav2_navigation_launch.py`（本地副本） | controller / planner / bt_navigator / behaviors / velocity_smoother / collision_monitor |
| `tools/no_pivot_guard.py` | 硬禁原地转：`|w| <= |v| / 0.5` |
| `tools/pose_persist.py` | `/amcl_pose` 存盘；启动时发 `/initialpose` |

不再启动 smoother / route / waypoint / docking。

## 速度链路（优先级由 cmd_mux 仲裁）

```
controller_server -> cmd_vel_raw -> velocity_smoother -> cmd_vel_smoothed
  -> no_pivot_guard -> cmd_vel_guard -> collision_monitor -> cmd_vel_nav
  -> cmd_mux（导航 10，最低） -> cmd_vel_mux -> 驱动

/scan -> scan_self_filter -> /scan_filtered
  -> scan_motion_gate (scan-gate.service) -> /scan_motion_guard
  -> collision_monitor
/scan_filtered 同时给 AMCL / slam_toolbox / 代价图（不经过方向门）
```

`nav2_navigation_launch.py` 是官方文件的副本，把 `cmd_vel -> cmd_vel_nav`
改成了 `cmd_vel -> cmd_vel_raw`，最终输出由 collision_monitor 收口到
`/cmd_vel_nav`，不会落到 `/cmd_vel`（legacy 80，会压过语音）。

## 2026-11-18 算法与参数

| 环节 | 选型 | 关键参数 |
|---|---|---|
| 定位 | AMCL likelihood field | 500~2000 粒子；`recovery_alpha_slow=0.01`、`fast=0.2`；初始位姿由 `auto_localize` 给出（score + 命中率双判据），yaml 里不设兜底 |
| 全局规划 | **Smac Hybrid-A\*** | `motion_model_for_search=REEDS_SHEPP`、`minimum_turning_radius=0.5`、`reverse_penalty=2.0`、`change_penalty=0.3`、`cost_penalty=2.0`、`allow_unknown=false`、`tolerance=0.15` |
| 局部控制 | **RPP** | `use_rotate_to_heading=false`、`allow_reversing=true`、`use_cost_regulated_linear_velocity_scaling=true`、`desired_linear_vel=0.25`、`max_angular_accel=0.8` |
| 局部代价图 | **obstacle 层**（不再 voxel） | 3×3 m，`/scan_filtered`，膨胀 **0.7 m**、`cost_scaling_factor=2.0` |
| 全局代价图 | static + obstacle + inflation | **导航模式 `track_unknown_space=false`**（图外当自由）；融合模式仍 true；膨胀 0.7/2.0 |
| 行为 | backup / drive_on_heading / assisted_teleop / wait | **无 spin** |
| 行为树 | `config/bt_nav_to_pose.xml` | 重规划 0.5 Hz；恢复 = 清代价图 → Wait 2 s → BackUp 0.15 m |
| 硬安全 | collision_monitor | stop 区（车体外扩 15 cm）+ 1 m 减速区；scan 超时 0.5 s；**只吃运动方向过滤后的 scan**（见 2026-11-26） |

**不优先贴边**：靠 Smac 的 `cost_penalty` 在“路短”和“离障碍远”之间权衡，
RPP 严格跟随路径；**没有加禁行区/硬性横向距离**（按修订版）。

## 2026-11-24 定位可靠性修补

实车踩到：新地图第一次起导航时，`auto_localize` 全图搜索把位姿选错了
（错 4 m、147°），错位姿打分 0.12 仍低于旧的 0.25 阈值被接受；AMCL 又关闭了
随机恢复粒子，错位姿被锁死；`pose_persist` 把它存盘，下次启动直接"恢复"。

三条修改：

| 改动 | 内容 |
|---|---|
| 位姿文件按地图区分 | `<地图名>_last_pose.json`，规则在 `tools/nav_pose_file.py`；`auto_localize` 与 `pose_persist` 共用。换地图不会再继承别的图的位姿 |
| 双判据接受 | `score < 0.25` **且** 命中率 `>= 50%`（端点在墙 0.1 m 内）。实测正确位姿 score 0.005~0.02/命中 86%，错误 0.08~0.12/命中 37% |
| 等完整扫描 | 有效点 `< 500`（雷达刚复位的第一圈约 200 点）不参与定位 |

参数：`hit_dist=0.10`、`hit_thresh=0.50`、`min_scan_points=500`。


## 2026-11-26 方向感知的近场保护

**现象**：车想往前走，但身后或侧边有墙/家具时原地不动。
`collision_monitor` 的 stop 区是 base_footprint 下的固定矩形
`[[-0.45,-0.28],[0.45,0.28]]`，前后左右全包围；`scan_self_filter` 只清掉
footprint +2 cm 内的点（`Y_MAX=0.22`），所以车侧 22~28 cm（距车体 2~8 cm）、
车后 0.32~0.45 m（距车体 0~13 cm）的点仍然落在 stop 区里，不管此刻是前进
还是倒车都会停车。

**改法**：新增 `tools/scan_motion_gate.py`（由 `scan-gate.service` 常驻），
把 `/scan_filtered` 复制成 `/scan_motion_guard` 再喂给 `collision_monitor`。
门控按当前 `/cmd_vel_guard` 做两件事，只保留“车体会碰到”的点，其余置 `inf`：

- **当前航向走廊**（与速度无关）：前进时保留 `|y| <= 半宽 + 5 cm`、从车尾
  往前无限长的条带；倒车镜像。这样低速时车头 stop 区也不会漏。
- **未来 1 s 的等曲率扫掠**（直行 / 倒车 / 原地转都覆盖）：处理转弯时车角
  会扫向侧边的情况。原地转 0.35 rad/s 时约看未来 20°。

**侧向余量的更正**：第一版 `pad=0.02` 让走廊边界正好等于
`scan_self_filter` 的 `0.22`，所以直行时**一个侧点也到不了** collision_monitor
（所谓“2 cm 余量”实际是 0）。现在拆成 `pad_long=0.02`、`pad_width=0.05`：
走廊半宽 0.25，`scan_self_filter` 清掉 0.22 以内，所以 **2~5 cm 的侧向带仍会
停**（真快蹭上时保护还在），5~8 cm 的侧向带和 0~13 cm 的车后带在直行时被
丢弃。转弯/扫掠仍用同样的 5 cm 横向 pad。

- **TF 用扫描时间戳**（`Time.from_msg(header.stamp)`），与
  collision_monitor 的 stamp + `base_shift_correction` 对齐；不再取“最新 TF”。
- **变号不立刻换掩码**：v/w 符号变化后 0.3 s 内取 旧掩码 ∪ 新掩码（`union_hold`），
  让车按 `max_decel` 先减速；只用新掩码会在车还没停的 6 cm 行程里失去旧方向保护。
- 指令过期（>0.5 s）、TF 失败、`enabled:=false`：**不过滤**，整帧透传。
- 节点挂掉：`scan-gate.service` 的 `Restart=always` 拉起；carhealthd 的
  `scan_gate` 组件会报 `ABSENT`。如果一直起不来，collision_monitor 的
  `source_timeout=0.5 s` 会让车停车（失效安全）。

代价图、AMCL、slam_toolbox 仍然用完整的 `/scan_filtered`，所以侧边障碍物
照样参与规划绕行；只是不再触发近场急停。

参数（`scan_motion_gate.py`）：`half_length=0.30`、`half_width=0.20`、
`pad_long=0.02`、`pad_width=0.05`、`horizon=1.0 s`、`step=0.1 s`、
`union_hold=0.3 s`、`cmd_timeout=0.5 s`、`enabled=true`。
改 footprint 时要同步这里、两个 costmap 的 `footprint` 和
`scan_self_filter.py`。

**验证（2026-11-26 实车 A/B，已通过）**

条件：车后正中约 10 cm 放箱子（`/scan_filtered` 在 stop 区 x≈-0.40 m 有 162
个点，`range_min=0.15` 边缘可见）；车头前方清空；停掉 `no_pivot_guard`，直接在
`/cmd_vel_guard` 发 0.15 m/s；上层/rosbridge/K230/ToF/arm-grasp 为降负载临时停止。
临时发布节点必须 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，与 systemd 门控和
collision_monitor 同一传输；否则假指令根本到不了门控（日志里不会出现 `kept`）。

- `enabled=false`（旧行为）：collision_monitor 报
  `Robot to stop due to stop_zone polygon`，**车不动**。
- `enabled=true`：`gate: kept 0/555 points in sweep (v=0.15 w=0.00)`；随后
  collision_monitor `Robot to continue normal operation`，cmd_mux
  `速度控制权 → nav`；**车前进约 0.2 m**。

结论：车后 10 cm 的箱子在旧行为下确实触发 collision_monitor 停车；门控把它从
collision_monitor 的输入里去掉后，collision_monitor/cmd_mux 放行。第一次
“true 不走”是测试节点没用 UDPv4（假指令没到门控）+ 高负载下一次 invalid source
把 collision_monitor 停住，不是门控的问题。

边界：这次 A/B 用直接速度指令隔离了 planner/起点，验证的是 collision_monitor +
门控这一层；融合模式的 `Start occupied` 是另一个问题（见下一节）。侧向 2~5 cm
带只做了纯几何测试，没有单独做侧边箱子的实车 A/B。

验证日志：`journalctl -u scan-gate -n 20` 应看到
`scan_motion_gate: ... pads 0.02/0.05 m, horizon 1.0 s`；运动时
`gate: kept N/M points in sweep`。

注意：`collision_monitor` 现在只订阅 `/scan_motion_guard`。`scan-gate.service`
必须常驻（`nav.service` 已 `Wants` 它；融合模式需要开机自启或手动
`sudo systemctl start scan-gate`）。单独起 nav2 而忘了门控，scan 源超时会让
车一直停着。回退方法：`sudo systemctl disable --now scan-gate`，并把两个 yaml
里 `collision_monitor.scan.topic` 改回 `scan_filtered`。

## 未知格策略（2026-11-27 更新）

| costmap | 未知格 | 说明 |
|---|---|---|
| nav global | **map_plan 非三值，~221** | 未知不再 255/0；`/map` 的 -1 由转发节点写成 87 |
| nav local | **false（0）**（nav2 默认） | 滚动窗口只有已观测障碍；未知当墙会把窗口填满 |
| fusion global | **map_plan 非三值，~221** | 同上；`track_unknown_space` 保持 false（ab6fe95） |
| fusion local | **false（0）**（nav2 默认） | 同上 |

**为什么用转发而不是改局部**：local costmap 是 3~4 m 滚动窗口，obstacle 层只
标记雷达当前看到的障碍；未知当墙会把窗口填满，所以 local 保持 0。真正需要
“未知有代价”的是两个 **global**：用 `tools/map_plan.py` 订阅 `/map`，把 -1 改成
87，发布 `/map_plan`（transient_local）；global static layer 设
`map_topic: /map_plan`（绝对名；相对名会解析到 `/global_costmap/map_plan`）+
`trinary_costmap: false`，按 `87/100*254 ≈ 221` 换算。
87 这个值让未知格：

- **> `analytic_expansion_max_cost=200`**：Reeds-Shepp 解析扩展（最多 3 m）会绕开
  未知格，不会无惩罚地直接穿过去；
- **< 253**：A* 主搜索和起点足迹检查仍然允许通过，必要时能走。

`track_unknown_space` 保持 false（ab6fe95），因为转发后地图里已经没有 -1；它
对已知格没有影响。`allow_unknown` 仍 false（设计 D2）。

**不改的部分**：`slam_toolbox` / AMCL / `map_server` / `fusion_plan` 继续读原始
`/map`（含 -1），前沿选点不变；local costmap、collision_monitor、scan gate 都
不碰。`fusion_plan` 仍按 -1 选点，只是 Smac 现在会在“必要时”走 221 的未知格。

**为什么不是 200/250**：200 会让解析扩展无惩罚穿未知（更爱走未知）；250 和 220
在 Smac 行走代价（≈距离 × (1 + cost_penalty × cost/252)）上只差 2.7× vs 3.0×，
220 足够“一般不走、必要时能走”，且比 250 留出更多起步余量。没有改
`cost_penalty` / `use_quadratic_cost_penalty`（会同时让车更远离墙）。

**3D/低矮障碍的后续**：臂上相机 + 100×100 ToF（1 m~20 cm）接入 local costmap
和 collision_monitor 的 observation source；相机 SLAM 融合补语义/远处障碍。
在那之前，221 的未知格仍可能在“必要的长绕路”里被走；近场靠雷达平面 + 局部
层，极端情况（雷达平面外、玻璃等）是已知残余风险（用户已接受）。

**起点（2026-11-27 已实测，补丁已落地）**：未知格 221 < 253，起点不会因未知
被拒；真正拒的是**贴墙**。实测（`/global_costmap/costmap_raw`，原点出发垂直
走向最近致命格）拿到 253 的真实含义：

| 距墙中心 | cost | 含义 |
|---|---|---|
| 0.00~0.15 | 171→230 | 正常膨胀衰减 |
| **0.20~0.35** | **253** | inscribed 环，墙外 0.20 m |
| **0.40~0.45** | **254** | 墙本身（地图画了 2 格 = 0.10 m 厚） |
| 0.50~0.65 | 253 | 另一侧同样的 0.20 m 环 |
| 0.70+ | 230→0 | 衰减到自由 |

即 **253 = 墙两侧各 0.20 m 的 inscribed 带**（与 `inscribed_radius=0.20` 精确
对上），不是物理墙。Smac 原来只要**中心**落在 253 就直接判节点无效（
`collision_checker.cpp` 里 `center_cost_ == INSCRIBED || == OCCUPIED`），
所以车侧边（0.20 m）贴进这个带就 `Start occupied`——0.40 m 被拒、0.50 m
成功，与表完全一致。

**补丁（`pi/nav2_patch/`，只编两个包）**：

- `nav2_smac_planner`：判据从 `>= 253` 收紧到 `>= 254`（`LETHAL`）。只有
  footprint **真的压到致命格**才无效；253 的 0.20 m 保守带不再拦截。中心
  在 253 时继续做足迹检查（原有 `footprint_cost >= 254` 分支），不是
  `forceValid`。
- `nav2_regulated_pure_pursuit_controller`：**只跳过当前位姿**那一次碰撞
  否决（位姿是物理事实；代价图说压墙说明地图/定位错）。路径前方的投射
  检查保留，所以“planner 放行、controller 却 Collision Ahead”的第三层
  拒绝不会出现。
- 副作用（有意接受）：**goal 也一起放宽**——goal 有效性走同一个
  `isNodeValid`，现在只拒真压致命格的 goal。以前 goal 落在 253 带会
  `Goal was in lethal cost`，现在会尝试接近。文档在此明示。
- 代价：planner 可以把路径贴到墙边（0.20 m 带内）甚至让足迹擦过墙格。
  近场兜底是 `collision_monitor`（stop_zone 0.90×0.56 m）+ scan gate；
  侧向擦墙不属于“前进方向”，门控会把它过滤掉，这是 2026-11-26 方向门控
  已接受的取舍。

**构建与回退**：

```
bash ~/ROS-car/pi/nav2_patch/build_nav2_overlay.sh   # 约 3~8 分钟
source /opt/ros/jazzy/setup.bash
source ~/nav2_ws/install/setup.bash
ros2 pkg prefix nav2_smac_planner    # 必须是 ~/nav2_ws/install/...
```

`nav.service` / `slam.service` / `upper/run.sh` 都已按上述顺序 source；
`carhealthd` 的 `nav2_overlay` 组件会检查 `ros2 pkg prefix` 是否指向 overlay
（没编/没 source 时 DEGRADED）。回退：删掉 `~/nav2_ws/install` 或把
`nav2_ws/src/navigation2` `git checkout -- .` 后重编；服务会自动退回
`/opt/ros/jazzy` 的 nav2。

验收（只规划不动车 + 一段直线）：

1. 起点距墙 0.10/0.20/0.30 m：planner **SUCCEEDED**；
2. 起点距墙 0.40 m 且车体真的压墙格：**ABORTED**（`Start occupied`）；
3. 上述 SUCCEEDED 场景下发 `NavigateToPose`，`cmd_vel_guard` 非零 ≥1 s，
   `collision_monitor` 不出现 `STOPPED`。

## 明早运行手册（Pi 上）

1. 上电，确认场地清空、电池 ≥ 12.3 V。
2. 同步 + 安装服务（不自动启动）：
   ```
   cd ~/ROS-car
   git status --short          # 有未提交改动先确认是不是自己的；别在 Pi 上直接改代码
   git pull --ff-only
   bash ~/ROS-car/pi/ros2_lidar/deploy_nav.sh --install-units
   ```
   如果当前分支里没有最新 commit（含 `scan-gate.service`），先别硬切分支
   （会动到其它服务的 symlink install），检查 `origin/feat/arm-voice-motion`
   是否已包含这次改动。**不要用 file_upload 往 `~/ROS-car` 塞文件**，只走
   git pull；否则下次 pull 会冲突。
3. 启动导航（二选一）：
   ```
   sudo systemctl start scan-gate   # 首次安装后；之后随开机自启
   sudo systemctl start nav
   # 或前台：ros2 launch ~/ros2_lidar/launch/nav.launch.py
   ```
   `nav.service` 已经 `Wants=scan-gate.service`；融合模式（upper 起的
   `fusion.launch.py`）不会拉起门控，必须确保 `scan-gate` 在跑。
4. 预检：
   ```
   systemctl is-active scan-gate       # active
   ros2 topic hz /scan                 # ~7 Hz
   ros2 topic hz /scan_motion_guard    # 与 /scan 同频
   ros2 param get /scan_motion_gate enabled   # True
   ros2 topic echo /amcl_pose --once
   ros2 run tf2_ros tf2_echo map base_footprint
   ```
   `pose_persist` 会用上次位姿；如果车被搬动过，删
   `~/ros2_lidar/maps/<地图名>_last_pose.json`（例如
   `nav_20261124_last_pose.json`）。**不再有所有地图共用的 `last_pose.json`**。
   **不要靠原地转一圈让 AMCL 收敛**（新方案禁止原地转）；丢了定位就用 PS2
   以低速往前开 0.3~0.5 m，让扫描匹配重新收敛。
5. 验收（20 次往返 = 40 个目标；约 20 分钟）：
   ```
   python3 ~/ros2_lidar/tools/nav_accept.py --rounds 20
   ```
   结果写到 `~/ros2_lidar/maps/nav_accept_*.json`。
   看四个数：
   - `in_place_rotation_samples` **必须 0**
   - `rounds_succeeded` **≥ 18/20**
   - `stop_error_max_m` **≤ 0.10**
   - `lateral_y_mean` 接近 0.45~0.50（软指标）
6. 如果 planner 插件加载失败（`SmacPlannerHybrid` 名字不对），把
   `nav2_params.yaml` 里的 `plugin` 在
   `nav2_smac_planner::SmacPlannerHybrid` 和 `nav2_smac_planner/SmacPlannerHybrid`
   之间换一下；RPP 同理：
   `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` ↔
   `.../RegulatedPurePursuitController`。如果 composition 起不来，加
   `use_composition:=False`。

> 注：`bond_timeout=10 s` 只改了 navigation 的 lifecycle manager（失败发生在
> behavior_server 这一侧）；localization 的 manager 仍是官方默认 4 s，因为
> map_server/amcl 一直稳定。若它也出现心跳丢失，需要复制
> `localization_launch.py` 单独改。

7. 如果门洞/窄处规划失败（膨胀 0.7 m 太贵），先把 `nav2_params.yaml` 里两个
   costmap 的 `inflation_radius` 降到 0.5 再试；致命区只取决于车体尺寸，不会
   因为这一步缩小可通行范围。

## 资源实测（2026-11-18，干净重启、静止）

| 场景 | 4 核占用 | 折合核心 | 内存 |
|---|---|---|---|
| 基础服务（语音/手柄/驱动/臂/TOF） | 17% | 0.7 | 1.6 GB |
| + SLAM（静止） | 17% | 0.7 | 1.7 GB |
| + nav2（14 节点，静止） | 31% | 1.25 | 2.0 GB |

nav2 相对基线只多用 ~0.5 核，CPU 有 68% 空闲。**算力不是瓶颈**；
`load average` 高来自 Fast DDS 线程和 D 状态任务。

## 五个已修的坑

1. **`bt_navigator.default_server_timeout` 官方默认 20 ms**
   DDS 发现稍慢就 abort 整个目标。已改 2000 ms。
2. **机械臂自遮挡**：雷达在车尾 -0.256 m、臂在车头 → `/scan` 0° 固定回波
   0.44 m，collision_monitor 判成障碍永久停车。`scan_self_filter.py` 把
   footprint 内的点置 inf → `/scan_filtered`。
3. **雷达重启卡在 `getDeviceInfo`** → 前置 `lidar_reset.py`（STOP+RESET，DTR 低）。
4. **Fast DDS SHM 端口残留** → nav launch export
   `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。
5. **DDS 心跳/发现延迟（曾误判成 CPU 过载）**：`load average` 高但 CPU 只
   31% 忙；症状是 behavior_server 4 s 没心跳 → lifecycle manager 关掉整组节点。
   修法：重启 Pi 清状态、UDPv4、稳定链路；`bond_timeout` 已放宽到 10 s。

## 2026-11-17 第一版实测（历史）

- 地图 `nav_20261117`：553×139 @0.05（27.6×7.0 m），unknown 82.8%
- 去程 (-1.157,0.556) → (4.0,0.45)：**SUCCEEDED**，停车误差 ~5 cm
- 返程：走到 (0.34,-0.03)，因 DDS 心跳丢失 ABORTED
- 电压 12.36 V → 11.67 V（11.2 V 以下性能减半）
