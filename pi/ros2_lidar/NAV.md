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
```

SLAM 与导航**不能同时跑**（都发 `map->odom`）。`nav.service` / `slam.service`
用 `Conflicts=` 互斥。

## 启动了什么

| 节点 | 作用 |
|---|---|
| `tools/lidar_reset.py`（前置） | STOP + RESET 雷达；避免重启后卡在 `getDeviceInfo` |
| `rplidar_composition` | 雷达 7 Hz；`auto_standby=false`、by-id 串口 |
| `tools/scan_self_filter.py` | `/scan` → `/scan_filtered`，滤掉车体 footprint 内的机械臂回波 |
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
```

`nav2_navigation_launch.py` 是官方文件的副本，把 `cmd_vel -> cmd_vel_nav`
改成了 `cmd_vel -> cmd_vel_raw`，最终输出由 collision_monitor 收口到
`/cmd_vel_nav`，不会落到 `/cmd_vel`（legacy 80，会压过语音）。

## 2026-11-18 算法与参数

| 环节 | 选型 | 关键参数 |
|---|---|---|
| 定位 | AMCL likelihood field | 500~2000 粒子；`recovery_alpha_slow=0.01`、`fast=0.2`；初始位姿由 `pose_persist` 恢复，yaml 里的只是兜底 |
| 全局规划 | **Smac Hybrid-A\*** | `motion_model_for_search=REEDS_SHEPP`、`minimum_turning_radius=0.5`、`reverse_penalty=2.0`、`change_penalty=0.3`、`cost_penalty=2.0`、`allow_unknown=false`、`tolerance=0.15` |
| 局部控制 | **RPP** | `use_rotate_to_heading=false`、`allow_reversing=true`、`use_cost_regulated_linear_velocity_scaling=true`、`desired_linear_vel=0.25`、`max_angular_accel=0.8` |
| 局部代价图 | **obstacle 层**（不再 voxel） | 3×3 m，`/scan_filtered`，膨胀 **0.7 m**、`cost_scaling_factor=2.0` |
| 全局代价图 | static + obstacle + inflation | unknown 当障碍；膨胀 0.7/2.0 |
| 行为 | backup / drive_on_heading / assisted_teleop / wait | **无 spin** |
| 行为树 | `config/bt_nav_to_pose.xml` | 重规划 0.5 Hz；恢复 = 清代价图 → Wait 2 s → BackUp 0.15 m |
| 硬安全 | collision_monitor | stop 区（车体外扩 15 cm）+ 1 m 减速区；scan 超时 0.5 s |

**不优先贴边**：靠 Smac 的 `cost_penalty` 在“路短”和“离障碍远”之间权衡，
RPP 严格跟随路径；**没有加禁行区/硬性横向距离**（按修订版）。

## 明早运行手册（Pi 上）

1. 上电，确认场地清空、电池 ≥ 12.3 V。
2. 同步 + 安装服务（不自动启动）：
   ```
   cd ~/ROS-car
   git log --oneline -1        # 应能看到 e7c62b5 在历史里
   git pull --ff-only
   bash ~/ROS-car/pi/ros2_lidar/deploy_nav.sh --install-units
   ```
   如果当前分支里没有 `e7c62b5`，说明 Pi 不在 `feat/arm-voice-motion`，
   先别硬切分支（会动到其它服务的 symlink install），告诉我或手动把
   `origin/feat/arm-voice-motion:pi/ros2_lidar` 拷过来。
3. 启动导航（二选一）：
   ```
   sudo systemctl start nav
   # 或前台：ros2 launch ~/ros2_lidar/launch/nav.launch.py
   ```
4. 预检：
   ```
   ros2 topic hz /scan          # ~7 Hz
   ros2 topic echo /amcl_pose --once
   ros2 run tf2_ros tf2_echo map base_footprint
   ```
   `pose_persist` 会用上次位姿；如果车被搬动过，删 `~/ros2_lidar/maps/last_pose.json`
   并改 `nav2_params.yaml` 的 `amcl.initial_pose`。
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
