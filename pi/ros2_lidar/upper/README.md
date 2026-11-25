# upper — 建图/导航入口（临时件）

实时看 `/map`、雷达、位姿，并在页面上起停建图/导航、存图、点图发目标。
后续 App 会做正式的可视化，这个目录可以整个删掉。

## 起停

```bash
# Pi 上
bash ~/ROS-car/pi/ros2_lidar/upper/run.sh
# 浏览器（笔记本/手机）
http://<Pi IP>:8850
```

- 端口默认 **8850**，`--port` 可改。
- 地图目录默认 `/home/cy/ros2_lidar/maps`（不存在时用仓库里的 `maps/`），
  `--maps-dir` 可改。

### 开机自启

```bash
bash ~/ROS-car/pi/ros2_lidar/upper/install.sh    # 装 systemd 单元并立即启动
bash ~/ROS-car/pi/ros2_lidar/upper/uninstall.sh  # 取消自启
```

服务名 `upper`（`pi/systemd/system/upper.service`）。启动后是空闲状态，
建图/导航仍需在页面上点。`systemctl stop upper` 会把它拉起的建图/导航一并停掉。

## 页面操作

| 按钮 | 行为 |
|---|---|
| 开始建图 / 停止 | 以子进程组起停 `mapping.launch.py`，日志 `/tmp/upper_slam.log` |
| 选地图 + 开始导航 | 起 `nav.launch.py map:=<选择的地图> use_composition:=False`，日志 `/tmp/upper_nav.log` |
| 边建边航 | 起 `fusion.launch.py`：slam_toolbox 提供定位，nav2 只跑规划/控制；日志 `/tmp/upper_fusion.log` |
| 自动探索 | 仅融合模式可用；`fusion_nav` 自动选前沿目标，可随时被点图打断 |
| 画边界线 / 清除边界 | 在"不能去"的一侧点至少两个点；线另一侧的目标会被拒绝 |
| 保存地图 | 复用 `tools/save_map_snapshot.py`；融合模式下同时写 `<名字>_last_pose.json` |
| 点图发目标 | 导航或融合模式下点地图；融合模式下未知区会分步推进，可取消 |

**安全边界**：

- 只管理**本页面自己拉起的**子进程；检测到 systemd 的 `slam`/`nav` 或其他实例在跑
  就拒绝并提示，不抢、不 kill 别人的进程。
- 建图与导航互斥；切换时先「停止」。
- 关掉页面服务时会停掉自己拉起的进程（Ctrl-C）。

HTTP API（供以后语音/脚本调用）：`GET /api/maps`、`POST /api/mode
{"mode":"slam|nav|fusion|idle","map":"x.yaml"}`、`POST /api/map/save {"name":"x"}`、
`POST /api/goal {"x":..,"y":..,"yaw":..}`、`POST /api/goal/cancel`、
`POST /api/explore {"enabled":bool}`、
`POST /api/boundary {"anchor":[x,y],"segments":[[x1,y1,x2,y2],...]}`
（`anchor:null, segments:[]` 清除）、`GET /api/boundary`、`GET /api/health`。

融合模式的执行细节在独立节点 `tools/fusion_nav.py`：它订阅 `/map`、TF 和
`/fusion/*`，按"分步追目标"发 `NavigateToPose`。模式探测现在是三态：
`slam_toolbox` 与 `nav2_controller` 同时存在 ⇒ `fusion`。

> **状态**：2026-11-25 已在 Windows 上完成纯算法/后端/页面的单测；
> `fusion.launch.py` 与 `fusion_nav.py` **尚未实车验收**。

不连 ROS 只看页面（Pi 关机也能跑）：

```bash
python3 ~/ROS-car/pi/ros2_lidar/upper/server.py --demo
```

## 页面

- 浅色 = 障碍，深灰 = 可通行，更暗 = 未知，红点 = 雷达
- 滚轮缩放、拖拽平移、双击复位
- 地图第一次到达时自动缩放到全图

## 排障

| 现象 | 原因 |
|---|---|
| 状态条「ROS 不可用」 | 没用 `run.sh` 起（没 source ROS 环境） |
| 「等待定位」一直亮 | 没起 slam/nav，或 TF `map→base_footprint` 没发布 |
| 页面为空、没有地图 | 没起建图；`/map` 只有 SLAM 或 map_server 才会发 |
| 地址打不开 | Pi 防火墙 / 不在同一网段；`ss -lnt \| grep 8850` 看是否在听 |

健康检查：

```bash
curl -s http://127.0.0.1:8850/api/health
```

## 删除

```bash
bash ~/ROS-car/pi/ros2_lidar/upper/uninstall.sh   # 先撤 systemd 单元
rm -rf ~/ROS-car/pi/ros2_lidar/upper             # 再删代码
```

除此之外没有别的东西需要清理：没有 sudoers、symlink 或配置文件。
