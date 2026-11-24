# upper — 建图/导航可视化（临时件）

给「看建图」用的最轻入口。**只读**：不启动/停止任何 ROS 进程，不写任何文件。
后续 App 会做正式的可视化，这个目录可以整个删掉。

## 起停

```bash
# Pi 上
bash ~/ROS-car/pi/ros2_lidar/upper/run.sh
# 浏览器（笔记本/手机）
http://<Pi IP>:8850
```

- 端口默认 **8850**，`--port` 可改。
- 建图/导航仍按原方式起：`sudo systemctl start slam` / `sudo systemctl start nav`。
- 本服务只显示「建图中 / 导航中 / 空闲」，不控制它们。

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
rm -rf ~/ROS-car/pi/ros2_lidar/upper
```

没有 systemd 单元、sudoers、symlink 或配置文件需要清理。
