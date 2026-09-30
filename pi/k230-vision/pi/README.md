# K230 Pi 侧（拍照 + 结果接收）

从树莓派上操作 K230 的两件事。**不含 `/dev/video0`**（用户 2026-09-17 决定延期），
**不含任何媒体转发**（视频上云是云端的事）。

```
pi/k230pi.py   库
pi/k230ctl     命令行（唯一入口）
```

## 快速上手

```bash
# 1) 拍一张照片（成功时 stdout 只有一行路径，方便 shell 取用）
IMG=$(/home/cy/k230-vision/pi/k230ctl snap)
echo "$IMG"                     # /tmp/k230/snap-20260918-081234.jpg
# 稳定路径始终指向最近一张：/tmp/k230/latest.jpg
#   ⚠️ 注意：/tmp 是**开机清空**的（实测 /usr/lib/tmpfiles.d/tmp.conf 里是 `D /tmp ... 30d`），
#   所以重启后这两个文件会消失，要等下一次 snap / 下一条结果才回来。

# 2) 看最近一条视觉结果（K230 的 KPU 推理结果）
/home/cy/k230-vision/pi/k230ctl result --json

# 3) 看状态（resultd 在不在、连没连上、照片有没有）
/home/cy/k230-vision/pi/k230ctl status
```

`k230ctl snap --help` 看全部参数。

## 常驻的结果接收

`k230-resultd.service`（systemd，**已 enable**，开机自启）：

```bash
systemctl status k230-resultd
journalctl -u k230-resultd -f
```

它监听 **8556**，收 K230 推来的「换行分隔 JSON」，只保留**最近一条**：

| 产物 | 内容 |
|---|---|
| `/tmp/k230/latest-result.json` | 最近一条结果（原子更新，~10 Hz） |
| `/tmp/k230/resultd-status.json` | 守护进程状态（连接数、条数、坏行数、最近帧号） |

**单客户端**：K230 只会有一条连接。断连是常态，守护进程会自动等它重连。

## 为什么是这个行为（三个容易踩的点）

1. **照片永远是「最后一帧」，不是第一帧。**
   K230 的 RTSP 流**开头是 P 帧不是 IDR**（实测 `AUD SPS PPS P AUD P ...`），
   第一帧没有参考帧、一定是坏的。所以 `snap` 按帧数攒够一整个 GOP 再取**最后一张完整 JPEG**。
   附带好处：gst 进程中途死掉也不影响，落盘的最后一帧仍然是完整的。

2. **等的是「帧数」不是「秒数」。**
   板子的帧率取决于**它的推流有没有接收端**（云端接上后才有）：
   有接收端 ~30 fps（30 帧约 1 s），**没有接收端只有 ~8.4 fps（30 帧约 3.6 s）**。
   按秒数等会在后者跨越不到 IDR 而抓到坏帧。

3. **结果上报频率同样取决于接收端。**
   无接收端时板子主循环被拖慢，结果只有 **~0.9 Hz**；有接收端时回到 **~9.5 Hz**。
   所以「结果怎么半天才来一条」通常不是 Pi 侧的问题，是板子那边没人收视频。

## 排查

| 症状 | 先看这个 |
|---|---|
| `SNAP FAILED: 没抓到可用的 JPEG（中间产物 0 字节）` | K230 的 RTSP 起手偶发死掉。`snap` 已自动重试 3 次；仍失败就查 `ping 192.168.1.112` 和 8554 端口 |
| `result` 说「还没有收到任何结果」 | `systemctl status k230-resultd`；再看板子能不能到 `192.168.1.108` |
| `status` 报「状态文件 Ns 没更新」 | 守护进程死了（或端口被别的进程占了）：`journalctl -u k230-resultd -n 50` |
| 照片是花屏 | 看 stderr 有没有 `WARN 只攒到 N 帧`；有的话说明没跨过 IDR，抬高 `--min-frames` 或先给板子接个视频接收端 |

`snap --json` 会给出 `frames` / `suspect` / `attempts_used`，排查时比只看文件有用。
