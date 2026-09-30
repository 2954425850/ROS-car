
## Task 4 前置调查：多 VENC 通道（2026-09-17）

**问题**：计划的多通道 `cam.py` 是"每个通道各 new 一个 `Encoder()`"，
但官方 `02-Media/rtsp_server.py` 只用一个 `Encoder()` 配 `VENC_CHN_ID_0`。
两种写法哪种合法？

### 1. 静态扫描：examples 里没有多通道先例

在板上扫 `/sdcard/examples` **全部 344 个 .py**（脚本 `scan_venc3.py`）：

```
PYFILES 344 / OK 344 FAIL 0
TOTAL_MULTI 0        # 没有任何文件同时出现两个 VENC_CHN_ID_n
TOTAL_ENC2 1         # 只有 video_encoder.py 出现两次 Encoder()
TOTAL_RTSP 1         # 只有 rtsp_server.py 提到 rtsp server
```

`video_encoder.py` 的那两次 `Encoder()` 是**两个互不相干的函数**
（`vi_bind_venc_test` / `stream_venc_test`），两个都用 `VENC_CHN_ID_0`，
**不是**同时开两个通道。→ **没有可抄的先例，只能实测。**

另：`VENC_MAX_CHN_NUMS = 4`（硬件支持 4 个通道）；
`dir(Encoder)` 里**没有**任何请求 IDR 的接口（无 `IDR`/`Request` 字样），
`Encoder` 只有 SetOutBufs / Create / Start / Stop / Destroy / GetStream /
ReleaseStream / SendFrame。**`cam.py` 的 `request_idr()` 只能是显式 no-op。**

### 2. 实测：MODE=A 可行，MODE=B 会把板子搞挂（探针 `t_venc_probe2.py`）

每个 MODE 之前都 `machine.reset()` + 等 10 s：

```
MODE=A（每个通道各一个 Encoder()）
  Encoder()#0 -> chn0 / Encoder()#1 -> chn1
  link chn0 OK / link chn1 OK
  Start(chn=0) OK / Start(chn=1) OK
  CHN0 frames=102 packs=102 bytes=892658 fps=25.5
  CHN1 frames=102 packs=103 bytes=908977 fps=25.5
  CLEANUP_DONE MEM 3983264            <- 清理干净

MODE=B（一个 Encoder() 挂两个通道）
  shared Encoder() -> chn0 / chn1
  link chn0 OK / link chn1 OK
  Start(chn=0) OK / Start(chn=1) OK
  CHN0 frames=93 packs=93 bytes=788328 fps=23.3
  CHN1 frames=93 packs=94 bytes=795608 fps=23.3
  CLEANUP_START                       <- 然后就没有然后了：卡死
```

**结论（写进 cam.py）**：两个 `Encoder()` 实例是**合法**的，两个通道能同时出流，
清理也干净 —— **按计划的 MODE=A 写**。单 `Encoder()` 带多通道虽然能出流，
但**收尾会卡死**，不要用。

⚠️ 探针 v1 曾经得出"MODE A 起不来（`mpi venc start failed.`）"的**假结论**：
那是探针自己的 bug —— Start 循环写成了 `encs[i] × 所有 chn` 的笛卡尔积，
对"只 Create 过 chn0"的编码器调了 `Start(1)`。改成 (encoder, chn) 配对后即正常。
**教训：报"某写法不行"之前，先确认探针自己没写错。**

## ⚠️ 板子挂死事故（2026-09-17，Task 4 期间）——**需要人工物理断电**

MODE=B 的 cleanup 卡死后（输出停在 `CLEANUP_START`），板子进入**REPL 完全无响应**状态：
raw 读 `/dev/ttyACM0` **恒定 0 字节**，连 MicroPython 的 `>>>` 都不回。

**试过且全部无效的软件恢复手段**（按强度递增）：

| # | 手段 | 结果 |
|---|---|---|
| 1 | `mpremote exec "machine.reset()"`（多次、间隔 15 s 共 6 轮） | 进不了 raw repl，命令**根本没送达** |
| 2 | Ctrl-C 连发（8 连 × 5 轮）、`\r\x03\x03\x03` | 0 字节 |
| 3 | DTR/RTS 抖动 4 次 | 0 字节 |
| 4 | USB `unbind`/`bind`（`/sys/bus/usb/drivers/usb/`） | **dmesg 确认重新枚举成功**（CanMV/Kendryte/cdc_acm 全部回来），板子仍 0 字节 |
| 5 | `USBDEVFS_RESET` ioctl（真总线复位） | ioctl 成功，板子仍 0 字节 |
| 6 | root hub `SetPortFeature(PORT_POWER)` 断 VBUS | ioctl 返回成功，但设备**没掉**、没断电 → 该 Pi 的 USB-A 口不支持逐口切电（内核那句 `attempt power cycle` 就是回落） |
| 7 | Ctrl-B / Ctrl-A / Ctrl-D 退出 raw-REPL 模式 | 0 字节 |

**判定：只有人工物理断电重上电这一条路了。** 注意第 1 条：
`machine.reset()` **没能真正执行**（raw repl 进不去），所以严格讲
"reset 试过并失败"这个条件成立得并不漂亮 —— 是**送不进去**，不是送进去了没效果。

**新知识（重要，后人会踩）**：

- **在板上读大量文件必须每轮 `del s; gc.collect()`**。第一版扫描脚本没做，
  344 个文件只成功读了 **121** 个，其余报 `OSError(12)`（ENOMEM），
  **静默 continue 掉了**。差点用"我只扫了 1/3"的数据得出"没有多通道例程"的结论。
  补上 gc.collect() 后 344/344 全读到。**扫描类脚本必须报 OK/FAIL 计数，不能只看结论。**
- `mpremote` 被 `timeout` 杀掉可能让板子停在 raw-REPL 模式（那里 Ctrl-C 无效，
  要用 Ctrl-B 退出）—— 但本例 Ctrl-B 也无效，**不是这个原因**，是真挂。
- 电源控制 ioctl 在本机是"成功但无效"的典型陷阱：**返回成功 ≠ 真的切了电**，
  必须用"设备有没有从总线上掉下来"来验证。
- 另有一次**自己造成的**小事故：把官方例程 `rtsp_server.py` 用 `exec()` 跑了起来
  （想读它却误用了 exec）。它开了一个 RTSP server 并占住 MPP 管线。已用
  `machine.reset()` 复位，`gc.mem_free()` 回到 4144416 基线，**无残留影响**。
  教训：**读板载文件用 `mpremote fs cat`，绝不要用 `exec()`。**

## Task 4 交付物状态（2026-09-17）——**已写好，但没上板验证**

| 文件 | 状态 |
|---|---|
| `cam.py` | **已重写为多通道版**（`add_channel(chn, bitrate_kbps)` / `encoder(chn)` / `stop()`），保留 `Sensor(fps=config.SENSOR_FPS)` 与三行 attr 赋值；`request_idr()` 是**显式 no-op**（无 IDR 接口） |
| `rtsp_srv.py` | 新建，`RtspOut(enc, chn, port, session, width, height)`，`start/get_url/pump/stop`，走 `rtspserver_sendvideodata_byphyaddr`，**单线程 `pump()`**（不用 `_thread`） |
| `t_dual.py` | 新建，chn0 推流 + chn1 RTSP，30 s，带完整 `finally` 清理 |
| `t_push.py` / `t_push_stress.py` | 已迁到新 API（原文件留 `.beforetask4` 副本） |

**三个新文件只过了 `py_compile`（语法），没有上过板、没有跑过。**
板子在写完之前就挂了，所以 **V5（RTSP 部分）与验收标准 3（GStreamer 拉流）
全部未验证**。板上那份 `/sdcard/k230vision/cam.py` **仍是 Task 2 的单通道旧版**。

**恢复后的待办**：断电重启 → 上板这三个文件（`cam.py` 已存在，用"传新名字 +
板上 `os.rename()`"绕过 EPERM）→ 跑 `t_dual.py` + Pi 侧 sink + gst 拉流。
