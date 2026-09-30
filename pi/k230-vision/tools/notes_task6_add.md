
---

## Task 6 结构化结果上报 + 视场附加验证（2026-09-17）

### ★★ 结论：chn0（720p 推流帧）与 chn2（AI 帧）**同一视场**，归一化坐标可直接搬

**这回答了设计文档「方案 A（单流原画 + 前端叠框）」的承重点，答案是成立的。**
Task 5 原先只有「chn2 在 320x320 与 320x180 下归一化 x 范围相同」这一条间接证据，
**从没把 chn0 和 chn2 放在一起比过**（它那张证据图底下的 720p 帧是花的/读错了）。
本次补上了这个对比，**同一管线会话里**把两路各 dump 一帧原样数据，搬到 Pi 上解码比对。

**证据 A —— 结构相关性（与颜色无关，最硬）**

把 chn2 的 320x320 **各向异性拉伸回 1280x720**，与 chn0 灰度求归一化相关，
并在 ±128 像素（1280x720 坐标系）内做位移搜索：

```
EVID_A_CORR_ZERO_SHIFT            0.9990     <- 零位移相关
EVID_A_BEST_SHIFT dx=0 dy=0 corr=0.9996     <- 最佳位移**恰好是 (0,0)**
EVID_A_CONTROL_WRONG_ZOOM_CORR    0.3651     <- 负对照：按"垂直放大 1.78 倍"错配
```

最佳位移**逐像素为 0** ⇒ 两路取景完全重合。
负对照 0.3651 说明这个指标**本身有区分力**（不是"什么都能得高分"）。

**证据 B —— 目标（苹果）归一化 bbox 对比**

用「最红连通域」在同一套算法下各测一遍：

| | l | t | r | b | 覆盖率 |
|---|---|---|---|---|---|
| chn0 (1280x720) | 0.3750 | 0.1222 | 0.7859 | 0.8444 | 0.2293 |
| chn2 (320x320)  | 0.3750 | 0.1219 | 0.7844 | 0.8438 | 0.2294 |
| **差** | **0.0000** | **-0.0003** | **-0.0016** | **-0.0007** | 0.0001 |

四个边界差 **全部 ≤ 0.0016（≤0.16% 画面）**。
另一条与颜色无关的版式参照物（最强竖直边缘，归一化 x）：
`chn0=0.1711  chn2=0.1688  delta=-0.0023`。

**⇒ chn2 是「把整幅 sensor 视野各向异性缩放到目标尺寸」，与 chn0 同视场。
归一化坐标乘前端显示尺寸即可直接叠框，方案 A 前提成立，无需改设计。**

证据图（Pi 侧）：`fov3_side_by_side.png`（两图并排 + 各自归一化框）、
`fov3_overlay.png`（把 chn2 的框按归一化坐标画到 chn0 上）、
`fov3_chn0.png`、`fov3_chn2.png`、`fov3_chn0_nv21.png`。
苹果仍在镜头前，所以用的是真实目标而不是静态参照物。

### ★ 新平台知识 1：`image.Image.bytearray()` —— 安全拿 720p 原始帧的第四条路

「绝不对 720p 帧做转换」是硬规则（`to_rgb888`/`to_grayscale`/`get_pixel` 搞死过板子 3 次）。
本次找到一条**没有把板子搞死**的路：

```python
im = sensor.snapshot(chn=CAM_CHN_ID_0)   # 1280x720 YUV420SP
ba = im.bytearray()                      # 返回 <class 'bytearray'>，长度 = 完整缓冲区大小
open('/sdcard/k230vision/raw_chn0.bin','wb').write(ba)
```

- 实测 `len(ba)` = **1382400** = 1280*720*1.5，**恰好等于无 stride 补齐的 YUV420SP 大小**
  （即 `alignment=12` 没有引入行填充，stride = 1280）。
- 小图对照：8x8 RGB888 → 192 B；16x16 YUV420 → 384 B。
- **不做任何格式转换、不逐像素读、不申请 2.7 MB 连续内存**。
- chn2 那路仍用 `to_numpy_ref()`（RGBP888，Task 5 已验安全）。
- ⚠️ `to_numpy_ref()` 对 YUV420SP 依旧抛 `ValueError: image format not support` —— 没变。
- ⚠️ `len(to_numpy_ref())` 对 RGBP888 返回 **3**（ulab 三维数组取 `shape[0]`），
  **不是字节数**；字节数要自己按 w*h*3 算。别拿 `len()` 当长度用。
- ⚠️ `im.format` 是**属性且是 int**（`470417443` / `201916931`），不是字符串、不是方法。

### ★ 新平台知识 2：**`time.time()` 只有 1 秒分辨率**，定速必须用 `ticks_ms`

`t_report.py` v1 用 `time.time()` 做 10 Hz 节拍器，**实测跑成 24.55 Hz**
（`STATS frames=491 ... report_hz=24.55`，即推理满速）—— 节拍器整个退化成空操作。

根因：**本 build 的 `time.time()` 是整数秒**：
```
TIME_RES 1789646288 1789646288 1789646288     # 连续三次调用同一个整数
TICKS    103318      103318      103318        # ticks_ms 也一样是"当拍"
```
（`ticks_ms` 那三个一样只是因为在同一毫秒内连打。）

**sleep 本身是准的**，别误判：用 `ticks_ms` 量出来
`sleep_ms(60/100/500)` → 60/100/500 ms，`sleep(0.06/0.1/0.5)` → 60/100/500 ms。
**坑在时钟源，不在 sleep。**

⚠️ **陷阱警告**：如果用 `time.time()` 的差值去量 sleep（我第一版就是这么量的），
会得到 `sleep(0.06)→0.000s`、`sleep(0.1)→1.000s`、`sleep(0.5)→0.000s` 这种
**自相矛盾的结果**（0.1 睡了 1 秒、0.5 却睡了 0 秒），看着像 sleep 坏了。
那是**测量被 1 秒量化**造成的假象。**量小睡必须用 `ticks_ms`/`ticks_diff`。**

改用 `ticks_ms` 后的验证：3 s 目标 10 Hz → **n=30 / el=3000 ms = 10.0 Hz** 精确命中。

⚠️ 连带的：`ts` 字段 `int(time.time()*1000)` 实际**只有秒级精度**（尾数恒为 000）。
设计要的是 epoch_ms，本 build 拿不到真正的毫秒 epoch。**如实记录，没有伪造精度。**

### ★ 新平台知识 3：`mpremote fs cp` 传大文件极慢 —— 大文件走 WiFi

- `mpremote fs cp` 拉 1.38 MB：**90 s 都没传完**（被 `timeout` 砍掉，文件 0 字节）。
- 同样 1.38 MB 走板子 WiFi：**2.00 s = 691200 B/s（5.5 Mbps）**，字节数与板上逐位相等。
- 做法：板载 `t_sendfile2.py`（裸 TCP 推）+ Pi 侧 `tools/recv_file.py`（落盘）。
- ⚠️ **`t_sendfile.py`（v1）的坑**：`netup.connect()` 返回后**立刻**发包，
  只发出 **42340 B** 就 `OSError: [Errno 11] EAGAIN`。
  这正是 NOTES 开头那条「连上后十几秒内速率自适应没爬升，吞吐低 2~3 个数量级」。
  v2 在 connect 后**静置 18 s** 再发，send 超时放宽到 20 s —— 一次通过。
  **v1 留档不删**，它记的就是这个坑。

### ★ 新平台知识 4：dump 帧做内容比对前**必须先预热**（自动曝光）

v1 的 `t_fov3.py` 在 `sensor.run()` 之后**立刻**dump chn0，拿到的是 AE 未收敛的第一帧：
Pi 侧解出 `Y mean=22.7 / max=46`（整帧近乎全黑），而同一时刻的 chn2 帧 luma `mean=104`。

**后果（差点得出错误结论）**：两帧亮度差一个数量级，
「偏红区域」的 bbox 因此不可比 —— v1 测出的 bbox 差高达
`[-0.0500, -0.0420, +0.0320, +0.0910]`，看着像"两路取景不一致"。
**那全是曝光差造成的，不是几何差。**
v2 对 chn0/chn2 **各自预热 30/15 帧**后再 dump，`Y_SPARSE_MEAN` 升到 **108.5**，
bbox 差立刻收敛到 `≤0.0016`。
⇒ **任何"两帧内容比对"的实验，都必须先确认两帧曝光已收敛**，否则会测出假差异。

### Task 6 验收实测（原始回显见对话）

**Run 1（有 sink，t_report.py）**

```
K230: WIFI OK 192.168.1.112 / REPORTER OK 192.168.1.108:8556
      CLOUD disabled (CLOUD_RESULT_HOST empty) - local path unaffected
      CHN2 OK 320x320 RGBP888 / PIPELINE OK kmodel=/sdcard/examples/kmodel/yolov8n_320.kmodel
      STATS frames=200 obj_frames=200 el=20.00 report_hz=10.00
      SAMPLE_OBJS [{'cls': 'apple', 'score': 0.503, 'box': [0.3625, 0.1156, 0.7812, 0.8156]}]
      DONE sent=200 dropped=0
      CLEANUP_DONE MEM 3920576

Pi  : RESULT 200 messages in 19.9s = 10.0 msg/s (span 19.9s)
      BAD_LINES 0
      MAX_FRAME_GAP 1                      <- 帧号无跳号 ⇒ 两侧对账一致，确实一条没丢
      FIRST_OBJ_MSG [1] {"objs": [{"cls": "apple", "score": 0.503,
                        "box": [0.3625, 0.1219, 0.7812, 0.8344]}],
                        "ts": 1789646346000, "w": 1280, "frame": 1, "h": 720}
      LAST_OBJ_MSG  [200] {...同形，score 0.503, box [0.3625, 0.1219, 0.7812, 0.8344]}
```

- **200/200 帧全部有真检出**，`obj_frames=200`，类别 `apple`（COCO index 47），
  box ≈ `[0.3625, 0.1219, 0.7812, 0.8344]` —— 与计划验收里写的
  ≈`[0.36,0.12,0.78,0.83]` 吻合，**且与上面 chn0 独立算出的偏红区域 bbox 吻合**。
- `w/h = 1280/720` 是**推流**分辨率（不是 AI 的 320x320），前端直接乘显示尺寸即可。
- `CLOUD_RESULT_HOST` 为空串时**只打印一行 disabled，本地路径完全不受影响** ⇒ 验收第 4 条成立。

**Run 2（无 sink，核心契约："发不出去就丢，绝不排队"）**

```
K230: connect to server faild! / FAIL 连不上 192.168.1.108:8556
      CLEANUP_DONE MEM 3926304
```
起手就连不上 ⇒ 直接 return，**连管线都没建**，干净退出。

**Run 3（sink 收到第 80 条时挂断并退出）** —— 这才是"发不出去"的真场景

```
Pi  : HANGUP after 80 msgs (t=10.3s)
K230: DONE sent=81 dropped=112        # 81+112 = 193 = frames，账目闭合
      STATS frames=193 obj_frames=193 el=20.04 report_hz=9.63
      CLEANUP_DONE MEM 3920576
```

- **丢了 112 条、一条没排队、板子没崩、推理一路跑到底**（193 帧全有检出）⇒ 契约成立。
- `sent=81` 比 80 多 1：挂断瞬间那条已经在途（RST 还没回到）。
- ⚠️ **`errno 12 (ENOMEM)` 没有复现** —— Task 4 的 `t_dual.py` 出现过，那是它
  `if p.pump()==-2: p.connect()` **裸循环零延迟**的重连风暴。本处 10 Hz 只有 10 次/s 的
  connect 尝试，**安然无事**。⇒ 再次印证那条 ENOMEM 是**测试脚本自己造成的**，
  生产 `app.py` 按 `RECONNECT_BACKOFF_MS` 节流即可。

### 一处待观察：视觉管线跑完后堆会低 ~220 KB

三次带 Detector + sensor 的运行，收尾 `CLEANUP_DONE MEM` 分别是
`3921376 / 3920576 / 3920576`，而复位后基线是 **4144416**，差 **~224 KB**。
- 只跑 sensor（无 KPU）的 `t_fov3.py`：`3998048`（差 ~146 KB）。
- **不是崩溃性的**，`app.py` 一进程一条管线不受影响；但若以后要在一个进程里
  反复起停，**别指望堆能完全回到基线**。机制未查明，**不下断言，只记录**。

### 交付物清单（板载 == Pi，md5 逐位相符）

| 文件 | 位置 | 字节 | 行数 | md5 |
|---|---|---|---|---|
| `reporter.py` | 板 `/sdcard/k230vision/` + Pi `k230/` | 2485 | 76 | `e6ffcfaf84efde52b1930671d13b093e` |
| `t_report.py` | 同上 | 6625 | 177 | `c89f46d2c250a7d5fa0db1736ae88a12` |
| `t_fov3.py` | 同上 | 4672 | 134 | `b60bdc864429c33d8b54e4bfe8249855` |
| `config.py` | 同上（加了 CLOUD_RESULT_HOST/PORT） | 3804 | 62 | `80cc2f0c79f4567476ad10f06088c0b9` |
| `json_sink.py` | Pi `/home/cy/k230-vision/tools/` | 2986 | 100 | `6fd3c31e1f8386da8578537a68b23421` |

探针（非交付接口，留着记录坑）：`t_sendfile.py`（v1，EAGAIN）、`t_sendfile2.py`（v2）、
`tools/recv_file.py`、`tools/json_sink_hangup.py`、`tools/fov_analyze.py`。

### Task 6 与计划的偏离

1. **`t_report.py` 发真检出**（任务明确要求），不是计划草稿里写死的 `{"cls":"person",...}`。
2. **`config.py` 加的常量名用 `CLOUD_RESULT_HOST`**（计划 Step 4 也是这个），
   计划 Step 3 代码里写的 `config.RESULT_HOST` 是对的、未改名。
3. **`json_sink.py` 比计划版多了**：`BAD_LINES` / `MAX_FRAME_GAP` /
   `FIRST_OBJ_MSG` / `LAST_OBJ_MSG`，用来把"真检出"和"帧号无跳号"变成可验收的输出。
4. **`reporter.py` 的 `send()` 增加了"短写"处理**（计划版只判 `OSError`）。
   换行分隔的消息一旦短写，行边界就不可信、之后每条都错位 ——
   按"发不出去"处理：计 dropped、**关连接**、**不重发**（与 H.264「绝不重发半帧」同源）。
5. **`t_report.py` 不建 VENC**：它验的是上报链路；chn0/chn1 VENC 共存已由 Task 4 验过。
6. **`t_fov3.py` 不建 VENC**：取景几何由 `sensor.set_framesize` 决定，与有没有挂编码器无关。
