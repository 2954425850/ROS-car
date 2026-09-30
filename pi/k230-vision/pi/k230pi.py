#!/usr/bin/env python3
"""K230 视觉链路的 Pi 侧。

## 只做两件事

1. `snap()`    —— 按需从 K230 拉一路 RTSP，抓一张 JPEG 落盘。
2. `resultd()` —— 常驻监听 8556，收 K230 推来的换行 JSON（结构化视觉结果）。

## 不做什么

不含 `/dev/video0`（v4l2loopback 用户 2026-09-17 决定延期），不含任何媒体转发，
不碰 `voice-chatbot` / RPLIDAR / L150Pro。
"""
import json
import os
import signal
import socket
import subprocess
import time

K230_HOST = "192.168.1.112"
RTSP_PORT = 8554
RTSP_SESSION = "k230"
RTSP_URL = "rtsp://%s:%d/%s" % (K230_HOST, RTSP_PORT, RTSP_SESSION)

RESULT_PORT = 8556          # 与 K230 侧 config.py 的 RESULT_PORT 一致
TARGET_PORT = 8557          # 与 K230 侧 config.py 的 TARGET_PORT 一致
# ⚠️ 要盖过板子那边的 FOLLOW_TIMEOUT_MS（8s）：`cmd=follow` 是**锁上之后**才回 ok 的，
# 中间那 8 秒都在等正脸。超时给短了会把"还没锁上"误判成"板子没响应"。
TARGET_TIMEOUT = 10.0

# Task 1 实测选定（2026-09-18）：720p30 满载软解吃 ~17~20% 单核，够用。
# `openh264dec` 也能用；`v4l2h264dec`/`v4l2slh264dec`/`vaapih264dec` 在 Ubuntu 24.04
# 的包里都没有 —— `/dev/video19`(rpivid) 那个硬解没有 GStreamer 元素能驱动，故走软解。
DECODER = "avdec_h264"

OUT_DIR = "/tmp/k230"
LATEST_JPEG = os.path.join(OUT_DIR, "latest.jpg")
LATEST_RESULT = os.path.join(OUT_DIR, "latest-result.json")
STATUS_FILE = os.path.join(OUT_DIR, "resultd-status.json")

JPEG_QUALITY = 90
JPEG_SOI = b"\xff\xd8\xff"      # JPEG 起始
JPEG_EOI = b"\xff\xd9"          # JPEG 结束

# ---- snap 的三个关键常量（改动前先读 NOTES.md 的「Task 1 实测修正」） ----
#
# GOP_LEN = 25：**RTSP 流开头是 P 帧，不是 IDR**（实测首帧序列 `AUD SPS PPS P ...`）。
#   P 帧没有参考帧，直接取「第一帧」一定是坏的 —— 所以只能取「最后一帧」。
#   首个 IDR 最迟在第 GOP_LEN 帧出现，因此**攒够 GOP_LEN+1 帧之后，末帧必然干净**。
GOP_LEN = 25
#
# MIN_FRAMES = 30：留余量。**必须按帧数等，不能按秒数等。**
#   同一个 GOP 要多久取决于板子的帧率，而帧率取决于板子的推流有没有接收端：
#     有接收端（云端在收） → ~28~30 fps → 30 帧约 1.0 s
#     没有接收端（云端未部署）→ ~8.4 fps  → 30 帧约 3.6 s
#   固定秒数在后者会跨越不到 IDR 而抓到坏帧。2026-09-18 实测，见 NOTES.md。
MIN_FRAMES = 30
DEFAULT_TIMEOUT = 12.0          # 单次尝试的墙钟硬上限（s）
DEFAULT_ATTEMPTS = 3            # 见 snap() 的说明：RTSP 起手偶发快速失败，重试很划算


class SnapError(RuntimeError):
    pass


class TargetError(RuntimeError):
    """给 K230 发目标命令失败（连不上 / 没应答 / 应答不是 JSON）。"""


def ensure_out_dir():
    os.makedirs(OUT_DIR, exist_ok=True)
    return OUT_DIR


def build_snap_cmd(tmp_path, proto="tcp", latency=200):
    """Task 1 验证过的同一组参数。

    `h264parse` 是**必需的**：`rtph264depay` 默认吐 AVCC（长度前缀），
    不是 Annex-B（实测文件里 `00 00 00 01` 出现 0 次）。见 NOTES.md。
    `-e` 让 SIGINT 时发 EOS，把 sink 刷干净。
    """
    return [
        "gst-launch-1.0", "-e", "-q",
        "rtspsrc", "location=%s" % RTSP_URL, "protocols=%s" % proto,
        "latency=%d" % latency, "drop-on-latency=true",
        "!", "rtph264depay",
        "!", "h264parse",
        "!", "video/x-h264,stream-format=byte-stream,alignment=au",
        "!", DECODER,
        "!", "videoconvert",
        "!", "video/x-raw,format=I420",
        "!", "jpegenc", "quality=%d" % JPEG_QUALITY,
        "!", "filesink", "location=%s" % tmp_path,
    ]


def last_jpeg(blob):
    """从「多帧 JPEG 首尾相接」的 blob 里切出最后一个完整 JPEG。

    从后往前找 EOI，再往前找对应的 SOI。写了一半的尾帧会被自然跳过 ——
    这正是我们要的：进程即使中途死了，也还能拿到死之前那一张完整的。
    """
    eoi = blob.rfind(JPEG_EOI)
    if eoi < 0:
        return None
    soi = blob.rfind(JPEG_SOI, 0, eoi)
    if soi < 0:
        return None
    return blob[soi:eoi + len(JPEG_EOI)]


def _count_frames_so_far(path, state):
    """增量统计临时文件里已有的 JPEG 帧数（只读新增的字节）。

    state = [已读到的偏移, 累计帧数]。非重叠读取，所以**跨读取边界的那个 SOI 会被漏数**
    （3 字节的 marker 正好被切开，概率极低）—— 漏数只会让等待多一个轮询周期（50 ms），
    方向是安全的；重复计数才会让我们过早收工，所以这里宁可漏不可重。
    """
    try:
        size = os.path.getsize(path)
    except OSError:
        return state[1]
    if size <= state[0]:
        return state[1]
    with open(path, "rb") as f:
        f.seek(state[0])
        chunk = f.read()
    state[0] = state[0] + len(chunk)
    state[1] = state[1] + chunk.count(JPEG_SOI)
    return state[1]


def snap(out=None, min_frames=MIN_FRAMES, timeout=DEFAULT_TIMEOUT, proto="tcp",
         attempts=DEFAULT_ATTEMPTS):
    """抓一张 JPEG。成功返回 dict；全部尝试都失败才抛 SnapError。

    ## 为什么要重试

    实测 K230 的 RTSP 拉流会**偶发在起手几秒内直接死掉**（中间产物 0 字节），gst 报：
        gst_rtspsrc_loop_interleaved(): Could not receive message. (Parse error)
    10 次里出现 1 次。**这种失败是快速失败**（不浪费时间），所以直接重试最划算。
    重试之间停 0.5 s，给板子的 RTSP 服务端一点时间收拾上一个会话。

    返回： {"path", "frames", "suspect", "seconds", "attempts_used"}
      suspect=True 表示攒到的帧数没到 min_frames（可能没跨过 IDR，画面可能不可靠）。
      **这时仍然返回文件**（总比什么都没有强），但调用方应该把它当「可能不可靠」对待。
    """
    last = None
    for i in range(max(1, attempts)):
        try:
            r = _snap_once(out=out, min_frames=min_frames, timeout=timeout,
                           proto=proto)
            r["attempts_used"] = i + 1
            return r
        except SnapError as e:
            last = e
            if i + 1 < attempts:
                time.sleep(0.5)
    raise SnapError("重试 %d 次都失败；最后一次：%s" % (max(1, attempts), last))


def _snap_once(out, min_frames, timeout, proto):
    ensure_out_dir()
    tmp = os.path.join(OUT_DIR, "_snap_tmp.jpg")
    if os.path.exists(tmp):
        os.unlink(tmp)

    cmd = build_snap_cmd(tmp, proto=proto)
    t0 = time.time()
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.PIPE)
    state = [0, 0]
    try:
        while True:
            if time.time() - t0 >= timeout:
                break
            if _count_frames_so_far(tmp, state) >= min_frames:
                break
            time.sleep(0.05)
    finally:
        _stop(proc)
    elapsed = time.time() - t0

    with open(tmp, "rb") as f:
        blob = f.read()
    jpg = last_jpeg(blob)
    if jpg is None:
        err = ""
        try:
            err = proc.stderr.read().decode("utf-8", "replace").strip()
        except Exception:
            pass
        raise SnapError("没抓到可用的 JPEG（中间产物 %d 字节）%s"
                        % (len(blob),
                           "；gst: " + err[-400:] if err else ""))

    if out is None:
        out = os.path.join(OUT_DIR,
                           "snap-%s.jpg" % time.strftime("%Y%m%d-%H%M%S"))
    _atomic_write(out, jpg)
    _atomic_write(LATEST_JPEG, jpg)
    return {"path": out, "frames": state[1],
            "suspect": state[1] < min_frames,
            "seconds": round(elapsed, 2)}


def _stop(proc):
    """先 SIGINT（配合 -e 让 sink 刷完），再兜底 kill -9。"""
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=3)
    except Exception:
        try:
            proc.kill()
            proc.wait(timeout=2)
        except Exception:
            pass


def _atomic_write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ==================== Task 3: resultd（常驻收结构化结果） ====================

def _atomic_write_fast(path, data):
    """同 `_atomic_write`，但**不做 fsync**。

    结果流是 ~10 Hz 的「最新值」缓存。fsync 会把 SD 卡写爆（10 次/秒 × 86400 = 每天 86 万次），
    而这里丢了最多丢「最后一个值」—— 100 ms 后下一条就补上了，不值得付 fsync 的代价。
    照片（`snap`）走的是带 fsync 的 `_atomic_write`，因为那张图必须真的落盘。
    """
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def _flush_status(st, force=False):
    """把守护进程状态写进 STATUS_FILE，给 `k230ctl status` 读。

    **不在这里假装知道「多久没收到结果」** —— 想要那个数就由读的一方拿
    STATUS_FILE 的 mtime 去算（见 `status_age_s()`）。写一个用 time.time() 差值
    糊出来的数字只会撒谎。
    """
    _atomic_write_fast(STATUS_FILE,
                       json.dumps(st, ensure_ascii=False).encode("utf-8"))


def status_age_s():
    """STATUS_FILE 多久没更新了（秒）。文件不存在返回 None。"""
    try:
        return round(time.time() - os.path.getmtime(STATUS_FILE), 1)
    except OSError:
        return None


def resultd():
    """常驻监听 RESULT_PORT，收 K230 的换行 JSON。前台运行，给 systemd 用。

    ## 铁律

    - **单客户端**：K230 只会有一条连接。顺序 accept 就够，不要多线程。
    - **断连是常态不是异常**：K230 的 reporter 连不上就重试，所以收到 EOF 就
      `close()` 然后回去 accept，**绝不退出、不打印成错误**。
    - **一行一条**：一次 recv 可能到多行、也可能是半行，必须按 `\\n` 切。
    - **坏行跳过并计数**，不许因为一行坏数据把服务搞挂。
    - **不排队、不存历史**：只保留「最近一条」。
    """
    ensure_out_dir()
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", RESULT_PORT))
    srv.listen(2)
    # K230 的 reporter 是 ~10 Hz；静默超过这么久 = 对端已经不在了。
    IDLE_LIMIT = 12.0

    st = {"pid": os.getpid(),
          "started": time.strftime("%Y-%m-%d %H:%M:%S"),
          "listening": RESULT_PORT,
          "connects": 0, "msgs": 0, "bad_lines": 0,
          "connected": False, "peer": None,
          "last_frame": None, "last_objs": None, "last_msg": None}
    _flush_status(st)
    print("k230-resultd listening on %d (out=%s)" % (RESULT_PORT, OUT_DIR),
          flush=True)

    while True:
        try:
            conn, peer = srv.accept()
        except OSError as e:
            print("accept error: %r" % (e,), flush=True)
            time.sleep(1)
            continue
        st["connects"] += 1
        st["connected"] = True
        st["peer"] = "%s:%d" % peer
        print("connect #%d from %s" % (st["connects"], st["peer"]), flush=True)
        conn.settimeout(5.0)
        try:
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except OSError:
            pass
        buf = b""
        t_status = time.time()
        t_idle = time.time()
        while True:
            try:
                d = conn.recv(65536)
            except socket.timeout:
                _flush_status(st)          # 顺带让 status 的心跳保持新鲜
                # 关键：**K230 的 machine.reset() 不会关闭它的 TCP 连接**
                # （网络栈在 rt-smart 侧，软复位不动它，也就没有 FIN/RST）。
                # 对端死掉时我们收不到任何通知，只能靠静默超时放手；
                # 否则新连接永远 accept 不到（堆在 backlog 里），表现为
                # 「板子说连上了、Pi 却再也收不到结果」而两边都不报错。
                if time.time() - t_idle > IDLE_LIMIT:
                    print("idle %.0fs - dropping stale conn"
                          % (time.time() - t_idle), flush=True)
                    break
                continue
            except OSError as e:
                print("recv error: %r" % (e,), flush=True)
                break
            if not d:
                break                      # 对端关闭 —— 正常事件，回去 accept
            buf += d
            t_idle = time.time()
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line.decode("utf-8", "replace"))
                except Exception:
                    st["bad_lines"] += 1
                    continue
                st["msgs"] += 1
                st["last_frame"] = obj.get("frame")
                st["last_objs"] = len(obj.get("objs") or [])
                st["last_msg"] = time.strftime("%H:%M:%S")
                obj["_received_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
                obj["_peer"] = st["peer"]
                _atomic_write_fast(
                    LATEST_RESULT,
                    json.dumps(obj, ensure_ascii=False).encode("utf-8"))
                if time.time() - t_status >= 1.0:
                    _flush_status(st)
                    t_status = time.time()
        conn.close()
        st["connected"] = False
        _flush_status(st)
        print("disconnect from %s (msgs=%d)" % (st["peer"], st["msgs"]),
              flush=True)


def read_latest_result():
    try:
        with open(LATEST_RESULT, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def read_status():
    try:
        with open(STATUS_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def send_target(payload, host=K230_HOST, port=TARGET_PORT, timeout=TARGET_TIMEOUT):
    """给 K230 发一条目标命令（**一次连接一条**），返回它的应答 dict。

    板子那边的约定：连接 → 发一行 JSON → 收一行 JSON → 关。没有长连接。

    ⚠️ `{"cmd":"follow","who":...}` 是**锁上之后才回 ok** 的 —— 中间那几秒
    板子在等人脸，所以 timeout 默认给到 10s（板子的 FOLLOW_TIMEOUT_MS 是 8s）。
    """
    import socket
    line = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
    try:
        sock = socket.create_connection((host, port), timeout=timeout)
    except OSError as e:
        raise TargetError("连不上 K230 %s:%d —— %s" % (host, port, e))
    buf = b""
    try:
        sock.sendall(line)
        sock.settimeout(timeout)
        while b"\n" not in buf:
            chunk = sock.recv(256)
            if not chunk:
                break
            buf += chunk
    except OSError as e:
        raise TargetError("发完命令没收到应答 —— %s" % e)
    finally:
        try:
            sock.close()
        except OSError:
            pass
    if not buf.strip():
        raise TargetError("K230 没回应答（连接被关？）")
    try:
        return json.loads(buf.split(b"\n")[0].decode("utf-8"))
    except (ValueError, UnicodeDecodeError) as e:
        raise TargetError("应答不是 JSON: %r" % buf[:80])


PEOPLE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "people.json")


def load_people():
    """返回 {id: 显示名}。文件不存在/坏了就当空表，**不抛**。

    为什么显示名在这边：板子上文件名只能是 ASCII（id1/id2…，FAT32 上中文名风险大），
    所以"张三"这种名字只能存在消费这一侧。
    """
    try:
        with open(PEOPLE_FILE, encoding="utf-8") as fh:
            data = json.load(fh)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def resolve_who(name):
    """把**用户说的名字**解析成 faces 里的 id。

    认不出名字就**原样返回** —— 这样直接说 "id1" 也能用，而且新增的人
    不用先改表就能跟。_send 那边会拿板子的 err 兜底。
    """
    name = (name or "").strip()
    for pid, disp in load_people().items():
        if disp == name:
            return pid
    return name
