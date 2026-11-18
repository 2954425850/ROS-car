# /sdcard/k230vision/configrx.py
"""远程配置入口：让 Pi 改板子的 WiFi 与各接收端地址（端口 8558）。

## 为什么单独开一个端口

8557 是**跟踪目标**的命令通道（点/框/follow），语义是"让相机跟什么"；
配置是**设备管理**，改的是板子本身的网络参数，和视觉任务无关。
混在一个端口上，以后任何一方改协议都会牵连另一方 —— 所以分开：
8557 目标 / 8558 配置 / 8556 结果 / 8559 避障 / 8554 RTSP / 8555 推流。

## 协议

和 8557 完全同形：**一次连接一行 JSON**（连接 → 发一行 → 收一行 → 关）。

    {"cmd": "config_get"}
    {"cmd": "config_set", "result_host": "10.64.35.7", "push_host": "10.64.35.7"}
    {"cmd": "config_set", "wifi_ssid": "MyWiFi", "wifi_pass": "secret123"}
    {"cmd": "config_set", "obstacle_host": null}     # null = 跟随 result_host
    {"cmd": "config_clear"}                          # 删掉远程配置，重启后回到 config_local

可设的键：`wifi_ssid` / `wifi_pass` / `push_host` / `result_host` / `obstacle_host`。

应答（成功）：

    {"ok": true, "accepted": ["result_host"], "persisted": ["result_host"],
     "deferred": [], "note": "已落盘（config_remote.json），重启后仍生效"}

## ⚠️ 改 WiFi 的应答是**动手之前**回的

真正的换网在主循环里做（`netup.reconnect`），可能要几十秒。
所以 **ok 只代表"收到了、校验通过"**，不代表已经连上新网。
换网失败板子会自动回滚到旧网络；结果看 `config_get` 里的 `last_apply`：

    {"ok": true, "config": {...}, "remote_keys": [...],
     "last_apply": {"t": 1789690497, "ok": false,
                    "detail": "new wifi failed -> reverted to MyWiFi"}}

**WiFi 凭据要等连上新网之后才落盘**（`config_remote.json`）——
这正是为了防"写进磁盘的是一个连不上的网、然后板子重启失联"。
地址类没有这个问题，收到就落盘。

换网成功后板子在新网络里的 IP 可能变；Pi 侧用 `k230ctl config` 时靠
`board_host()`（resultd 记下的对端地址）重新发现，别写死。

## 边界

- **板子已经离线时，这个端口救不了它** —— 连不上任何网就没有入口。
  远程换 WiFi 只适用于"板子现在在线、要换到另一个能用的网"。
  彻底连不上时还是拔卡改 config_local.py。
- **没有鉴权**：和 8557 一样，局域网内的设备都能发。信任边界是局域网本身。
- `wifi_pass` 在 `config_get` 里只回 `wifi_pass_set: true/false`，**不回明文**。

## 与主循环的分工

`poll()` 只做：收 → 校验 → **地址类落盘** → 回 ok → 把改动排队。
`take()` 由主循环取走，在**没有帧要处理的地方**应用（换 WiFi 会阻塞几十秒，
不能放在 10 Hz 推理那段里，见 app.py 主循环的节流约定）；WiFi 凭据也在
**连上新网之后**由主循环落盘 —— 所以 `poll()` 不写 WiFi。
"""

import socket

import ujson

import config
import netup
import remote_cfg


class ConfigRx:
    """监听配置端口，校验并落盘，把运行时应用留给主循环。

    ⚠️ 必须在 `netup` 之后构造（bind 需要活网卡）。
    """

    def __init__(self, port):
        self.port = port
        self.pending = None          # 待主循环应用的 changes（地址已落盘，WiFi 未落盘）
        self.last_apply = None       # 最近一次应用结果，config_get 里回给 Pi
        self.recv_ok = 0
        self.recv_bad = 0

        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind(("0.0.0.0", port))
        self._srv.listen(1)
        self._srv.settimeout(0)          # 非阻塞 accept

    # ---------------- 应答内容 ----------------

    def _snapshot(self):
        remote = remote_cfg.load()
        return {
            "ok": True,
            "ip": netup.ip(),
            "config": {
                "wifi_ssid": config.WIFI_SSID,
                "wifi_pass_set": bool(config.WIFI_PASS),
                "push_host": config.PUSH_HOST,
                "result_host": config.RESULT_HOST,
                "obstacle_host": config.OBSTACLE_HOST,
                # 实际用的（None = 跟随 result_host）
                "obstacle_host_effective": config.OBSTACLE_HOST or config.RESULT_HOST,
            },
            "remote_keys": sorted(remote.keys()),
            "last_apply": self.last_apply,
        }

    # ---------------- 收 ----------------

    def rebind(self):
        """WiFi 断开/重连后重建监听 socket。

        2026-11-18 评审：`netup.reconnect` 会把网卡 down/up，板子只重建了
        pusher/result/obstacle 的出站 socket，**没有重建 8558 的监听 socket**。
        旧监听 socket 在 lwIP 上可能随 netif 一起失效 → 板子已经在新网、
        但 8558 永远 accept 不到，Pi 的扫描/改指向就全断了。
        换网（无论成功还是回滚）后必须调一次。
        """
        old = self._srv
        self._srv = None
        try:
            old.close()
        except BaseException:
            pass
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", self.port))
            srv.listen(1)
            srv.settimeout(0)
        except BaseException:
            try:
                srv.close()
            except BaseException:
                pass
            raise
        self._srv = srv

    def poll(self):
        """非阻塞。收一条命令、校验、落盘、应答，然后把运行时应用排队。"""
        if self._srv is None:
            return                       # rebind 失败：配置入口暂时不可用，别崩主循环
        try:
            conn, _addr = self._srv.accept()
        except OSError:
            return                       # 没有连接 —— 正常路径
        try:
            conn.settimeout(0.2)
            data = b""
            while b"\n" not in data and len(data) < 1024:
                try:
                    chunk = conn.recv(256)
                except OSError:
                    break
                if not chunk:
                    break
                data += chunk
            line = data.split(b"\n")[0].strip()
            if not line:
                self._reply(conn, {"ok": False, "err": "empty"})
                self.recv_bad += 1
                return
            req = ujson.loads(line)
            if not isinstance(req, dict):
                self._reply(conn, {"ok": False, "err": "not an object"})
                self.recv_bad += 1
                return

            cmd = req.get("cmd")
            if cmd == "config_get":
                self._reply(conn, self._snapshot())
                self.recv_ok += 1
                return

            if cmd == "config_set":
                changes, err = remote_cfg.validate(req)
                if err:
                    self._reply(conn, {"ok": False, "err": err})
                    self.recv_bad += 1
                    return
                # 地址类没有失败风险：现在落盘，写失败要让 Pi 知道
                # （板上 SD 的写入是这里最容易坏的一步）。
                host_changes = {}
                wifi_keys = []
                for k in changes:
                    if k in ("wifi_ssid", "wifi_pass"):
                        wifi_keys.append(k)
                    else:
                        host_changes[k] = changes[k]
                if host_changes:
                    try:
                        remote_cfg.save(host_changes)
                    except BaseException as e:
                        self._reply(conn, {"ok": False, "err": "save failed: %s" % e})
                        self.recv_bad += 1
                        return
                self.pending = changes
                if wifi_keys:
                    # ⚠️ **不现在落盘**：先换网，连上了才写。原因见文件头 ——
                    #    写进去的是一个连不上的网，板子下次上电就失联，只能拔卡。
                    note = ("WiFi 将在本连接关闭后切换（失败自动回滚到旧网络、"
                            "不写入磁盘）；连上后才落盘，结果看 config_get 的 last_apply")
                    if host_changes:
                        note += "；地址部分已落盘"
                else:
                    note = "已落盘（config_remote.json），重启后仍生效"
                self._reply(conn, {"ok": True, "accepted": sorted(changes.keys()),
                                   "persisted": sorted(host_changes.keys()),
                                   "deferred": sorted(wifi_keys),
                                   "note": note})
                self.recv_ok += 1
                return

            if cmd == "config_clear":
                gone = remote_cfg.clear()
                # 运行时值**不在这里改**：WiFi 要重新连接、地址要重指，
                # 都只能在主循环里做。而且清掉的往往就是想回滚的东西 ——
                # 让 Pi 明确知道"重启后生效"，比假装立刻撤销安全。
                self._reply(conn, {"ok": True, "cleared": gone,
                                   "note": "已删除远程配置；运行时值不变，重启后回到 "
                                           "config_local.py"})
                self.recv_ok += 1
                return

            self._reply(conn, {"ok": False,
                               "err": "need cmd=config_get|config_set|config_clear"})
            self.recv_bad += 1
        except BaseException as e:
            self.recv_bad += 1
            try:
                self._reply(conn, {"ok": False, "err": "%s" % e})
            except BaseException:
                pass
        finally:
            try:
                conn.close()
            except BaseException:
                pass

    def _reply(self, conn, obj):
        conn.send(ujson.dumps(obj).encode() + b"\n")

    # ---------------- 交给主循环 ----------------

    def take(self):
        """返回 ("set", changes) 或 (None, None)。

        ⚠️ **不要在这里应用**：换 WiFi 要断开重连（最多几十秒），
        而且连上之后才轮到落盘 —— 那都是主循环的事。
        """
        if self.pending is None:
            return (None, None)
        p = self.pending
        self.pending = None
        return ("set", p)

    def close(self):
        try:
            self._srv.close()
        except BaseException:
            pass
