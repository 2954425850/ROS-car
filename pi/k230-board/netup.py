# /sdcard/k230vision/netup.py
"""WiFi 连接与重连。

## 为什么必须"等到非 0.0.0.0"

connect() 返回不等于拿到 IP。且在刚连上的十几秒内，速率自适应还没爬升，
此时测吞吐会得到低 2~3 个数量级的假数字（2026-09-17 实测踩过，见 NOTES.md）。
所以这里既等 IP，也给调用方一个"链路尚未预热"的信号。

## 2026-09-17 实测（CanMV v1.8-0 / MicroPython 3.4.0，庐山派 k230_canmv_lckfb）

- `network.STA_IF` == 0，可用。
- `WLAN(STA_IF)` 的 `active/isconnected/ifconfig/connect/disconnect` 均存在。
- **不要用 `status()` / `config()` 判断连接状态**：本固件上
  `w.status()` 返回 True（bool，不是状态码），`w.config('ssid')`、
  `w.config('rssi')` 也返回 True 而不是值。只有 `ifconfig()[0]` 给出的
  才是真 IP，故本模块只用 `isconnected()` + `ifconfig()`。
- `ifconfig()` 返回 4 元组 `(ip, netmask, gateway, dns)`，
  实测 DNS 位是 `111.11.11.1`。
"""
import network
import time

_SSID = None
_PASS = None
_wlan = None


def init(ssid, password):
    global _SSID, _PASS, _wlan
    _SSID, _PASS = ssid, password
    _wlan = network.WLAN(network.STA_IF)
    _wlan.active(True)
    return _wlan


def ip():
    """未连接返回 '0.0.0.0'。"""
    if _wlan is None:
        return "0.0.0.0"
    try:
        return _wlan.ifconfig()[0]
    except Exception:
        return "0.0.0.0"


def is_up():
    if _wlan is None:
        return False
    try:
        return bool(_wlan.isconnected()) and ip() != "0.0.0.0"
    except Exception:
        return False


def connect(timeout_s=20):
    """连上并等到有 IP。返回 True/False。"""
    if _wlan is None:
        raise RuntimeError("netup.init() 未调用")
    if is_up():
        return True
    try:
        _wlan.connect(_SSID, _PASS)
    except Exception as e:
        print("netup: connect() 抛出", e)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if is_up():
            # 额外静置，让速率自适应爬升完（见模块 docstring）
            time.sleep(3)
            return True
        time.sleep(0.5)
    return False


def ensure(timeout_s=20):
    """断线重连。已在线上直接返回 True。"""
    if is_up():
        return True
    try:
        _wlan.disconnect()
    except Exception:
        pass
    time.sleep(0.5)
    return connect(timeout_s)


def creds():
    """当前在用的 (ssid, password)。"""
    return (_SSID, _PASS)


def _drop():
    """断开并把状态清干净，供 reconnect 用。

    `disconnect()` 之后 WLAN 不是立刻回到"未连接"。**必须等到 is_up() 真的变 False**
    再 connect —— 否则新一次 connect() 可能被旧连接吞掉：以为换网成功，
    实际上还在旧网上（那是最坏的结果：回滚也没触发，下次上电才发现）。
    """
    try:
        _wlan.disconnect()
    except Exception:
        pass
    t0 = time.time()
    while time.time() - t0 < 5 and is_up():
        time.sleep(0.2)
    time.sleep(0.5)


def reconnect(ssid, password, timeout_s=15):
    """换 WiFi（SSID/密码）并重连；**失败自动回滚到原来的网络**。

    返回 `(ok, detail)`：

        (True,  "unchanged")                      —— 跟现在一样，什么都没做
        (True,  "connected 10.64.35.109")         —— 换过去了（IP 可能变了）
        (False, "new wifi failed -> reverted to X") —— 新网连不上，已回到旧网
        (False, "new wifi failed AND rollback failed ...") —— 两头都没连上（要救援）

    ## 为什么要回滚（这不是可选项）

    远程换 WiFi 最危险的失败模式是：**新 SSID 写错一个字，板子从此不在任何网上**，
    网络入口全部消失，只能拔卡。回滚把"换错"变成"白折腾一次"：
    这里把 WLAN 拉回旧网络，调用方（app.py）则**只在成功后才写盘** ——
    坏凭据根本不会进 config_remote.json，下次上电不受影响。

    ## 代价

    这个函数会**阻塞**：新网最多等 `timeout_s`，回滚再最多等 `timeout_s`。
    期间没有推流、没有上报。它是显式的配置命令，不是常态路径，可以接受；
    但**绝不能**放在 10 Hz 那段里（见 app.py 主循环的节流约定）。

    ⚠️ **未在真机上验证过换网路径**（2026-11-18 写代码时板子没在线）。
    第一次现场使用时盯着串口：成功应看到 `WIFI OK <新IP>`，
    失败应看到回滚后的 `WIFI OK <旧IP>`。
    """
    global _SSID, _PASS
    old = (_SSID, _PASS)
    if (ssid, password) == old:
        return (True, "unchanged")

    _SSID, _PASS = ssid, password
    _drop()
    if connect(timeout_s):
        return (True, "connected %s" % ip())

    # 新网失败 -> 回滚。**用旧凭据再连一次**，成功了板子就还在网上。
    _SSID, _PASS = old
    _drop()
    if connect(timeout_s):
        print("WIFI ROLLBACK OK -> %s (%s)" % (ip(), old[0]))
        return (False, "new wifi failed -> reverted to %s" % old[0])
    print("WIFI ROLLBACK FAILED —— 板子现在不在网上，只能拔卡改 config_local.py")
    return (False, "new wifi failed AND rollback failed (old=%s)" % old[0])
