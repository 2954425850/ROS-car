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
