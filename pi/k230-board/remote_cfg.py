# /sdcard/k230vision/remote_cfg.py
"""Pi 远程可改的那几项 —— WiFi 与各接收端地址。

## 为什么不是直接改 config_local.py

`config_local.py` 是**人**维护的真值（见 config.py 文件头），里面是裸 Python 源码、
带注释、还可能有人自己加的其他键。让 Pi 去重写它等于把人的文件变成脚本生成的产物：
注释会丢，Pi 不认识的键也会被一起抹掉。

所以远程配置单独落一个**机器生成**的文件：

    /sdcard/k230vision/config_local.py     人写：WiFi、地址……（真值，不入库）
    /sdcard/k230vision/config_remote.json  板子写：Pi 通过 8558 下发的那几项

`config.py` 在 import 时先读 `config_local.py`，再读 `config_remote.json` 覆盖同名项。
**远程优先**。想撤销远程配置：`k230ctl config --clear`（删掉 JSON，重启后回到
config_local），或者直接删板上那个文件。

## 为什么是 JSON 不是 .py

不生成代码就不必关心转义（SSID/密码里可能有引号、中文、空格），
Pi 侧也能直接看懂这个文件。加载时 ujson 一句 loads 就够。

## ⚠️ 板上 SD 不能覆盖已存在的文件（2026-10-02 实测）

`open(已存在文件, 'w')` 返回 `OSError(1) EPERM`，写卡途中还可能把板子写得重启。
所以落盘一律走 `新文件 -> 删旧文件 -> rename`（`_write_atomic`）。
**掉电风险窗口**：删掉旧文件到 rename 之间。真的断在这一下，结果只是远程配置丢了、
回到 config_local —— 不影响启动，也不影响连 WiFi（只要 config_local 里有）。
"""

import os

# 与 config.py 的 REMOTE_CFG_FILE 必须一致
PATH = "/sdcard/k230vision/config_remote.json"
TMP = PATH + ".new"

# JSON 键 -> config.py 里的名字
KEYS = {
    "wifi_ssid": "WIFI_SSID",
    "wifi_pass": "WIFI_PASS",
    "push_host": "PUSH_HOST",
    "result_host": "RESULT_HOST",
    "obstacle_host": "OBSTACLE_HOST",     # null = 跟随 RESULT_HOST
}

_ALLOWED = "wifi_ssid / wifi_pass / push_host / result_host / obstacle_host"


def _json():
    try:
        import ujson
        return ujson
    except ImportError:
        import json                # PC 上跑单元测试时用
        return json


# ---------------- 校验（纯函数，不碰文件/网络）----------------

def _has_ctrl(s):
    for ch in s:
        o = ord(ch)
        if o < 32 or o == 127:
            return True
    return False


def _valid_ipv4(s):
    parts = s.split(".")
    if len(parts) != 4:
        return False
    for p in parts:
        if not p or len(p) > 3:
            return False
        for ch in p:
            if ch < "0" or ch > "9":
                return False
        if len(p) > 1 and p[0] == "0":
            return False
        if int(p) > 255:
            return False
    return True


def _valid_hostname(s):
    if not (1 <= len(s) <= 253):
        return False
    if s[0] in "-." or s[-1] in "-.":
        return False
    for ch in s:
        if not (("a" <= ch <= "z") or ("A" <= ch <= "Z")
                or ("0" <= ch <= "9") or ch in "-."):
            return False
    return True


def _looks_numeric(s):
    for ch in s:
        if not (("0" <= ch <= "9") or ch == "."):
            return False
    return True


def _valid_host(s):
    # 只由数字和点组成 = 本意肯定是 IPv4，就**必须**是合法的 IPv4。
    # 否则 "10.64.35" 会落进 hostname 分支被当成合法名字 —— 拼错的地址会被放过去，
    # 板子从此连不上 Pi，而且返回值看上去一切正常。
    if _looks_numeric(s):
        return _valid_ipv4(s)
    return _valid_hostname(s)


def validate(req):
    """把一条 config_set 请求校验成 {json键: 值}。**纯函数。**

    返回 `(changes, None)` 或 `(None, 错误字符串)`。
    只接受 KEYS 里的键 —— 拼错的键直接报错，免得"设了但没生效"。
    """
    if not isinstance(req, dict):
        return None, "not an object"
    changes = {}
    for k in req:
        if k == "cmd":
            continue
        if k not in KEYS:
            return None, "unknown key %r (allowed: %s)" % (k, _ALLOWED)
        v = req[k]
        if k == "wifi_ssid":
            if not isinstance(v, str):
                return None, "wifi_ssid must be a string"
            if _has_ctrl(v):
                return None, "wifi_ssid has control chars"
            try:
                n = len(v.encode())        # UTF-8 字节数（SSID 上限是按字节算的）
            except Exception:
                n = len(v)
            if n < 1 or n > 32:
                return None, "wifi_ssid must be 1..32 bytes"
        elif k == "wifi_pass":
            if not isinstance(v, str):
                return None, "wifi_pass must be a string"
            if _has_ctrl(v):
                return None, "wifi_pass has control chars"
            n = len(v)
            if n != 0 and n < 8:
                return None, ("wifi_pass must be 8..63 chars (or empty for "
                              "an open network) -- %d is almost certainly a typo" % n)
            if n > 64:
                return None, "wifi_pass too long (max 64)"
        else:                              # 地址类
            if v is None:
                if k != "obstacle_host":
                    return None, "%s must be an address" % k
            elif not isinstance(v, str) or not _valid_host(v):
                return None, "%s is not a valid IPv4/hostname: %r" % (k, v)
        changes[k] = v
    if not changes:
        return None, "nothing to set (allowed: %s)" % _ALLOWED
    return changes, None


# ---------------- 落盘 ----------------

def load():
    """读远程配置。文件不存在/坏了都返回 {}（不抛）。"""
    try:
        with open(PATH) as f:
            txt = f.read()
    except OSError:
        return {}
    try:
        d = _json().loads(txt)
    except ValueError:
        return {}
    if not isinstance(d, dict):
        return {}
    return d


def _write_atomic(text):
    # 板上不能覆盖已存在的文件，只能新文件 -> 删旧 -> rename（见文件头）。
    try:
        os.remove(TMP)
    except OSError:
        pass
    with open(TMP, "w") as f:
        f.write(text)
        f.flush()
    try:
        os.remove(PATH)
    except OSError:
        pass
    os.rename(TMP, PATH)


def save(changes):
    """把 changes 合并进远程配置文件。返回合并后的完整 dict。可能抛 OSError。"""
    d = load()
    d.update(changes)
    _write_atomic(_json().dumps(d))
    return d


def clear():
    """删掉远程配置。返回 True/False（本来就没有也算 False）。"""
    gone = False
    for p in (PATH, TMP):
        try:
            os.remove(p)
            gone = True
        except OSError:
            pass
    return gone
