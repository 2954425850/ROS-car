#!/usr/bin/env python3
"""Pi 换 WiFi 时，把 K230 的 WiFi 和指向地址一起迁过去。

## 为什么不能"一条命令同时改两边"

两边都在网上时才能发命令；Pi 一旦离开旧网，就**再也够不到**还在旧网上的板子。
所以顺序只能是：

    1. 还在旧网时，先把 K230 安排好（换 WiFi、换指向）；
    2. 再切 Pi；
    3. 到新网上确认两边接上了。

真正的难点是第 1 步：Pi 在新网上的 IP 通常要**切过去之后**才知道（DHCP），
而 K230 现在就需要知道往哪发。两种解法：

### 模式 A：`--new-host`（推荐，最稳）

Pi 在目标网有**固定 IP 或 DHCP 保留**，那就在旧网一次把
「WiFi + result/push 指向」全发给 K230，然后切 Pi。一次到位，不用发现。

### 模式 B：自动发现（Pi 的 IP 事先不知道）

1. 只给 K230 发「换 WiFi」；
2. 切 Pi；
3. 在**新网的同网段**里扫描 8558（配置端口）找板子；
4. 找到后把 result/push 指向 Pi 的新 IP；
5. 用 `k230-resultd` 的收包计数做端到端验证。

模式 B 依赖：Pi 和 K230 在**同一个子网**、AP **没有客户端隔离**（guest 网络常见）。

## 失败会怎样（必须先知道）

- K230 的新 WiFi 连不上：板子自己**回滚到旧网络**，且坏凭据不会落盘。
  但这时 Pi 已经在新网 —— 两边就分开了。按日志里给的
  `nmcli connection up "<旧连接>"` 把 Pi 切回旧网即可找回板子。
- Pi 连不上新 WiFi：K230 可能已经过去了。先在 Pi 上把 WiFi 连好，
  再跑一次本命令（或 `k230ctl config` 看板子在哪）。

## 硬件前提（很容易漏）

- **K230 的 WiFi 是 2.4GHz**。目标网如果是 5GHz-only，板子根本连不上，
  只会回滚 —— 这不是本脚本能解决的。
- 目标网必须 Pi 和 K230 都能加入；访客网/隔离网会让第 3 步永远找不到板子。

## 关于 SSH

换 WiFi 会断开 SSH / 当前终端所在的会话。所以命令**默认转到后台**
（日志 `/tmp/k230/switch-net-<时间>.log`，状态 `/tmp/k230/switch-net-status.json`）。
本地终端想盯着看就加 `--foreground`（SSH 那条会话会随换网一起死）。
"""

import json
import os
import re
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import k230pi

CONFIG_PORT = k230pi.CONFIG_PORT
SCAN_WORKERS = 64
PROBE_TIMEOUT = 0.4
STATUS_FILE = os.path.join(k230pi.OUT_DIR, "switch-net-status.json")


class SwitchError(RuntimeError):
    pass


# ---------------- 小工具 ----------------

def _run(cmd, timeout=30):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except FileNotFoundError:
        raise SwitchError("命令不存在：%s" % cmd[0])
    except subprocess.TimeoutExpired:
        raise SwitchError("命令超时：%s" % " ".join(cmd))


def active_wifi_dev():
    """当前已连接的 WiFi 设备名（nmcli），没有则 None。"""
    r = _run(["nmcli", "-t", "-f", "DEVICE,TYPE,STATE", "device", "status"])
    if r.returncode != 0:
        raise SwitchError("nmcli 不可用：%s" % (r.stderr or r.stdout).strip())
    for line in r.stdout.splitlines():
        parts = line.split(":")
        if len(parts) >= 3 and parts[1] == "wifi" and parts[2] == "connected":
            return parts[0]
    return None


def active_conn(dev):
    """该接口当前的 NetworkManager 连接名（用于失败时切回）。"""
    r = _run(["nmcli", "-t", "-f", "GENERAL.CONNECTION", "device", "show", dev])
    txt = (r.stdout or "").strip()
    if ":" not in txt:
        return None
    name = txt.split(":", 1)[1].strip()
    return name or None


def ipv4_of(dev):
    """(ip, prefixlen) 或 (None, None)。"""
    r = _run(["ip", "-4", "-o", "addr", "show", "dev", dev, "scope", "global"])
    m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)/(\d+)", r.stdout or "")
    if not m:
        return (None, None)
    return (m.group(1), int(m.group(2)))


def hosts_in_prefix(ip, prefix=24):
    """要扫的地址列表。**最多按 /24 扫** —— /16 扫 6 万个地址没有意义。"""
    import ipaddress
    if prefix < 24:
        prefix = 24
    net = ipaddress.ip_network("%s/%d" % (ip, prefix), strict=False)
    return [str(h) for h in net.hosts()]


def permission_hint():
    """nmcli 改网络是否需要额外授权；需要时返回那个值（如 'auth'），否则 None。"""
    r = _run(["nmcli", "-t", "general", "permissions"])
    if r.returncode != 0:
        return None
    for line in (r.stdout or "").splitlines():
        if line.startswith("org.freedesktop.NetworkManager.network-control:"):
            val = line.split(":", 1)[1].strip()
            return None if val == "yes" else val
    return None


def nmcli_connect(ssid, password=None, hidden=False):
    cmd = ["nmcli", "--wait", "60", "device", "wifi", "connect", ssid]
    if password:
        cmd += ["password", password]
    if hidden:
        cmd += ["hidden", "yes"]
    r = _run(cmd, timeout=90)
    if r.returncode != 0:
        txt = (r.stderr or r.stdout or "").strip()
        hint = ""
        if "authoriz" in txt.lower() or "permission" in txt.lower():
            hint = ("\n需要权限：用 `sudo k230ctl switch-net ...` 重跑，"
                    "或把用户加入 netdev 组")
        raise SwitchError("Pi 连接 WiFi %r 失败：%s%s" % (ssid, txt, hint))
    return True


def wait_ipv4(dev, timeout=25):
    t_end = time.time() + timeout
    while time.time() < t_end:
        ip, prefix = ipv4_of(dev)
        if ip:
            return ip, prefix
        time.sleep(0.5)
    raise SwitchError("%s 在 %ds 内没拿到 IPv4" % (dev, timeout))


def wait_pi_change(dev, old_ip, old_conn=None, timeout=25):
    """--no-pi-switch 时：等用户自己把 Pi 换到新网（地址或连接名变了）。

    不能只看 IP —— 同一个路由器下换 SSID，DHCP 很可能给同一个地址。
    """
    t_end = time.time() + timeout
    while time.time() < t_end:
        ip, _ = ipv4_of(dev)
        if old_conn is not None and active_conn(dev) != old_conn:
            return ip
        if ip and ip != old_ip:
            return ip
        time.sleep(0.5)
    raise SwitchError("等 Pi 换网超时（%ds 内 %s 的地址和连接都没变）"
                      % (timeout, dev))


# ---------------- 在新网找板子 ----------------

def probe(host, timeout=PROBE_TIMEOUT):
    """8558 上发一句 config_get；确实是 K230 才返回应答 dict。"""
    try:
        s = socket.create_connection((host, CONFIG_PORT), timeout=timeout)
    except OSError:
        return None
    try:
        s.settimeout(timeout)
        s.sendall(b'{"cmd":"config_get"}\n')
        buf = b""
        while b"\n" not in buf and len(buf) < 4096:
            chunk = s.recv(512)
            if not chunk:
                break
            buf += chunk
        if b"\n" not in buf:
            return None
        obj = json.loads(buf.split(b"\n")[0].decode("utf-8", "replace"))
    except (OSError, ValueError, UnicodeDecodeError):
        return None
    finally:
        try:
            s.close()
        except OSError:
            pass
    if isinstance(obj, dict) and obj.get("ok") and isinstance(obj.get("config"), dict):
        return obj
    return None


def find_board(ip, prefix, timeout=90.0, log=print):
    """在新网扫描 8558，返回 (板子IP, config_get 应答)。找不到抛 SwitchError。"""
    own = ip
    targets = [h for h in hosts_in_prefix(ip, prefix) if h != own]
    t_end = time.time() + timeout
    sweep = 0
    while time.time() < t_end:
        sweep += 1
        with ThreadPoolExecutor(max_workers=SCAN_WORKERS) as ex:
            futs = {ex.submit(probe, h): h for h in targets}
            for fut in futs:
                info = fut.result()
                if info is not None:
                    return (futs[fut], info)
        log("   扫描 %d 个地址（第 %d 轮）没找到，2s 后重试…" % (len(targets), sweep))
        time.sleep(2)
    raise SwitchError("在 %s/%d 里没找到 K230（%ds）。它可能已经回滚到旧网 —— "
                      "按下面的命令把 Pi 切回旧网再排查。" % (ip, max(prefix, 24), timeout))


# ---------------- 配置计划（纯函数，可单测）----------------

def make_set_plan(ssid=None, password=None, new_host=None, old_cfg=None):
    """组装一条 config_set。`old_cfg` 是切换前 config_get 的 config 段。

    obstacle_host 的处理：原来 `null`（跟随 result_host）就继续跟随；
    原来显式写成旧 Pi IP 的，就改成 `None` 让它跟随新地址；
    原来指向别的机器的，不动。
    """
    plan = {"cmd": "config_set"}
    if ssid is not None:
        plan["wifi_ssid"] = ssid
    if password is not None:
        plan["wifi_pass"] = password
    if new_host is not None:
        plan["result_host"] = new_host
        plan["push_host"] = new_host
        if old_cfg is not None:
            ob = old_cfg.get("obstacle_host")
            if ob is not None and ob == old_cfg.get("result_host"):
                plan["obstacle_host"] = None
    return plan


def mask_plan(plan):
    """日志里别打印 WiFi 密码。"""
    p = dict(plan)
    if "wifi_pass" in p:
        p["wifi_pass"] = "***"
    return p


def _write_status(ok, **fields):
    st = {"ok": bool(ok), "t": time.strftime("%Y-%m-%d %H:%M:%S")}
    st.update(fields)
    try:
        k230pi.ensure_out_dir()
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(st, f, ensure_ascii=False)
        os.replace(tmp, STATUS_FILE)
    except OSError:
        pass


# ---------------- 主流程 ----------------

def run(ssid, password=None, new_host=None, timeout=90.0, pi_timeout=25.0,
        iface=None, hidden=False, no_pi_switch=False, board_host=None,
        board_new_host=None, log=print):
    """执行换网。成功返回 0，失败返回 1（并写 STATUS_FILE）。"""
    old_conn = None
    old_pi_ip = None
    try:
        # ---- 0) 旧网预检 ----
        old_board = board_host or k230pi.board_host()
        log("K230 当前地址：%s" % old_board)
        info = k230pi.get_config(host=old_board)
        if not info.get("ok"):
            raise SwitchError("板子拒绝 config_get：%s" % info.get("err"))
        old_cfg = info.get("config") or {}
        old_board_ip = info.get("ip")
        log("板子：ip=%s  push=%s  result=%s  obstacle=%s"
            % (old_board_ip, old_cfg.get("push_host"), old_cfg.get("result_host"),
               old_cfg.get("obstacle_host")))

        dev = iface or active_wifi_dev()
        if not dev:
            raise SwitchError("没有已连接的 WiFi 接口（nmcli）；可以用 --iface 指定")
        old_conn = active_conn(dev)
        old_pi_ip, _ = ipv4_of(dev)
        log("Pi：接口=%s  连接=%r  ip=%s" % (dev, old_conn, old_pi_ip))

        # ---- 1) 先把安排发给 K230（此时 Pi 还在旧网）----
        plan = make_set_plan(ssid, password, new_host, old_cfg)
        log("1/4 下发到板子：%s" % mask_plan(plan))
        resp = k230pi.send_config(plan, host=old_board)
        if not resp.get("ok"):
            raise SwitchError("板子拒绝配置：%s" % resp.get("err"))
        log("     受理：accepted=%s persisted=%s deferred=%s"
            % (resp.get("accepted"), resp.get("persisted"), resp.get("deferred")))

        # ---- 2) 切 Pi ----
        if not no_pi_switch:
            log("2/4 切换 Pi 到 WiFi %r（SSH 会断）…" % ssid)
            nmcli_connect(ssid, password, hidden)
        else:
            log("2/4 等你手动把 Pi 切到 %r（最多 %ds）…" % (ssid, pi_timeout))
            wait_pi_change(dev, old_pi_ip, old_conn, timeout=pi_timeout)
        new_ip, new_prefix = wait_ipv4(dev, timeout=pi_timeout)
        log("     Pi 新地址：%s/%d" % (new_ip, new_prefix))
        if new_host is not None and new_host != new_ip:
            log("     ⚠ --new-host=%s 与 Pi 实际地址 %s 不一致，改为自动发现修正"
                % (new_host, new_ip))

        # ---- 3) 让板子指向新 IP ----
        target_board = None
        if board_new_host is not None:
            # 板子在新网的地址已知（DHCP 保留 / 路由表里看到的）—— 不用扫
            log("3/4 按 --board-new-host=%s 直接改指向…" % board_new_host)
            fix = make_set_plan(None, None, new_ip, old_cfg)
            try:
                resp = k230pi.send_config(fix, host=board_new_host)
            except k230pi.ConfigError as e:
                log("     连不上 %s（%s），改为扫描" % (board_new_host, e))
            else:
                if not resp.get("ok"):
                    raise SwitchError("改指向失败：%s" % resp.get("err"))
                target_board = board_new_host
                log("     已把 result/push 指向 %s" % new_ip)

        if target_board is None:
            need_scan = True
            if new_host is not None and new_host == new_ip:
                log("3/4 板子在切网前已指向 %s，无需扫描" % new_ip)
                need_scan = False
            elif new_host is None and new_ip == old_pi_ip \
                    and old_cfg.get("result_host") == new_ip:
                log("3/4 Pi 地址没变（%s），板子的指向不用改" % new_ip)
                need_scan = False

            if need_scan:
                log("3/4 在 %s/%d 里找 K230（最多 %ds）…"
                    % (new_ip, max(new_prefix or 24, 24), timeout))
                target_board, got = find_board(new_ip, new_prefix or 24,
                                              timeout=timeout, log=log)
                log("     找到 K230：%s（它自己报的 ip=%s）"
                    % (target_board, got.get("ip")))
                fix = make_set_plan(None, None, new_ip, old_cfg)
                log("     改指向：%s" % mask_plan(fix))
                resp = k230pi.send_config(fix, host=target_board)
                if not resp.get("ok"):
                    raise SwitchError("改指向失败：%s" % resp.get("err"))

        # ---- 4) 端到端验证（resultd 的收包计数必须往前走）----
        st_before = k230pi.read_status()
        msgs0 = (st_before or {}).get("msgs")
        ok = _verify_resultd(msgs0, timeout=max(40.0, timeout / 2), log=log)
        if not ok:
            raise SwitchError("板子/结果流没有在新网恢复（见上面的验证日志）")
        if target_board is None:
            # 没扫描时板子的新 IP 从 resultd 的对端拿（它刚连上来）
            peer = (k230pi.read_status() or {}).get("peer")
            if isinstance(peer, str) and ":" in peer:
                target_board = peer.rsplit(":", 1)[0]

        log("完成：Pi=%s  K230=%s" % (new_ip, target_board))
        _write_status(True, ssid=ssid, pi_ip=new_ip, pi_old_ip=old_pi_ip,
                      board=target_board, board_old_ip=old_board_ip,
                      detail="ok")
        return 0
    except SwitchError as e:
        log("失败：%s" % e)
        if old_conn:
            log('把 Pi 切回旧连接：nmcli connection up "%s"' % old_conn)
        _write_status(False, ssid=ssid, pi_ip=old_pi_ip, detail=str(e))
        return 1
    except k230pi.ConfigError as e:
        log("失败（连不上板子）：%s" % e)
        if old_conn:
            log('把 Pi 切回旧连接：nmcli connection up "%s"' % old_conn)
        _write_status(False, ssid=ssid, pi_ip=old_pi_ip, detail=str(e))
        return 1


def _verify_resultd(msgs0, timeout, log=print):
    """等 resultd 的 msgs 计数超过切换前的值 —— 板子真的在新网上把数据发到了 Pi。"""
    st = k230pi.read_status()
    if st is None:
        log("4/4 resultd 没在跑，跳过端到端验证（用 k230ctl config 确认指向）")
        return True
    if msgs0 is None:
        msgs0 = st.get("msgs") or 0
    t_end = time.time() + timeout
    while time.time() < t_end:
        st = k230pi.read_status() or {}
        if (st.get("msgs") or 0) > msgs0 and st.get("connected"):
            log("4/4 验证 OK：resultd 收到新结果（msgs %s -> %s, peer=%s）"
                % (msgs0, st.get("msgs"), st.get("peer")))
            return True
        time.sleep(1)
    log("4/4 验证超时：%ds 内 resultd 没收到新结果（peer=%s, msgs=%s）"
        % (timeout, st.get("peer"), st.get("msgs")))
    return False


# ---------------- 后台执行（SSH 会断）----------------

def detach(argv_tail, log_dir=None):
    """把本命令重新拉起到独立会话里，返回日志路径。

    换 WiFi 会踢掉 SSH，所以必须脱离子进程 —— `start_new_session=True` 让
    子进程不随父进程的终端/SIGHUP 一起死。
    """
    log_dir = log_dir or k230pi.OUT_DIR
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir,
                            "switch-net-%s.log" % time.strftime("%Y%m%d-%H%M%S"))
    # __main__.__file__ 比 sys.argv[0] 可靠：经 PATH 调用、或 `python k230ctl` 都成立
    main_mod = sys.modules.get("__main__")
    script = os.path.abspath(getattr(main_mod, "__file__", None) or sys.argv[0])
    cmd = [sys.executable, script] + list(argv_tail) + ["--foreground"]
    with open(log_path, "w", encoding="utf-8") as f:
        subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL, start_new_session=True,
                         close_fds=True)
    return log_path
