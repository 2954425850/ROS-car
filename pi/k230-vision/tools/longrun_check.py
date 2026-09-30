#!/usr/bin/env python3
"""长稳测试观察端：每分钟打印一次 Pi 侧 wlan0 接收速率。

K230 把 H.264 裸流推到 Pi 的 8555，Pi 的 wlan0 是这条链路唯一的落地接口，
所以 wlan0 rx 的差值就是推流速率（另有很小的 SSH / 结果上报流量，可忽略）。

用法:
    python3 longrun_check.py [duration_s]      # 默认 1900 s

输出形如:
    HH:MM:SS wlan0 rx <bytes>/min = <x.xx> Mbps   (total <n> MB, interval <t>s)

判据（与计划一致）：稳定在 **1.5~2.2 Mbps**、无长时间归零。
注意本项目链路会**瞬时塌陷**（曾实测 0.161 Mbps），单看某一分钟的低值
不能直接归因于代码 —— 要同时看板子的 `dropped` 是否在涨、板子是否还活着。
"""
import time
import sys

IFACE = "wlan0"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 1900.0


def rx_bytes():
    for line in open("/proc/net/dev"):
        if line.strip().startswith(IFACE + ":"):
            return int(line.split()[1])
    return 0


def main():
    last = rx_bytes()
    t0 = time.time()
    print("LONGRUN_CHECK start iface=%s duration=%ds target=1.5~2.2 Mbps"
          % (IFACE, DUR), flush=True)
    while time.time() - t0 < DUR:
        time.sleep(60)
        now = rx_bytes()
        el = time.time() - t0
        d = now - last
        print("%s %s rx %d bytes/min = %.2f Mbps   (total %.2f MB, t=%.0fs)"
              % (time.strftime("%H:%M:%S"), IFACE, d, d * 8 / 60 / 1e6,
                 now / 1e6, el), flush=True)
        last = now


main()
