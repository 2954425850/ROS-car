# /sdcard/k230vision/t_wifi.py
# Task 1 一次性验证脚本。验证完可删。
import sys
sys.path.insert(0, "/sdcard/k230vision")
import config
import netup
import time

netup.init(config.WIFI_SSID, config.WIFI_PASS)
t0 = time.time()
ok = netup.connect()
print("CONNECT", ok, "in", round(time.time() - t0, 1), "s")
print("IP", netup.ip())
print("IS_UP", netup.is_up())
print("IFCONFIG", netup._wlan.ifconfig())

# 断线重连验证：手动断开，再 ensure
netup._wlan.disconnect()
time.sleep(2)
print("AFTER_DISCONNECT is_up =", netup.is_up())
t1 = time.time()
ok2 = netup.ensure()
print("RE_ENSURE", ok2, netup.ip(), "in", round(time.time() - t1, 1), "s")
