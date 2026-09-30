# /home/cy/k230-vision/pi/seriallog.py
#
# 常驻记录 K230 板子的串口控制台到文件。
#
# ## 为什么需要
#
# 板子的自启钩子 `/sdcard/main.py` 崩溃时只做 `print("APP CRASH", repr(e))`
# 然后**掉回 REPL，不重启 app**。所以：
#   · 崩了之后 RTSP / 结果上报 / 串口 STATS 全停，但网卡还在（ping 得通），
#     看上去像"板子死了"，其实是"app 退出了"；
#   · 而那句 `APP CRASH` 是**唯一**的崩溃原因线索 —— 没人读串口就永远丢了。
#
# 2026-09-21 实测就是这么丢了一次（09:23:50 崩，发现时已经过了 1 分钟）。
#
# ## 注意
#
# `/dev/ttyACM*` **同一时刻只能被一个进程打开**。这个记录器跑着的时候，
# 那些临时探针（replpoke / peekstats 之类）会打不开端口 ——
# 要用探针就先 `systemctl --user stop k230-seriallog`，用完再起。
#
# 板子复位会重新枚举 USB，所以这里出错就重开端口，不要退出。
#
# 用法：
#   systemd-run --user --unit=k230-seriallog --collect \
#     --property=StandardOutput=append:/tmp/board-serial.log \
#     --property=StandardError=append:/tmp/board-serial.log \
#     bash -lc 'exec python3 -u /home/cy/k230-vision/pi/seriallog.py'

import sys
import time

import serial

PORT = "/dev/ttyACM1"


def main():
    while True:
        try:
            s = serial.Serial(PORT, 115200, timeout=1.0)
        except Exception as e:
            print("\n[seriallog] 打不开 %s: %s（2 秒后重试）" % (PORT, e), flush=True)
            time.sleep(2)
            continue
        print("\n[seriallog] 已打开 %s" % PORT, flush=True)
        try:
            while True:
                b = s.read(4096)
                if b:
                    sys.stdout.buffer.write(b)
                    sys.stdout.buffer.flush()
        except Exception as e:
            print("\n[seriallog] 读失败: %s（重开端口）" % e, flush=True)
            try:
                s.close()
            except Exception:
                pass
            time.sleep(2)


if __name__ == "__main__":
    main()
