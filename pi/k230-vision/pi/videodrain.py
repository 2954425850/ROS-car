# /home/cy/k230-vision/pi/videodrain.py
#
# 树莓派侧的 **8555 接收端**：把 K230 推来的 H.264 流读掉并**丢弃**（不落盘）。
#
# ## 为什么需要它
#
# 板子的视频推流目标由 config.PUSH_HOST 决定。原本指向阿里云 114.215.188.147:8555，
# 但实测板子↔公网那条链路顶不住 720p30 的码率：板子的推流是"**整帧原子发送，
# 发不完就断开重连**"（不缓冲，内存只有 4MB 逼出来的设计），于是它不停地
# 发送失败 → RST → 重连，**每一次重连都把主循环卡住**，结果上报跟着从 6 Hz 掉到 0.8 Hz
# （云端日志实测每轮只活 1~15 秒，源端口一直在变）。
#
# 把 PUSH_HOST 指到局域网内的树莓派，走 LAN 不走公网，抖动消失 → 板子不再反复重连
# → 结果速率回到 6 Hz。
#
# ## 边界
#
# 只收不存、不解析、不看内容。板子重新连上来是**正常现象**（它的设计就是断了就重连），
# 所以这里是 accept 循环，一个连接结束就回去 accept 下一个。
#
# 用法：
#   python3 videodrain.py            # 前台
#   systemd-run --user --unit=k230-videodrain --collect \
#       --property=StandardOutput=append:/tmp/videodrain.log \
#       --property=StandardError=append:/tmp/videodrain.log \
#       bash -lc 'exec python3 -u /home/cy/k230-vision/pi/videodrain.py'

import socket
import time

HOST = "0.0.0.0"
PORT = 8555


def main():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, PORT))
    srv.listen(8)
    print("videodrain listening on %s:%d" % (HOST, PORT), flush=True)

    n = 0
    while True:
        conn, addr = srv.accept()
        n += 1
        t0 = time.time()
        total = 0
        print("CONN #%d from %s:%d" % (n, addr[0], addr[1]), flush=True)
        try:
            while True:
                b = conn.recv(65536)
                if not b:
                    break
                total += len(b)
        except Exception as e:
            print("  ERR %s" % e, flush=True)
        finally:
            try:
                conn.close()
            except Exception:
                pass
        print("CLOSE #%d  %d bytes  %.1fs" % (n, total, time.time() - t0), flush=True)


if __name__ == "__main__":
    main()
