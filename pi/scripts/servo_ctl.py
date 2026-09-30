#!/usr/bin/env python3
"""L150Pro 舵机手动控制（不需要手动 source ROS 环境）。

用法:  python3 ~/servo_ctl.py
命令:
    <通道> <微秒>   例: 5 1200   -> ch5 转到 1200us
    c              所有通道回中位 1500us
    h <通道>        例: h 3      -> ch3 保持当前位置不动
    s              显示当前设定
    q              退出

通道对照: 1=PC6  2=PC7  3=PC8  4=PC9  5=PB14  6=PB15
范围 500~2500us，1500 是中位。0 = 该路保持不动。
"""
import os
import sys

# ---- 自动加载 ROS 环境：没 source 就自己重跑一遍 ----
def _ensure_ros():
    try:
        import rclpy  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_L150PRO_ROS") == "1":
        sys.stderr.write(
            "错误：加载 ROS 2 环境失败。请手动执行：\n"
            "  source /opt/ros/jazzy/setup.bash\n"
            "  source ~/l150pro_ws/install/setup.bash\n")
        sys.exit(1)
    os.environ["_L150PRO_ROS"] = "1"
    cmd = ('source /opt/ros/jazzy/setup.bash; '
           'source "$HOME/l150pro_ws/install/setup.bash" 2>/dev/null; '
           'exec python3 "$0" "$@"')
    os.execvp("bash", ["bash", "-c", cmd, sys.argv[0]] + sys.argv[1:])

_ensure_ros()

# ---- 以下是正常工作路径 ----
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

PIN = {1: 'PC6', 2: 'PC7', 3: 'PC8', 4: 'PC9', 5: 'PB14', 6: 'PB15'}


class Ctl(Node):
    def __init__(self):
        super().__init__('servo_ctl')
        self.pub = self.create_publisher(JointState, '/servo_cmd', 10)
        self.pos = [1500.0] * 6
        self.lock = threading.Lock()
        self.running = True

    def spin_pub(self):
        while self.running:
            m = JointState()
            m.name = [f'ch{i}' for i in range(1, 7)]
            with self.lock:
                m.position = list(self.pos)
            self.pub.publish(m)
            time.sleep(0.05)

    def set(self, ch, us):
        with self.lock:
            self.pos[ch - 1] = float(us)

    def show(self):
        with self.lock:
            for i, v in enumerate(self.pos, 1):
                s = f"{int(v)}us" if v >= 500 else "保持不动"
                print(f"  ch{i} ({PIN[i]:>4}): {s}")


def main():
    rclpy.init()
    n = Ctl()
    threading.Thread(target=n.spin_pub, daemon=True).start()
    print(__doc__)
    print("当前设定:")
    n.show()
    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue
            p = line.split()
            try:
                if p[0] == 'q':
                    break
                elif p[0] == 'c':
                    for i in range(6):
                        n.set(i + 1, 1500)
                    print("全部回中位")
                elif p[0] == 's':
                    pass
                elif p[0] == 'h' and len(p) == 2:
                    n.set(int(p[1]), 0)
                    print(f"ch{p[1]} 保持不动")
                elif len(p) == 2:
                    ch, us = int(p[0]), int(p[1])
                    if not 1 <= ch <= 6:
                        raise ValueError("通道 1~6")
                    if not 500 <= us <= 2500:
                        raise ValueError("脉宽 500~2500us")
                    n.set(ch, us)
                    print(f"ch{ch} ({PIN[ch]}) -> {us}us")
                else:
                    print("命令看不懂，输入 s 看用法")
                    continue
                n.show()
            except (ValueError, IndexError) as e:
                print(f"参数错误: {e}")
    finally:
        n.running = False
        time.sleep(0.1)
        n.destroy_node()
        rclpy.shutdown()
        print("已退出——舵机保持最后位置（固件设计如此，不会回中）")


if __name__ == '__main__':
    main()
