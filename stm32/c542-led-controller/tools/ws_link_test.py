#!/usr/bin/env python3
"""Drive the C542 LED controller over the ST-LINK VCP (COM16).

Sends the binary STATE frames defined in project/ws_link.h so the strip can
be exercised with no F407 attached.  Also prints anything the firmware logs
on the same port ("[LINK] up/lost", demo stages).

Usage:  python ws_link_test.py [COMx] [baud]
"""
import struct
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM16"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

F_ESTOP = 0x01
F_LOWBAT = 0x02
F_BRAKE = 0x04
F_REVERSE = 0x08
F_ROS = 1 << 4          # ros_link = 1 (connected)
F_TURN_L = 0x40
F_TURN_R = 0x80

MODE_BLANK = 0
MODE_NORMAL = 1


def crc8(data: bytes) -> int:
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
    return c


def state_frame(flags, bright, speed_cms, nav, mode, seq):
    payload = bytes([flags & 0xFF, bright & 0xFF]) + struct.pack("<hH", speed_cms, nav) + bytes([mode & 0xFF])
    body = bytes([0x01, 0x01, len(payload), seq & 0xFF]) + payload
    return bytes([0xAA, 0x55]) + body + bytes([crc8(body)])


class Link:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0)
        self.seq = 0
        self.t0 = time.time()

    def send(self, flags, bright=128, speed_cms=0, nav=0xFFFF, mode=MODE_NORMAL):
        f = state_frame(flags, bright, speed_cms, nav, mode, self.seq)
        self.seq = (self.seq + 1) & 0xFF
        self.ser.write(f)

    def drain_log(self):
        data = self.ser.read(4096)
        if data:
            text = data.decode("ascii", "replace")
            for line in text.splitlines():
                if line.strip():
                    print(f"    [vcp] {line.strip()}")

    def run(self, seconds, hz=25, **kw):
        n = int(seconds * hz)
        for _ in range(n):
            self.send(**kw)
            self.drain_log()
            time.sleep(1.0 / hz)


def main():
    print(f"open {PORT} @ {BAUD}")
    link = Link(PORT, BAUD)
    time.sleep(0.2)
    link.drain_log()

    print("1) idle cyan (ROS connected), 3 s")
    link.run(3.0, flags=F_ROS)

    print("2) speed 0.45 m/s + TURN LEFT, 3 s")
    link.run(3.0, flags=F_ROS | F_TURN_L, speed_cms=45)

    print("3) BRAKE, 3 s")
    link.run(3.0, flags=F_ROS | F_BRAKE, speed_cms=30)

    print("4) E-STOP, 3 s")
    link.run(3.0, flags=F_ESTOP)

    print("5) BLANK (mode 0), 2 s")
    link.run(2.0, flags=F_ROS, mode=MODE_BLANK)

    print("6) idle again, 3 s")
    link.run(3.0, flags=F_ROS)

    print("7) stop sending for 2.5 s -> expect [LINK] lost + purple cue")
    time.sleep(2.5)
    link.drain_log()

    print("8) resume idle, 3 s")
    link.run(3.0, flags=F_ROS)

    print("9) final BLANK, 1 s")
    link.run(1.0, flags=F_ROS, mode=MODE_BLANK)

    link.drain_log()
    link.ser.close()
    print("done")


if __name__ == "__main__":
    main()
