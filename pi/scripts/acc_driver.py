import serial, time, struct, threading, sys

PORT = "/dev/l150pro"
RESET5 = (226, 500, 177, 129, 408)

def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c

def aa_frame(seq):
    f = bytes([0xAA, 0x0C, 0, 0, 0, 0, 0, 0, 0x01, seq & 0xFF])
    return f + struct.pack("<H", crc16(f))

def ac_frame(pos5, base_field, t_ms, flags, seq):
    body = struct.pack("<hhhhh", *(int(x) for x in pos5))
    body += struct.pack("<h", int(base_field))
    body += struct.pack("<H", int(t_ms)) + bytes([int(flags), seq & 0xFF])
    f = bytes([0xAC, 0x14]) + body
    return f + struct.pack("<H", crc16(f))

stop = threading.Event()
def keepalive(s, hz=25):
    seq = 0
    while not stop.is_set():
        s.write(aa_frame(seq)); seq += 1
        time.sleep(1.0 / hz)

def main():
    mode = sys.argv[1]
    s = serial.Serial(PORT, 115200, timeout=0.05)
    t = threading.Thread(target=keepalive, args=(s,), daemon=True)
    t.start()
    if mode == "enable":                      # enable [secs]
        time.sleep(float(sys.argv[2]) if len(sys.argv) > 2 else 8)
    elif mode == "move":                      # move <base_deg> [t_ms] [flags]
        deg = float(sys.argv[2]); t_ms = int(sys.argv[3]) if len(sys.argv) > 3 else 2000
        flags = int(sys.argv[4]) if len(sys.argv) > 4 else 0
        field = max(-1000, min(1000, int(round(deg * 1000.0 / 360.0))))
        s.write(ac_frame(RESET5, field, t_ms, flags, 1))
        time.sleep(float(sys.argv[5]) if len(sys.argv) > 5 else 0.3)
    elif mode == "stream":                    # stream <base_deg> <secs> [t_ms]
        deg = float(sys.argv[2]); secs = float(sys.argv[3])
        t_ms = int(sys.argv[4]) if len(sys.argv) > 4 else 40
        field = max(-1000, min(1000, int(round(deg * 1000.0 / 360.0))))
        t0 = time.time(); seq = 1
        while time.time() - t0 < secs:
            s.write(ac_frame(RESET5, field, t_ms, 0, seq)); seq += 1
            time.sleep(0.04)
    stop.set(); time.sleep(0.05); s.close()

main()
