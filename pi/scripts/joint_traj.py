import serial, time, sys
sys.path.insert(0, "/home/cy")
import _arm_probe as P

log = open("/tmp/joint_traj.txt", "w", buffering=1)
t0 = time.time()
s = serial.Serial("/dev/l150pro", 115200, timeout=0.02)
s.reset_input_buffer()
sid_cycle = (1, 2, 3, 4, 5)
k = 0
while time.time() - t0 < 20:
    t = time.time() - t0
    sid = sid_cycle[k % 5]
    k += 1
    raw = None
    for _ in range(2):
        r = P.wrap_raw(s, P.lx_frame(sid, 0x1C), 18)
        if r and len(r) == 8 and r[0] == 0x55 and r[1] == 0x55:
            import struct as _s
            raw = _s.unpack("<h", r[5:7])[0]
            break
    log.write("%.3f %d %s\n" % (t, sid, raw if raw is not None else "NA"))
    time.sleep(0.005)
s.close()
log.write("END\n")
