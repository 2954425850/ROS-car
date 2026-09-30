#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Bring-up probe for the arm bus servos, driven from the Pi.

USART2 belongs to the servo bus, so the Pi cannot talk to the servos
directly. This script wraps raw 0x55 frames in the firmware's 0xAD
passthrough frame and unwraps the 0x57 replies. That is how the M1-M5
measurements in the design doc get taken.

Build the firmware with ARM_DEBUG_PASSTHROUGH = 1 first. The passthrough
is executed by arm_task, so it needs Task 8's scheduler loop as well -
the 0xAD frame is decoded by the USART1 ISR and then proxied onto USART2.

0xAD down (all little endian):
    0      hdr 0xAD
    1      len = 7 + n                total frame length
    2..3   timeout_ms u16             how long the firmware collects bytes
    4      n                          payload length, <= 16
    5..    n raw 0x55-protocol bytes
    5+n..  crc u16 over bytes 0..4+n  (CRC-16/MODBUS)

0x57 up:
    0      hdr 0x57
    1      len = 8 + n_rx
    2      seq
    3      status (0 = ok)
    4..5   n_rx u16
    6..    n_rx reply bytes from the servo
    6+n..  crc u16

Subcommands
-----------
  id    [timeout]          read the ID of every servo 1..6
  pot   <id> <secs>        poll POS_READ at 20 Hz and print raw + degrees
  spin  <id> <speed> <secs> motor mode at `speed`, polling POS_READ throughout
  mode  <id> <0|1> <speed>  set the mode register
  torque <id> <0|1>         load / unload
  raw   <hexbytes...>       send arbitrary bytes, print the reply

NOTE: write commands get NO reply from the servo, so `raw` on a write will
just time out. That is correct behaviour, not a bug.
"""
import argparse, struct, sys, threading, time

try:
    import serial
except ImportError:
    # Importing this module must stay dependency free and side effect free:
    # _gen_arm_test.py imports it to generate the host test's cross language
    # frame vectors, and a missing serial stack must not break that. Only
    # actually opening a port needs pyserial - main() checks for it.
    serial = None

HEAD_DOWN, HEAD_UP = 0xAD, 0x57
LX_HEAD = 0x55


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def check_crc16():
    assert crc16_modbus(b'123456789') == 0x4B37, 'crc16 vector failed'


def lx_frame(servo_id: int, cmd: int, params: bytes = b'') -> bytes:
    """Build one 0x55 frame. Length = params + 3, checksum = ~sum & 0xFF.

    Byte for byte the same frame the firmware's lx_build() produces. The
    two implementations are independent, so agreement between them is a
    real cross check - see Task 1's T1 cases for the expected hex.
    """
    length = (len(params) + 3) & 0xFF
    body = bytes([servo_id, length, cmd]) + params
    chk = (~sum(body)) & 0xFF
    return bytes([LX_HEAD, LX_HEAD]) + body + bytes([chk])


_TXLOCK = threading.Lock()   # serialise port.write: main thread vs keepalive
_STOP = threading.Event()


def wrap_raw(port, payload: bytes, timeout_ms: int) -> bytes:
    """Send `payload` over the passthrough frame; return the servo's bytes.

    The length byte counts the WHOLE frame, the same convention the
    firmware's ros_raw_byte() decodes: 1 hdr + 1 len + 2 timeout + 1 n +
    len(payload) + 2 crc.
    """
    n_payload = len(payload)
    frame = bytes([HEAD_DOWN, 7 + n_payload,
                   timeout_ms & 0xFF, (timeout_ms >> 8) & 0xFF,
                   n_payload]) + payload
    frame += struct.pack('<H', crc16_modbus(frame))
    port.reset_input_buffer()
    with _TXLOCK:
        port.write(frame)
    port.flush()

    # The uplink interleaves the 25 Hz 0x56 arm feedback and the 50 Hz 0x55
    # chassis telemetry with the 0x57 reply, so the 0x57 never sits at
    # buf[0] - checking only the first byte made every query time out once
    # real telemetry was flowing. Scan the stream for a CRC-valid 0x57
    # frame instead; a 0x57 byte inside telemetry payload fails the CRC
    # check and is skipped.
    buf = b''
    deadline = time.time() + (timeout_ms / 1000.0) + 0.25
    while time.time() < deadline:
        chunk = port.read(128)
        if chunk:
            buf += chunk
            k = buf.find(HEAD_UP.to_bytes(1, 'big'))
            while k != -1:
                if len(buf) - k >= 2:
                    ln = buf[k + 1]
                    if 8 <= ln <= 24 and len(buf) - k >= ln:
                        fr = buf[k:k + ln]
                        if crc16_modbus(fr[:ln - 2]) == fr[ln - 2] | (fr[ln - 1] << 8):
                            n = struct.unpack('<H', fr[4:6])[0]
                            return bytes(fr[6:6 + n])
                k = buf.find(HEAD_UP.to_bytes(1, 'big'), k + 1)
            # A 0x57 frame is at most 24 bytes; keep enough tail that one
            # straddling a read chunk boundary is not thrown away.
            if len(buf) > 64:
                buf = buf[-32:]
        else:
            time.sleep(0.002)
    return b''


def keepalive_thread(port, hz: float = 10.0):
    """Chassis 0xAA enable keepalive, streamed while a probe command runs.

    ArmHold in the firmware is recomputed from the CHASSIS link state
    (arm.c: estop / ROS_FAULT_TIMEOUT / chassis enable flag), and while it
    is set the runtime block sends five lx_move_stop frames EVERY 20 ms -
    which cancels any write the passthrough just made and clutters the
    bus. Streaming 0xAA with bit0 enable and zero motion clears the hold.
    Zero vx/vy/wz means the chassis itself is commanded to stand still.
    """
    head = bytes([0xAA, 0x0C, 0, 0, 0, 0, 0, 0, 0x01])
    period = 1.0 / hz
    seq = 0
    while not _STOP.is_set():
        frame = head + bytes([seq & 0xFF])
        frame += struct.pack('<H', crc16_modbus(frame))
        with _TXLOCK:
            try:
                port.write(frame)
            except Exception:
                pass
        seq += 1
        time.sleep(period)


def lx_read_pos(port, servo_id: int):
    """Returns the raw pot reading, or None on timeout."""
    reply = wrap_raw(port, lx_frame(servo_id, 0x1C), 30)
    # 55 55 ID 05 1C posL posH CS
    if len(reply) != 8 or reply[0] != LX_HEAD or reply[1] != LX_HEAD:
        return None
    body = reply[2:7]
    if reply[7] != (~sum(body)) & 0xFF:
        return None
    return struct.unpack('<h', reply[5:7])[0]


def lx_read_id(port, servo_id: int):
    reply = wrap_raw(port, lx_frame(servo_id, 0x0E), 50)
    # 55 55 ID 04 0E id CS
    if len(reply) != 7 or reply[0] != LX_HEAD:
        return None
    return reply[5]


def lx_write(port, servo_id: int, cmd: int, params: bytes):
    wrap_raw(port, lx_frame(servo_id, cmd, params), 20)


# --------------------------------------------------------------- commands

def cmd_id(port, args):
    for sid in range(1, 7):
        # one query per attempt: the firmware's own 20 ms readback shares
        # the half-duplex bus, so a single shot collides now and then
        got = None
        for _ in range(3):
            got = lx_read_id(port, sid)
            if got is not None:
                break
            time.sleep(0.02)
        print('servo %d -> %s' % (sid, 'NO REPLY' if got is None else 'ID %d' % got))


def cmd_pot(port, args):
    """M-1 helper: watch one servo's pot reading.

    Run this while the joint sits still to find the noise floor, and again
    while the base is in motor mode to see whether the reading tracks
    continuously (and whether it pins at a rail or wraps).
    """
    t0 = time.time()
    last = None
    jumps = 0
    while time.time() - t0 < args.secs:
        raw = lx_read_pos(port, args.id)
        now = time.time() - t0
        deg = None if raw is None else raw * 240.0 / 1000.0 - 120.0
        if raw is not None and last is not None and abs(raw - last) > 500:
            jumps += 1
            print('  %6.3fs  raw=%-5s  *** JUMP %+d  (wrap #%d) ***' %
                  (now, raw, raw - last, jumps))
        else:
            print('  %6.3fs  raw=%-5s  deg=%s' %
                  (now, raw, 'n/a' if deg is None else '%+.1f' % deg))
        last = raw
        time.sleep(0.05)
    print('---- %d wrap-sized jumps in %.1fs ----' % (jumps, args.secs))
    if jumps == 0:
        print('M1 verdict: no wrap seen -> likely TRUNCATING (set ARM_POT_WRAPS 0)')
    else:
        print('M1 verdict: %d wraps -> WRAPPING (set ARM_POT_WRAPS 1), '
              'check the jump spacing is a stable 240 deg' % jumps)


def cmd_spin(port, args):
    """M-3 helper: motor mode at a fixed speed, log the pot reading.

    Put the base inside the pot's valid window first, then read off the
    total angle change over `secs` to get deg/s per speed unit.
    """
    lx_write(port, args.id, 0x1F, bytes([1]))                 # torque on
    lx_write(port, args.id, 0x1D,
             bytes([1, 0]) + struct.pack('<h', args.speed))    # motor mode
    print('spinning id %d at %+d for %.1fs' % (args.id, args.speed, args.secs))
    t0 = time.time()
    first = last = None
    while time.time() - t0 < args.secs:
        raw = lx_read_pos(port, args.id)
        if raw is not None:
            if first is None:
                first = raw
            last = raw
        time.sleep(0.02)
    lx_write(port, args.id, 0x1D, bytes([1, 0]) + struct.pack('<h', 0))
    lx_write(port, args.id, 0x1F, bytes([0]))                  # torque off
    if first is None or last is None:
        print('no readings - is the servo in the pot window?')
        return
    delta_deg = (last - first) * 240.0 / 1000.0
    print('delta = %+.1f deg over %.2fs -> %.1f deg/s at speed %+d'
          % (delta_deg, args.secs, delta_deg / args.secs, args.speed))
    print('K = %.5f deg/s per speed unit' % (delta_deg / args.secs / args.speed))


def cmd_mode(port, args):
    lx_write(port, args.id, 0x1D, bytes([args.mode, 0]) + struct.pack('<h', args.speed))
    print('id %d mode -> %d speed -> %d (write has no reply, as expected)'
          % (args.id, args.mode, args.speed))


def cmd_torque(port, args):
    lx_write(port, args.id, 0x1F, bytes([args.on]))
    print('id %d torque -> %d' % (args.id, args.on))


def cmd_raw(port, args):
    payload = bytes(int(h, 16) for h in args.hexbytes)
    reply = wrap_raw(port, payload, args.timeout)
    print('sent    : ' + ' '.join('%02X' % x for x in payload))
    print('received: ' + (' '.join('%02X' % x for x in reply) or '(nothing)'))


def cmd_move(port, args):
    """M7 helper: drive one POSITION-mode joint to a raw position.

    Sends MOVE_TIME_WRITE (0x01) with the parameters in the order the
    protocol requires - position first, then time. Getting that order
    backwards sends the joint to the wrong angle, so it is generated here
    rather than typed by hand at the bench.

    This is only meaningful for servos 1..5. Servo 6 is in motor mode in
    the shipping firmware, where MOVE_TIME_WRITE does nothing; use `mode`
    or `spin` for it.
    """
    if not 0 <= args.pos <= 1000:
        sys.exit('pos must be 0..1000 (raw counts), got %d' % args.pos)
    lx_write(port, args.id, 0x01, struct.pack('<HH', args.pos, args.ms))
    print('id %d -> pos %d over %d ms' % (args.id, args.pos, args.ms))
    print('  (a write gets no reply - that is correct, not a failure)')
    print('  verify with:  pot %d 0.5   -> raw should settle near %d'
          % (args.id, args.pos))


def cmd_sweep(port, args):
    """M7 helper: step one joint between two positions, slowly.

    Intended for finding the mechanical travel of the five position
    joints: it walks pos from --lo to --hi in --step increments, pausing
    --dwell ms at each, so you can watch (and hear) for the mechanism
    stalling against a limit before anything gets hot.
    """
    positions = list(range(args.lo, args.hi + 1, args.step))
    if args.down:
        positions.reverse()
    print('sweeping id %d from %d to %d in steps of %d, %d ms each'
          % (args.id, positions[0], positions[-1], args.step, args.dwell))
    for p in positions:
        lx_write(port, args.id, 0x01, struct.pack('<HH', p, args.dwell))
        time.sleep(args.dwell / 1000.0)
        raw = lx_read_pos(port, args.id)
        print('  pos %4d -> read back %s' % (p, raw if raw is not None else 'no reply'))
    print('done. If the mechanism reached a limit part way, the position '
          'stops advancing while the command keeps going - note where.')


def main():
    check_crc16()
    if serial is None:
        sys.exit('pyserial is required: pip3 install pyserial')
    ap = argparse.ArgumentParser(description='arm bus-servo bring-up probe')
    ap.add_argument('-p', '--port', default='/dev/ttyAMA0')
    ap.add_argument('-b', '--baud', type=int, default=115200)
    ap.add_argument('--keepalive', action='store_true',
                    help='stream 0xAA enable at 25 Hz while the command '
                         'runs: clears the firmware ArmHold (chassis link '
                         'loss) and its MOVE_STOP flood so bus writes stick')
    sub = ap.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('id');     s.add_argument('timeout', type=float, default=0.5, nargs='?')
    s.set_defaults(func=cmd_id)

    s = sub.add_parser('pot');    s.add_argument('id', type=int); s.add_argument('secs', type=float, default=10.0)
    s.set_defaults(func=cmd_pot)

    s = sub.add_parser('spin');   s.add_argument('id', type=int); s.add_argument('speed', type=int)
    s.add_argument('secs', type=float, default=5.0)
    s.set_defaults(func=cmd_spin)

    s = sub.add_parser('mode');   s.add_argument('id', type=int); s.add_argument('mode', type=int)
    s.add_argument('speed', type=int, default=0)
    s.set_defaults(func=cmd_mode)

    s = sub.add_parser('torque'); s.add_argument('id', type=int); s.add_argument('on', type=int)
    s.set_defaults(func=cmd_torque)

    s = sub.add_parser('raw');    s.add_argument('hexbytes', nargs='+')
    s.add_argument('--timeout', type=int, default=50)
    s.set_defaults(func=cmd_raw)

    s = sub.add_parser('move');   s.add_argument('id', type=int); s.add_argument('pos', type=int)
    s.add_argument('ms', type=int, default=1000, nargs='?')
    s.set_defaults(func=cmd_move)

    s = sub.add_parser('sweep');  s.add_argument('id', type=int)
    s.add_argument('--lo', type=int, default=125); s.add_argument('--hi', type=int, default=875)
    s.add_argument('--step', type=int, default=50); s.add_argument('--dwell', type=int, default=600)
    s.add_argument('--down', action='store_true')
    s.set_defaults(func=cmd_sweep)

    args = ap.parse_args()
    with serial.Serial(args.port, args.baud, timeout=0) as port:
        if not args.keepalive:
            args.func(port, args)
            return
        _STOP.clear()
        worker = threading.Thread(target=keepalive_thread, args=(port,), daemon=True)
        worker.start()
        time.sleep(0.3)      # one link-watchdog period for ArmHold to clear
        try:
            args.func(port, args)
        finally:
            _STOP.set()
            worker.join(timeout=1.0)


if __name__ == '__main__':
    main()
