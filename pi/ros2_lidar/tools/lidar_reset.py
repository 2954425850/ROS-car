#!/usr/bin/env python3
"""Start the RPLIDAR from a clean state.

Without this, restarting the driver right after a previous run often hangs in
getDeviceInfo: the lidar is left streaming/motor-running and the new driver
cannot complete its handshake. STOP + RESET puts it back to idle.

Also note: on this CP2102 adapter the motor runs when DTR is LOW. The driver
sets that itself; this script only resets the device.
"""
import sys, time
import serial

port = sys.argv[1]
s = serial.Serial(port, 115200, timeout=0.5)
s.dtr = False
s.rts = True
time.sleep(0.3)
s.reset_input_buffer()
s.write(bytes([0xA5, 0x25]))   # STOP
time.sleep(0.3)
s.reset_input_buffer()
s.write(bytes([0xA5, 0x40]))   # RESET
time.sleep(1.5)
s.close()
print('lidar reset done:', port)
