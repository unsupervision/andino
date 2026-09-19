#!/usr/bin/env python3
"""Put an RPLIDAR back into a known idle state BEFORE its driver starts.

    rplidar-stop [/dev/ttyUSB_LIDAR]

Why: an RPLIDAR A1 keeps streaming scan data after the process driving it is killed. The next
driver to open the port sends GET_INFO into that stream and its handshake never completes —
rplidar_ros prints its banner and then nothing, for ever, with the process still up. Measured on
2026-09-19 over five starts: every start that followed a node killed MID-SCAN hung; every start
that followed an idle lidar worked. A driver cannot be relied on to stop the device on its way out
(it may be SIGKILLed), so the cure is on the way IN: STOP (0xA5 0x25), flush, then hand over.

Best effort by design — it reports what it saw and always exits 0, so a lidar that is unplugged or
already fine never prevents the driver from having its own try. Stdlib only.
"""
import os
import sys
import termios
import time

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB_LIDAR'
try:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
except OSError as e:
    print(f'rplidar-stop: cannot open {port}: {e} — leaving it to the driver', flush=True)
    sys.exit(0)

a = termios.tcgetattr(fd)
a[0] = a[1] = a[3] = 0
a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.HUPCL
a[4] = a[5] = termios.B115200
a[6][termios.VMIN] = 0
a[6][termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, a)


def drain(seconds):
    end, n = time.time() + seconds, 0
    while time.time() < end:
        try:
            n += len(os.read(fd, 65536))
        except BlockingIOError:
            time.sleep(0.01)
    return n


streaming = drain(0.3)                       # is it still sending scans nobody asked for?
os.write(fd, b'\xA5\x25')                    # STOP: leave the scanning state. It has no reply.
time.sleep(0.05)                             # the protocol asks for >= 1 ms before the next request
termios.tcflush(fd, termios.TCIOFLUSH)
after = drain(0.3)

os.write(fd, b'\xA5\x52')                    # GET_HEALTH, only to prove the line is sane again
time.sleep(0.2)
try:
    reply = os.read(fd, 64)
except BlockingIOError:
    reply = b''
os.close(fd)
ok = reply[:2] == b'\xA5\x5A'
print(f'rplidar-stop: {streaming} bytes of unrequested scan data in 0.3 s before STOP, {after} after; '
      f'health request {"answered" if ok else "NOT answered"}', flush=True)
