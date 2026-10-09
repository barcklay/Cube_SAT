#!/usr/bin/env python3
"""Download the flight log from the board's SPI flash as a CSV file.

Sends the console command 'D' to the hab_bringup firmware and saves
everything between "LOGDUMP BEGIN" and "LOGDUMP END". Then prints a short
summary: records per boot and whether the record numbers are continuous
(a power cut may leave one broken record, which the firmware skips).

Usage:
    python3 tools/logdump.py [output.csv]     # default: flightlog_<date>_<time>.csv
"""

import glob
import os
import select
import sys
import termios
import time


def open_port():
    ports = sorted(glob.glob("/dev/cu.usbmodem*"), key=lambda p: (not p[len("/dev/cu.usbmodem"):].isdigit(), p))  # the ST-LINK port is all digits; the Arduino receiver is not
    if not ports:
        sys.exit("board not found (/dev/cu.usbmodem*)")
    fd = os.open(ports[0], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    time.sleep(0.2)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else time.strftime("flightlog_%Y%m%d_%H%M%S.csv")
    fd = open_port()
    os.write(fd, b"D")

    buf = b""
    lines = []
    started = False
    last_data = time.time()
    while time.time() - last_data < 10:
        ready, _, _ = select.select([fd], [], [], 0.2)
        if not ready:
            continue
        try:
            buf += os.read(fd, 4096)
        except BlockingIOError:  # select() can wake up with nothing to read yet
            continue
        last_data = time.time()
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if "LOGDUMP BEGIN" in line:  # may share a line with a half-printed frame
                started = True
                print(f"downloading {line.split()[-1]} records…")
                continue
            if line.startswith("LOGDUMP END"):
                os.close(fd)
                finish(out_path, lines, line)
                return
            if started and line and not line.startswith(("FRM", "LOOP")):
                lines.append(line)
                if len(lines) % 1000 == 0:
                    print(f"  {len(lines)} lines")
    os.close(fd)
    sys.exit("no LOGDUMP END received (is the firmware running and the port free?)")


def finish(out_path, lines, end_line):
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    rows = [l.split(",") for l in lines[1:]]
    print(f"saved {len(rows)} records to {out_path}  ({end_line})")

    per_boot = {}
    for r in rows:
        per_boot.setdefault(r[1], []).append(int(r[0]))
    for boot, idx in per_boot.items():
        print(f"  boot {boot:>4}: {len(idx):6d} records, index {idx[0]}..{idx[-1]}")
    idx = [int(r[0]) for r in rows]
    gaps = [(a, b) for a, b in zip(idx, idx[1:]) if b != a + 1]
    print("  index continuous" if not gaps else f"  gaps (skipped broken slots): {gaps[:10]}")


if __name__ == "__main__":
    main()
