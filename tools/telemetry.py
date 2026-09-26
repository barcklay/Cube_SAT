#!/usr/bin/env python3
"""Live telemetry view of the hab_bringup board, mission-control style.

Reads the text the NUCLEO prints over ST-LINK (115200 baud) and shows one
colour block per second: GPS, barometer, IMU and the Airborne <4g status.
Exact coordinates are never shown (safe for public videos).

Usage:
    python3 tools/telemetry.py            # run until Ctrl+C
    python3 tools/telemetry.py --for 20   # stop after 20 s (for testing)

Only the Python standard library is used. Reconnects by itself if the USB
link drops.
"""

import glob
import os
import re
import select
import sys
import termios
import time

BAUD = termios.B115200

# ANSI colours
RST = "\033[0m"
DIM = "\033[2m"
BOLD = "\033[1m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RED = "\033[31m"
CYAN = "\033[36m"
MAGENTA = "\033[35m"
WHITE = "\033[97m"

RE_GPS = re.compile(r"GPS: UTC (\S*)\s+fix=(\d+)\s+used=(\d+)\s+in_view=(\d+)(?:.*?alt=(-?[\d.]+) m)?(?:.*?model@boot=(-?\d+))?")
RE_IMU = re.compile(r"IMU: acc g x=(-?[\d.]+) y=(-?[\d.]+) z=(-?[\d.]+)\s+gyro dps x=(-?[\d.]+) y=(-?[\d.]+) z=(-?[\d.]+)")
RE_OUT = re.compile(r"DS18B20: T=(-?[\d.]+) C")
RE_BMP = re.compile(r"^T=(-?[\d.]+) C\s+P=(-?[\d.]+) Pa\s+alt=(-?[\d.]+) m\s+rel=(-?[\d.]+) m")

# Boot messages worth showing (dimmed) when the board restarts.
BOOT_PATTERNS = [
    (re.compile(r"Flash JEDEC: (.*)"), lambda m: f"FLASH  JEDEC {m.group(1)}", lambda m: m.group(1).strip() == "C8 40 18"),
    (re.compile(r"boot_count now (\d+)"), lambda m: f"FLASH  boot #{m.group(1)} logged", lambda m: True),
    (re.compile(r"IMU WHO_AM_I: (0x[0-9A-F]+)"), lambda m: f"IMU    ICM-42688-P id {m.group(1)}", lambda m: m.group(1) == "0x47"),
    (re.compile(r"DS18B20 ROM: .*family (0x[0-9A-F]+)"), lambda m: f"TEMP   DS18B20 family {m.group(1)}", lambda m: m.group(1) == "0x28"),
    (re.compile(r"BMP390 chip ID: (0x[0-9A-F]+)"), lambda m: f"BARO   BMP390 id {m.group(1)}", lambda m: m.group(1) == "0x60"),
    (re.compile(r"GPS: NMEA OK at (\d+) baud"), lambda m: f"GPS    link up, {m.group(1)} baud", lambda m: True),
    (re.compile(r"GPS set Airborne<4g: (\w+)"), lambda m: f"GPS    dynamic model AIRBORNE<4g -> {m.group(1)}", lambda m: m.group(1) == "ACK"),
]


def open_port():
    ports = glob.glob("/dev/cu.usbmodem*")
    if not ports:
        return None
    try:
        fd = os.open(ports[0], os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = attrs[5] = BAUD
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # ST-LINK keeps old output in its buffer; drop it so the view starts "live".
    time.sleep(0.3)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def mission_time(t0):
    s = int(time.time() - t0)
    return f"T+{s // 3600:02d}:{s // 60 % 60:02d}:{s % 60:02d}"


def banner():
    title = " HAB-1 · flight computer · NUCLEO-G474RE "
    print(f"\n{BOLD}{WHITE}██{title}██{RST}   {DIM}GPS · BARO · IMU · TEMP · FLASH{RST}")
    print(DIM + "─" * 96 + RST)


def show_block(t0, gps, imu, bmp, airborne, out_c=None):
    tp = f"{DIM}{mission_time(t0)}{RST}"
    pad = " " * 12

    if gps is None:
        gps_txt = f"{YELLOW}GPS ○ no data   {RST}"
    elif gps["fix"] > 0:
        alt = f"{gps['alt']:7.1f} m" if gps["alt"] is not None else "    —   "
        gps_txt = f"{GREEN}GPS ● FIX {gps['used']:2d} sat{RST}  alt {alt}"
    else:
        gps_txt = f"{YELLOW}GPS ○ SEARCH {gps['in_view']:2d} in view{RST}   "

    if bmp is None:
        baro_txt = f"{RED}BARO no data{RST}"
        temp_txt = ""
    else:
        baro_txt = f"BARO {bmp['p'] / 100:8.2f} hPa  rel {bmp['rel']:+6.2f} m"
        temp_txt = f"in {bmp['t']:5.1f} °C"
    if out_c is not None:
        temp_txt += f"  out {out_c:5.1f} °C"

    print(f"{tp}  {gps_txt} │ {baro_txt} │ {temp_txt}")

    if imu is None:
        imu_txt = f"{RED}IMU no data{RST}"
    else:
        moving = max(abs(imu["gx"]), abs(imu["gy"]), abs(imu["gz"])) > 15
        gcol = CYAN if moving else ""
        imu_txt = (f"IMU acc {imu['ax']:+6.2f} {imu['ay']:+6.2f} {imu['az']:+6.2f} g │ "
                   f"{gcol}gyro {imu['gx']:+7.1f} {imu['gy']:+7.1f} {imu['gz']:+7.1f} °/s{RST}")
    mode = f"{GREEN}AIRBORNE<4g ✓{RST}" if airborne else f"{YELLOW}AIRBORNE<4g ?{RST}"
    print(f"{pad}{imu_txt} │ {mode}")


def main():
    stop_after = None
    if "--for" in sys.argv:
        stop_after = float(sys.argv[sys.argv.index("--for") + 1])

    banner()
    t0 = time.time()
    fd = None
    buf = b""
    gps = imu = bmp = out_c = None
    airborne = False
    lost_shown = False

    try:
        while stop_after is None or time.time() - t0 < stop_after:
            if fd is None:
                fd = open_port()
                if fd is None:
                    if not lost_shown:
                        print(f"{RED}{mission_time(t0)}  LINK LOST — waiting for the board…{RST}")
                        lost_shown = True
                    time.sleep(0.3)
                    continue
                buf = b""
                if lost_shown:
                    print(f"{GREEN}{mission_time(t0)}  LINK UP{RST}")
                    lost_shown = False

            ready, _, _ = select.select([fd], [], [], 0.2)
            if not ready:
                continue
            try:
                chunk = os.read(fd, 1024)
            except OSError:
                os.close(fd)
                fd = None
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue

                if line.startswith("=== hab_bringup"):
                    print(f"{MAGENTA}{mission_time(t0)}  BOARD RESTART — boot sequence{RST}")
                    continue
                for rx, text, ok in BOOT_PATTERNS:
                    m = rx.search(line)
                    if m:
                        mark = f"{GREEN}✓{RST}" if ok(m) else f"{RED}✗{RST}"
                        print(f"{DIM}{mission_time(t0)}  BOOT   {text(m)}{RST} {mark}")
                        if "AIRBORNE" in text(m):
                            airborne = ok(m)
                        break

                m = RE_GPS.search(line)
                if m:
                    gps = {"fix": int(m.group(2)), "used": int(m.group(3)), "in_view": int(m.group(4)),
                           "alt": float(m.group(5)) if m.group(5) else None}
                    if m.group(6) == "8":
                        airborne = True
                    continue
                m = RE_IMU.search(line)
                if m:
                    v = [float(x) for x in m.groups()]
                    imu = dict(zip(["ax", "ay", "az", "gx", "gy", "gz"], v))
                    continue
                m = RE_OUT.search(line)
                if m:
                    out_c = float(m.group(1))
                    continue
                m = RE_BMP.search(line)
                if m:
                    bmp = {"t": float(m.group(1)), "p": float(m.group(2)), "rel": float(m.group(4))}
                    show_block(t0, gps, imu, bmp, airborne, out_c)  # BMP line closes each 1 s tick
                    gps = imu = bmp = out_c = None
    except KeyboardInterrupt:
        pass
    finally:
        print(RST + DIM + "─" * 96 + RST)
        if fd is not None:
            os.close(fd)


if __name__ == "__main__":
    main()
