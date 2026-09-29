#!/usr/bin/env python3
"""Lift test of the flight state machine (HW-18), runs unattended on the laptop.

    python3 tools/lift_test.py        # board on USB, then ride as below; log: lift_test_<time>.log

Route: down to the ground floor and wait 30 s -> up home, wait 30 s -> down again and
wait 2 min at the bottom -> come back. Stops of the lift on the way do not matter.

1. Waits until the board is 40+ m below the start and has stood still for 15 s
   (lift arrived at the ground floor) -> sends 'T' (lift test mode, ground re-zeroed).
2. On the way up, once the state is ASCENT and h > 35 m -> sends 'H' (hang on purpose,
   the watchdog resets the board) to check that the flight resumes after a reset.
3. Logs everything; after LANDED + 60 s (or 25 min) sends 'T' again to leave test mode.
"""
import glob, os, re, select, termios, time

LOG = time.strftime("lift_test_%Y%m%d_%H%M%S.log")
RE_REL = re.compile(r"rel=(-?[\d.]+)")
RE_FLT = re.compile(r"FLT st=(\w+) h=(-?[\d.]+) vz=(-?[\d.]+) asl=(-?[\d.]+)")


def open_port():
    p = glob.glob("/dev/cu.usbmodem*")
    if not p:
        return None
    fd = os.open(p[0], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    return fd


def main():
    out = open(LOG, "w", buffering=1)
    t0 = time.time()
    def say(s):
        text = f"{time.time() - t0:7.1f} {s}"
        out.write(text + "\n")
        if not s.startswith("   st="):
            print(text, flush=True)  # events also on the screen
    fd = None
    buf = b""
    import sys
    phase = sys.argv[1] if len(sys.argv) > 1 else "wait_bottom"
    rel_hist = []
    landed_at = None
    say("lift test started: go down to the ground floor and wait there")
    while time.time() - t0 < 25 * 60:
        if fd is None:
            fd = open_port()
            if fd is None:
                time.sleep(0.5)
                continue
        r, _, _ = select.select([fd], [], [], 0.2)
        if not r:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        except OSError:
            os.close(fd)
            fd = None
            say("USB link lost")
            continue
        buf += chunk
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            now = time.time()
            if not line.startswith(("FRM", "LOOP", "GPS: UTC", "IMU:", "T=")):
                say(line)
            if not line.startswith("FRM"):
                continue
            m, f = RE_REL.search(line), RE_FLT.search(line)
            if not m or not f:
                continue
            rel = float(m.group(1))
            st, h, vz = f.group(1), float(f.group(2)), float(f.group(3))
            seq = int(line.split("seq=")[1].split()[0])
            if seq % 25 == 0:
                say(f"   st={st:<9} h={h:+7.1f} vz={vz:+5.1f} rel={rel:+7.1f} ({phase})")
            if phase == "wait_bottom":
                # the bottom = 50+ m below the start and 20 s without moving; a lift stopping
                # on the way to pick people up is higher up and shorter
                rel_hist = [(t, v) for t, v in rel_hist if now - t < 20] + [(now, rel)]
                span = max(v for _, v in rel_hist) - min(v for _, v in rel_hist)
                if rel < -50 and now - rel_hist[0][0] > 19 and span < 2.0:
                    os.write(fd, b"T")
                    phase = "armed"
                    say(">>> at the bottom: sent T (lift test mode ON). Ride up now.")
            elif phase == "armed":
                if st == "ASCENT" and h > 35:
                    os.write(fd, b"H")
                    phase = "after_reset"
                    say(">>> ASCENT above 35 m: sent H (hang -> watchdog reset). Keep riding up.")
            elif phase == "after_reset":
                if st == "LANDED" and landed_at is None:
                    landed_at = now
                    say(">>> LANDED seen")
                if landed_at and now - landed_at > 60:
                    os.write(fd, b"T")
                    say(">>> done: sent T (test mode off)")
                    time.sleep(1.5)
                    out.close()
                    return
    say("timeout (25 min)")
    if fd is not None:
        os.write(fd, b"T")


if __name__ == "__main__":
    main()
