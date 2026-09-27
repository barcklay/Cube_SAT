#!/usr/bin/env python3
"""Flash hab_bringup.bin in 4 KB pieces, then read everything back and compare.

Long continuous st-flash writes to this board keep failing with
LIBUSB_ERROR_TIMEOUT halfway, while reads and short writes are fine
(probably the target browns out/resets during long flash programming
with all modules on the breadboard). Short writes with retries work.

Usage:
    python3 tools/flash.py path/to/hab_bringup.bin
"""

import os
import subprocess
import sys
import tempfile

BASE = 0x08000000
CHUNK = 4096  # multiple of the 2 KB flash page, so pages are erased cleanly
TRIES = 3


def st_flash(*args, timeout=60):
    try:
        r = subprocess.run(["st-flash", *args], capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout + r.stderr
    except subprocess.TimeoutExpired:
        return -1, "TIMEOUT"


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    image = open(sys.argv[1], "rb").read()
    with tempfile.TemporaryDirectory() as tmp:
        piece = os.path.join(tmp, "piece.bin")
        for off in range(0, len(image), CHUNK):
            with open(piece, "wb") as f:
                f.write(image[off:off + CHUNK])
            for attempt in range(1, TRIES + 1):
                _, out = st_flash("--connect-under-reset", "write", piece, hex(BASE + off))
                if "verified" in out:
                    break
            else:
                sys.exit(f"FAILED at 0x{BASE + off:08X} after {TRIES} tries")
            note = f" (try {attempt})" if attempt > 1 else ""
            print(f"  0x{BASE + off:08X}  {min(CHUNK, len(image) - off):5d} B  ok{note}")

        back = os.path.join(tmp, "back.bin")
        st_flash("read", back, hex(BASE), str(len(image)))
        if open(back, "rb").read() != image:
            sys.exit("VERIFY FAILED: flash content differs from the image")
    st_flash("reset")
    print(f"flashed and verified {len(image)} bytes, board reset")


if __name__ == "__main__":
    main()
