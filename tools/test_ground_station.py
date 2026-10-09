#!/usr/bin/env python3
"""Test of the ground station (tools/ground_station.py) without any radio.

The firmware packs the packet in C, the ground station unpacks it in Python: two separate
implementations of one format. This test builds a small C program from the real header
(hab_bringup/Core/Inc/telemetry_packet.h), lets it print packets, and checks that Python
reads the same numbers back, and the other way round.

    python3 tools/test_ground_station.py
"""

import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import ground_station as gs  # noqa: E402

C_SOURCE = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "telemetry_packet.h"
/* "enc": fields on the command line -> packet in hex.  "dec": hex -> return code and fields. */
int main(int argc, char **argv)
{
  uint8_t b[64];
  Telemetry t = {0};
  if (argc == 16 && !strcmp(argv[1], "enc"))
  {
    t.payload_id = (uint8_t)atoi(argv[2]); t.seq = (uint16_t)atoi(argv[3]); t.uptime_s = (uint32_t)atol(argv[4]);
    t.state = (uint8_t)atoi(argv[5]); t.flags = (uint8_t)atoi(argv[6]); t.gps_sats = (uint8_t)atoi(argv[7]);
    t.lat_e7 = (int32_t)atol(argv[8]); t.lon_e7 = (int32_t)atol(argv[9]); t.gps_alt_m = (float)atof(argv[10]);
    t.alt_m = (float)atof(argv[11]); t.vz_mps = (float)atof(argv[12]); t.out_temp_c = (float)atof(argv[13]);
    t.board_temp_c = (float)atof(argv[14]); t.vbat_v = (float)atof(argv[15]);
    TelemetryEncode(&t, b);
    for (unsigned i = 0; i < TP_SIZE; i++) printf("%02X", b[i]);
    printf("\n");
    return 0;
  }
  if (argc == 3 && !strcmp(argv[1], "dec"))
  {
    unsigned n = (unsigned)strlen(argv[2]) / 2;
    for (unsigned i = 0; i < n && i < sizeof(b); i++) { unsigned v; sscanf(argv[2] + 2 * i, "%2x", &v); b[i] = (uint8_t)v; }
    int rc = TelemetryDecode(b, n, &t);
    printf("%d %u %u %lu %u %u %u %ld %ld %.0f %.0f %.2f %.0f %.0f %.2f\n", rc, t.payload_id, t.seq,
           (unsigned long)t.uptime_s, t.state, t.flags, t.gps_sats, (long)t.lat_e7, (long)t.lon_e7,
           t.gps_alt_m, t.alt_m, t.vz_mps, t.out_temp_c, t.board_temp_c, t.vbat_v);
    return 0;
  }
  return 2;
}
"""

fails = 0


def check(cond, what):
    global fails
    if not cond:
        fails += 1
        print("FAIL:", what)


CASES = [   # id seq uptime state flags sats lat_e7 lon_e7 gps_alt alt vz out board vbat
    (1, 0, 0, 0, 0x48, 0, 0, 0, 0.0, 0.0, 0.0, 28.0, 31.0, 0.0),
    (2, 1234, 5400, 1, 0x78, 14, 158000000, 1025000000, 29871.4, 29702.6, 5.37, -62.4, -11.2, 6.12),
    (1, 65535, 65535, 4, 0xF8, 31, -337000000, -1511234567, 812.0, -42.0, -38.2, -3.5, 4.5, 4.9),
    (7, 9, 77, 5, 0x08, 3, 899999999, 1799999999, 70000.0, 70000.0, 400.0, 200.0, -200.0, 20.0),
]


def main():
    with tempfile.TemporaryDirectory() as tmp:
        src, exe = os.path.join(tmp, "tp.c"), os.path.join(tmp, "tp")
        open(src, "w").write(C_SOURCE)
        subprocess.run(["cc", "-O2", "-Wall", "-I", os.path.join(HERE, "..", "hab_bringup", "Core", "Inc"),
                        "-o", exe, src, "-lm"], check=True)

        for case in CASES:
            hexpkt = subprocess.run([exe, "enc", *map(str, case)], capture_output=True, text=True).stdout.strip()
            raw = bytes.fromhex(hexpkt)
            ref = subprocess.run([exe, "dec", hexpkt], capture_output=True, text=True).stdout.split()
            p = gs.decode(raw)
            name = f"case id={case[0]} seq={case[1]}"
            check(ref[0] == "0", f"{name}: C decodes its own packet")
            flags = ((gs.FLAG_BARO_OK if p["baro_ok"] else 0) | (gs.FLAG_GPS_FIX if p["gps_fix"] else 0) |
                     (gs.FLAG_OUT_OK if p["out_ok"] else 0) | (gs.FLAG_LOG_OK if p["log_ok"] else 0) |
                     (gs.FLAG_RESUMED if p["resumed"] else 0))
            got = [p["id"], p["seq"], p["uptime_s"], gs.STATES.index(p["state"]), flags, p["sats"],
                   round(p["lat"] * 1e7), round(p["lon"] * 1e7), p["gps_alt_m"], p["alt_m"], p["vz_mps"],
                   p["out_temp_c"], p["board_temp_c"], p["vbat_v"]]
            want = [float(x) for x in ref[1:]]
            check(all(math.isclose(a, b, abs_tol=0.006) for a, b in zip(got, want)),
                  f"{name}: Python reads what C reads\n   python {got}\n   c      {want}")
            # and back: Python packs the decoded values -> exactly the same bytes
            check(gs.encode(p) == raw, f"{name}: Python packs the same bytes as C")

        good = bytes.fromhex(subprocess.run([exe, "enc", *map(str, CASES[1])], capture_output=True,
                                            text=True).stdout.strip())
        for bad, why in ((good[:-1], "short"), (b"\x00" + good[1:], "magic"),
                         (good[:10] + bytes([good[10] ^ 1]) + good[11:], "one flipped bit")):
            try:
                gs.decode(bad)
                check(False, f"damaged packet accepted ({why})")
            except ValueError:
                pass

    check(gs.crc16(b"123456789") == 0x29B1, "CRC-16/CCITT-FALSE check value")

    # geometry: one degree of latitude is 111.2 km due north; straight overhead is 90 degrees up
    d, b, e = gs.look((16.0, 103.0, 0.0), 17.0, 103.0, 0.0)
    check(abs(d - 111195) < 100 and (b < 0.1 or b > 359.9) and -1.0 < e < 0.0, f"1 degree north: {d:.0f} m, {b:.1f}, {e:.2f}")
    d, b, e = gs.look((16.0, 103.0, 0.0), 16.0, 104.0, 0.0)
    check(abs(d - 106888) < 150 and abs(b - 90) < 0.5, f"1 degree east: {d:.0f} m, {b:.1f}")
    d, b, e = gs.look((16.0, 103.0, 150.0), 16.0, 103.0, 30000.0)
    check(d < 1 and e > 89.9, "straight overhead")
    d, b, e = gs.look((16.0, 103.0, 0.0), 16.0 + 30.0 / 111.195, 103.0, 30000.0)
    check(abs(e - 44.9) < 0.3, f"30 km away and 30 km up is about 45 degrees: {e:.2f}")

    # the whole chain on a made-up flight: lines -> station -> counters
    st = gs.Station(None, None, "", True)
    st.say = lambda text: None
    sent = bad = alive = 0
    for when, line in gs.demo_lines(0):
        if when is not None:
            sent += line.startswith("RX ")
            bad += line.startswith("RXBAD")
        st.feed(when, line)
    pts = st.points
    check(st.stats["good"] == sent and st.stats["bad_radio"] == bad and st.stats["bad_packet"] == 0,
          f"every demo packet decoded: {st.stats} vs sent {sent}, bad {bad}")
    check(st.stats["good"] + st.stats["lost"] == pts[-1]["seq"] - pts[0]["seq"] + 1,
          "received + lost = packets sent between the first and the last one heard")
    check(max(p["alt_m"] for p in pts) > 29000 and pts[-1]["state"] == "LANDED", "demo flight reaches 30 km and lands")
    check([p["state"] for p in pts if p["state"] != "PRELAUNCH"][0] == "ASCENT", "states come in flight order")
    check(pts[-1]["dist_m"] > 10000 and pts[-1]["elev"] < 1.0, "landed far from the launch point, on the horizon")
    st.feed(1.0, "RX t=1 len=28 rssi=-80 snr=5 data=" + "00" * 28)
    st.feed(1.0, "RX t=1 len=3 rssi=-80 snr=5 data=A1ZZ")
    check(st.stats["bad_packet"] == 2, "foreign and garbled lines are counted, not crashed on")
    body = st.state_json(len(pts) - 2)
    check(b'"total": %d' % len(pts) in body and body.count(b'"seq"') == 2, "the page gets only the new points")

    print("ground station: all checks passed" if fails == 0 else f"ground station: {fails} FAILED")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
