#!/usr/bin/env python3
"""HAB-1 ground station (HW-28): radio packets -> table on disk -> point on a map.

The receiver board (firmware built with `cmake --preset Ground`) prints every packet it
hears as one text line. This program reads those lines, checks and decodes the 28-byte
packet (hab_bringup/Core/Inc/telemetry_packet.h), and:

  * prints one line per packet in the terminal;
  * appends it to a CSV table and keeps the raw lines (data/ground/, never committed:
    the files hold coordinates);
  * serves a page with the map, altitude and signal level at http://127.0.0.1:8017

Usage:
    python3 tools/ground_station.py                      # live, from the receiver board
    python3 tools/ground_station.py --here 16.4,102.8    # where I stand: distance, direction, angle up
    python3 tools/ground_station.py --demo               # made-up flight, no radio needed
    python3 tools/ground_station.py --replay data/ground/gs_..._raw.log --speed 20
    python3 tools/ground_station.py --public             # do not print coordinates (for videos)

Only the Python standard library is used. Reopens the USB port by itself if the link
goes silent. The map tiles need the internet; without it the page draws the track on a
plain grid.
"""

import argparse
import datetime
import glob
import json
import math
import os
import random
import select
import struct
import sys
import termios
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ---------------------------------------------------------------- packet (telemetry_packet.h)

TP_MAGIC = 0xA1
TP_SIZE = 28
TP_FORMAT = "<BBHHBBiiHHhbbBBH"   # 28 bytes, little-endian
STATES = ("PRELAUNCH", "ASCENT", "FLOAT", "BURST", "DESCENT", "LANDED")
STATES_RU = {"PRELAUNCH": "на земле", "ASCENT": "подъём", "FLOAT": "зависание", "BURST": "шар лопнул",
             "DESCENT": "спуск", "LANDED": "приземлился"}
FLAG_BARO_OK, FLAG_GPS_FIX, FLAG_OUT_OK, FLAG_LOG_OK, FLAG_RESUMED = 0x08, 0x10, 0x20, 0x40, 0x80


def crc16(data):
    """CRC-16/CCITT-FALSE, the same as TpCrc16 in the firmware."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode(raw):
    """bytes -> dict. Raises ValueError with the reason if the packet is not a good one."""
    if len(raw) != TP_SIZE:
        raise ValueError(f"длина {len(raw)} вместо {TP_SIZE}")
    if raw[0] != TP_MAGIC:
        raise ValueError(f"первый байт 0x{raw[0]:02X}, не наш пакет")
    if crc16(raw[:-2]) != raw[26] | raw[27] << 8:
        raise ValueError("не сошлась контрольная сумма")
    (_, pid, seq, up, sf, sats, lat, lon, gps_alt, alt, vz, out_t, board_t, vbat, _, _) = \
        struct.unpack(TP_FORMAT, raw)
    state = sf & 0x07
    return {
        "id": pid, "seq": seq, "uptime_s": up,
        "state": STATES[state] if state < len(STATES) else f"?{state}",
        "baro_ok": bool(sf & FLAG_BARO_OK), "gps_fix": bool(sf & FLAG_GPS_FIX),
        "out_ok": bool(sf & FLAG_OUT_OK), "log_ok": bool(sf & FLAG_LOG_OK),
        "resumed": bool(sf & FLAG_RESUMED), "sats": sats,
        "lat": lat / 1e7, "lon": lon / 1e7, "gps_alt_m": gps_alt, "alt_m": alt - 1000,
        "vz_mps": vz / 100.0, "out_temp_c": out_t, "board_temp_c": board_t,
        "vbat_v": round(vbat * 0.04, 2),
    }


def encode(p):
    """dict -> bytes, the same as TelemetryEncode. Used by --demo and by the tests."""
    def clamp(v, lo, hi):
        return max(lo, min(hi, int(math.floor(v + 0.5)) if v >= 0 else -int(math.floor(-v + 0.5))))
    state = STATES.index(p["state"])
    flags = ((FLAG_BARO_OK if p.get("baro_ok") else 0) | (FLAG_GPS_FIX if p.get("gps_fix") else 0) |
             (FLAG_OUT_OK if p.get("out_ok") else 0) | (FLAG_LOG_OK if p.get("log_ok") else 0) |
             (FLAG_RESUMED if p.get("resumed") else 0))
    body = struct.pack(TP_FORMAT[:-1], TP_MAGIC, p["id"], p["seq"] & 0xFFFF, min(p["uptime_s"], 65535),
                       state | flags, p["sats"], clamp(p["lat"] * 1e7, -2**31, 2**31 - 1),
                       clamp(p["lon"] * 1e7, -2**31, 2**31 - 1), clamp(p["gps_alt_m"], 0, 65535),
                       clamp(p["alt_m"] + 1000, 0, 65535), clamp(p["vz_mps"] * 100, -32768, 32767),
                       clamp(p["out_temp_c"], -128, 127), clamp(p["board_temp_c"], -128, 127),
                       clamp(p.get("vbat_v", 0) / 0.04, 0, 255), 0)
    return body + struct.pack("<H", crc16(body))


# ---------------------------------------------------------------- geometry

EARTH_R = 6371000.0


def look(here, lat, lon, alt_m):
    """From `here` (lat, lon, alt) to the payload: ground distance m, bearing deg, angle up deg."""
    la1, lo1, la2, lo2 = map(math.radians, (here[0], here[1], lat, lon))
    dla, dlo = la2 - la1, lo2 - lo1
    a = math.sin(dla / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin(dlo / 2) ** 2
    ang = 2 * math.asin(min(1.0, math.sqrt(a)))          # angle at the Earth's centre
    dist = EARTH_R * ang
    bearing = math.degrees(math.atan2(math.sin(dlo) * math.cos(la2),
                                      math.cos(la1) * math.sin(la2) - math.sin(la1) * math.cos(la2) * math.cos(dlo)))
    # angle above the horizon with the Earth's curvature taken into account
    r1, r2 = EARTH_R + here[2], EARTH_R + alt_m
    elev = math.degrees(math.atan2(r2 * math.cos(ang) - r1, r2 * math.sin(ang))) if ang > 1e-9 else 90.0
    return dist, bearing % 360.0, elev


# ---------------------------------------------------------------- where the lines come from

def open_port(path, attempt=0):
    """With no --port given, every reopen tries the next USB serial port: the payload board
    on the same laptop is also a /dev/cu.usbmodem*, and only the receiver prints "GS:" lines."""
    ports = [path] if path else sorted(glob.glob("/dev/cu.usbmodem*"))
    if not ports:
        return None, None
    name = ports[attempt % len(ports)]
    try:
        fd = os.open(name, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return None, name
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd, name


def serial_lines(path, silence_s=12.0):
    """Lines from the receiver board. The board prints "GS: alive" every 5 s, so a longer
    time without a receiver line means the USB link went stale (the port stays open but nothing
    comes) or this port is another board: reopen, trying the next port."""
    fd, buf, last, warned, attempt = None, b"", time.time(), False, 0
    while True:
        if fd is None:
            fd, name = open_port(path, attempt)
            attempt += 1
            if fd is None:
                if not warned:
                    yield None, "нет платы-приёмника на USB, жду…"
                    warned = True
                time.sleep(1.0)
                continue
            buf, last, warned = b"", time.time(), False
            yield None, f"порт {name} открыт, слушаю"
        try:
            ready, _, _ = select.select([fd], [], [], 0.5)
            chunk = os.read(fd, 4096) if ready else b""
            if ready and not chunk:
                raise OSError("port closed")
        except OSError:
            chunk = None
        if chunk is None or time.time() - last > silence_s:
            try:
                os.close(fd)
            except OSError:
                pass
            fd = None
            yield None, f"\a!!! ПРИЁМНИК МОЛЧИТ на {name} — проверь USB, переоткрываю порт"
            time.sleep(1.0)
            continue
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                if text.startswith(("GS:", "RX", "===")):
                    last = time.time()
                    yield time.time(), text


def replay_lines(path, speed):
    """Lines of a raw log written earlier by this program ("<iso time> <line>")."""
    first = None
    for row in open(path, encoding="utf-8"):
        stamp, _, line = row.rstrip("\n").partition(" ")
        try:
            t = datetime.datetime.fromisoformat(stamp).timestamp()
        except ValueError:
            continue
        if first is None:
            first, start = t, time.time()
        if speed > 0:
            time.sleep(max(0.0, (t - first) / speed - (time.time() - start)))
        yield t, line
    yield None, "запись закончилась"


def demo_lines(speed, period_s=5.0):
    """A made-up flight from a made-up place: up at 5 m/s to 30 km, burst, down under a
    parachute, landed. Goes through the very same decoding as real packets."""
    lat, lon, alt, t, seq, state = 16.4000, 102.8000, 0.0, 0.0, 0, "PRELAUNCH"
    start_wall, rng, burst_at = time.time(), random.Random(1), 30000.0
    yield start_wall, "GS: listening on 434500000 Hz SF9 BW125"
    while True:
        if state == "PRELAUNCH":
            vz = 0.0
            if t > 60:
                state = "ASCENT"
        elif state == "ASCENT":
            vz = 5.0 + rng.uniform(-0.4, 0.4)
            if alt >= burst_at:
                state = "BURST"
        elif state == "BURST":
            vz, state = -40.0, "DESCENT"
        elif state == "DESCENT":
            vz = -6.0 * math.sqrt(math.exp(alt / 7000.0))   # thin air up high: faster
            if alt <= 0:
                alt, vz, state = 0.0, 0.0, "LANDED"
        else:
            vz = 0.0
        alt = max(0.0, alt + vz * period_s)
        wind = 2.0 + 18.0 * math.exp(-((alt - 12000.0) / 5000.0) ** 2)   # jet stream near 12 km
        if state in ("ASCENT", "BURST", "DESCENT"):
            lon += wind * period_s / (111320.0 * math.cos(math.radians(lat)))
            lat += 0.25 * wind * period_s / 111320.0
        temp = max(-60.0, 30.0 - 6.5 * alt / 1000.0) if alt < 20000 else -60.0 + (alt - 20000) / 1000.0
        dist_km = math.hypot((lat - 16.4) * 111.32, (lon - 102.8) * 107.0, alt / 1000.0)
        rssi = int(-45 - 20 * math.log10(max(0.01, dist_km) * 1000) + 22 + rng.uniform(-3, 3))
        packet = encode({"id": 1, "seq": seq, "uptime_s": int(t), "state": state, "baro_ok": True,
                         "gps_fix": True, "out_ok": True, "log_ok": True, "sats": 12 + rng.randint(-2, 3),
                         "lat": lat, "lon": lon, "gps_alt_m": 170 + alt, "alt_m": alt, "vz_mps": vz,
                         "out_temp_c": temp, "board_temp_c": max(-20.0, 25.0 - alt / 1500.0),
                         "vbat_v": 6.4 - t / 20000.0})
        wall = start_wall + t
        roll = rng.random()
        if roll < 0.05 or (state == "LANDED" and roll < 0.6):
            pass                                               # lost on the air
        elif roll < 0.07:
            yield wall, f"RXBAD t={int(t * 1000)} rssi={rssi} snr=-9"
        else:
            yield wall, f"RX t={int(t * 1000)} len=28 rssi={rssi} snr={max(-12, min(12, (rssi + 125) // 4))} data={packet.hex().upper()}"
        if seq % 6 == 5:
            yield wall, f"GS: alive t={int(t * 1000)} rx={seq} bad=0 noise=-112"
        seq += 1
        t += period_s
        if state == "LANDED" and t > 60 and alt == 0 and seq % 40 == 0:
            yield None, "демо-полёт закончен (страница остаётся открытой, Ctrl+C — выход)"
            return
        if speed > 0:
            time.sleep(period_s / speed)


# ---------------------------------------------------------------- the station itself

CSV_COLUMNS = ("pc_time", "gs_ms", "id", "seq", "lost_before", "uptime_s", "state", "baro_ok", "gps_fix",
               "out_ok", "log_ok", "resumed", "sats", "lat", "lon", "gps_alt_m", "alt_m", "vz_mps",
               "out_temp_c", "board_temp_c", "vbat_v", "rssi_dbm", "snr_db", "dist_m", "bearing_deg",
               "elev_deg")


class Station:
    def __init__(self, here, out_dir, tag, public, live=True):
        self.here, self.public = here, public
        self.live, self.clock = live, 0.0   # demo and replay run on the recorded time, not on the wall clock
        self.here_from_first_fix = here is None
        self.lock = threading.Lock()
        self.points = []            # every good packet, as sent to the page
        self.last = {}              # payload id -> (seq, uptime_s)
        self.stats = {"good": 0, "lost": 0, "bad_radio": 0, "bad_packet": 0}
        self.receiver = {"noise": None, "alive": None, "note": ""}
        self.csv = self.raw = None
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M%S")
            self.csv_path = os.path.join(out_dir, f"gs_{stamp}{tag}.csv")
            self.csv = open(self.csv_path, "w", encoding="utf-8")
            self.csv.write(",".join(CSV_COLUMNS) + "\n")
            self.raw = open(os.path.join(out_dir, f"gs_{stamp}{tag}_raw.log"), "w", encoding="utf-8")

    def say(self, text):
        print(f"{time.strftime('%H:%M:%S')}  {text}", flush=True)

    def feed(self, when, line):
        if when is None:            # a note from the source itself, not from the board
            self.receiver["note"] = line.strip("\a! ")
            self.say(line)
            return
        if not line:
            return
        if self.raw:
            self.raw.write(f"{datetime.datetime.fromtimestamp(when).isoformat(timespec='milliseconds')} {line}\n")
            self.raw.flush()
        self.receiver["alive"] = self.clock = when
        self.receiver["note"] = ""
        fields = dict(part.split("=", 1) for part in line.split() if "=" in part)
        if line.startswith("RX "):
            self.packet(when, fields)
        elif line.startswith("RXBAD"):
            self.stats["bad_radio"] += 1
            self.say(f"пакет пришёл повреждённым (сигнал {fields.get('rssi', '?')} дБм)")
        elif line.startswith("GS: alive"):
            try:
                self.receiver["noise"] = int(fields["noise"])
            except (KeyError, ValueError):
                pass
        elif line.startswith("GS:") or line.startswith("LORA:") or line.startswith("==="):
            self.say(f"приёмник: {line}")

    def packet(self, when, fields):
        try:
            p = decode(bytes.fromhex(fields["data"]))
            rssi, snr = int(fields["rssi"]), int(fields["snr"])
        except (KeyError, ValueError) as err:
            self.stats["bad_packet"] += 1
            self.say(f"чужой или битый пакет: {err}")
            return
        lost, note = 0, ""
        if p["id"] in self.last:
            prev_seq, prev_up = self.last[p["id"]]
            if p["uptime_s"] + 30 < prev_up:
                note = "  ← ЗОНД ПЕРЕЗАГРУЗИЛСЯ"
            else:
                lost = (p["seq"] - prev_seq - 1) & 0xFFFF
                if lost > 5000:     # an old packet again, not 60000 lost ones
                    lost = 0
        self.last[p["id"]] = (p["seq"], p["uptime_s"])
        if self.here is None and p["gps_fix"]:
            self.here = (p["lat"], p["lon"], p["gps_alt_m"])
        dist = bearing = elev = None
        if self.here and p["gps_fix"]:
            dist, bearing, elev = look(self.here, p["lat"], p["lon"], p["gps_alt_m"])
        p.update(t=round(when, 2), gs_ms=int(fields.get("t", 0)), rssi=rssi, snr=snr, lost=lost,
                 dist_m=None if dist is None else round(dist), bearing=None if bearing is None else round(bearing, 1),
                 elev=None if elev is None else round(elev, 1))
        with self.lock:
            self.points.append(p)
            self.stats["good"] += 1
            self.stats["lost"] += lost
        if self.csv:
            row = (datetime.datetime.fromtimestamp(when).isoformat(timespec="seconds"), p["gs_ms"], p["id"],
                   p["seq"], lost, p["uptime_s"], p["state"], *(int(p[k]) for k in
                   ("baro_ok", "gps_fix", "out_ok", "log_ok", "resumed")), p["sats"], f"{p['lat']:.7f}",
                   f"{p['lon']:.7f}", p["gps_alt_m"], p["alt_m"], p["vz_mps"], p["out_temp_c"],
                   p["board_temp_c"], p["vbat_v"], rssi, snr, *("" if v is None else v for v in
                   (p["dist_m"], p["bearing"], p["elev"])))
            self.csv.write(",".join(str(v) for v in row) + "\n")
            self.csv.flush()
        arrow = "↑" if p["vz_mps"] > 0.3 else "↓" if p["vz_mps"] < -0.3 else "·"
        text = (f"зонд {p['id']} №{p['seq']:<5} {STATES_RU.get(p['state'], p['state']):<11} "
                f"высота {p['alt_m']:>6} м {arrow}{abs(p['vz_mps']):4.1f} м/с  спутников {p['sats']:>2}  "
                f"снаружи {p['out_temp_c']:>4}°  сигнал {rssi} дБм")
        if not p["gps_fix"]:
            text += "  НЕТ GPS: координаты старые"
        elif dist is not None and not self.here_from_first_fix:
            text += f"  до зонда {dist / 1000:.1f} км, направление {bearing:.0f}°, вверх {elev:.0f}°"
        elif dist is not None:
            text += f"  от старта {dist / 1000:.1f} км"
        if p["gps_fix"] and not self.public:
            text += f"  [{p['lat']:.5f}, {p['lon']:.5f}]"
        if lost:
            text += f"  (потеряно {lost})"
        self.say(text + note)

    def state_json(self, since):
        with self.lock:
            pts = self.points[since:]
            body = {"points": pts, "total": len(self.points), "stats": dict(self.stats),
                    "receiver": dict(self.receiver), "now": time.time() if self.live else self.clock, "here": self.here,
                    "here_is_launch": self.here_from_first_fix}
        return json.dumps(body).encode()


# ---------------------------------------------------------------- the page

PAGE = r"""<!doctype html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>HAB-1 · наземная станция</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
  :root { --bg:#0c1014; --panel:#141b22; --line:#25303b; --text:#e6edf3; --dim:#8494a3;
          --ok:#4cc38a; --warn:#f2b84b; --bad:#f2665c; --a:#5ab0f7; --b:#f59e5b; }
  * { box-sizing:border-box; }
  html, body { height:100%; margin:0; background:var(--bg); color:var(--text);
               font:14px/1.4 ui-sans-serif, system-ui, -apple-system, "Segoe UI", sans-serif; }
  body { display:grid; grid-template-columns:minmax(0,1fr) 380px; grid-template-rows:auto minmax(0,1fr); gap:10px; padding:10px; }
  header { grid-column:1/-1; display:flex; flex-wrap:wrap; gap:8px 18px; align-items:baseline; }
  h1 { font-size:15px; margin:0; letter-spacing:.06em; text-transform:uppercase; }
  #link { font-variant-numeric:tabular-nums; }
  .dot { display:inline-block; width:9px; height:9px; border-radius:50%; background:var(--dim); margin-right:6px; }
  #mapwrap { position:relative; min-height:320px; border:1px solid var(--line); border-radius:8px; overflow:hidden; }
  #map, #grid { position:absolute; inset:0; width:100%; height:100%; }
  #grid { display:none; background:#0f151b; }
  aside { display:flex; flex-direction:column; gap:10px; overflow:auto; }
  .card { background:var(--panel); border:1px solid var(--line); border-radius:8px; padding:10px 12px; }
  .card h2 { margin:0 0 6px; font-size:11px; font-weight:600; letter-spacing:.08em; text-transform:uppercase; color:var(--dim); }
  .big { font-size:30px; font-weight:650; font-variant-numeric:tabular-nums; line-height:1.1; }
  .row { display:flex; justify-content:space-between; gap:10px; padding:2px 0; font-variant-numeric:tabular-nums; }
  .row span:first-child { color:var(--dim); }
  .state { display:inline-block; padding:2px 8px; border-radius:99px; background:var(--line); font-weight:600; }
  .warn { color:var(--warn); } .bad { color:var(--bad); } .ok { color:var(--ok); }
  canvas.chart { width:100%; height:120px; display:block; }
  a { color:var(--a); } button { font:inherit; color:var(--text); background:var(--line); border:0; border-radius:6px; padding:3px 9px; cursor:pointer; }
  .coords { font-family:ui-monospace, Menlo, monospace; user-select:all; }
  @media (max-width:820px) { body { grid-template-columns:minmax(0,1fr); grid-template-rows:auto 45vh auto; height:auto; } }
</style></head><body>
<header><h1>HAB-1 · наземная станция</h1>
  <div id="link"><span class="dot" id="dot"></span><span id="linktext">жду данные…</span></div>
  <div id="recv" style="color:var(--dim)"></div></header>
<div id="mapwrap"><div id="map"></div><canvas id="grid"></canvas></div>
<aside>
  <div class="card"><h2>Зонд</h2>
    <div><span class="state" id="state">—</span> <span id="flags"></span></div>
    <div class="big"><span id="alt">—</span> <small style="font-size:14px;color:var(--dim)">м над стартом</small></div>
    <div class="row"><span>Скорость по вертикали</span><span id="vz">—</span></div>
    <div class="row"><span>Высота по GPS</span><span id="gpsalt">—</span></div>
    <div class="row"><span>Спутников</span><span id="sats">—</span></div>
    <div class="row"><span>Снаружи / на плате</span><span id="temp">—</span></div>
    <div class="row"><span>Батарея</span><span id="vbat">—</span></div>
    <div class="row"><span>Работает</span><span id="uptime">—</span></div></div>
  <div class="card"><h2>Где искать</h2>
    <div class="row"><span>Координаты</span><span class="coords" id="coords">—</span></div>
    <div class="row"><span id="distlabel">Расстояние</span><span id="dist">—</span></div>
    <div class="row"><span>Направление / угол вверх</span><span id="aim">—</span></div>
    <div class="row"><span><button id="copy">Скопировать</button></span><a id="gmaps" target="_blank" rel="noopener">Открыть в Google Maps</a></div></div>
  <div class="card"><h2>Радио</h2>
    <div class="row"><span>Сигнал / запас над шумом</span><span id="rssi">—</span></div>
    <div class="row"><span>Шум эфира</span><span id="noise">—</span></div>
    <div class="row"><span>Принято / потеряно / битых</span><span id="count">—</span></div></div>
  <div class="card"><h2>Высота, м</h2><canvas class="chart" id="c_alt"></canvas></div>
  <div class="card"><h2>Сигнал, дБм</h2><canvas class="chart" id="c_rssi"></canvas></div>
</aside>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const RU = {PRELAUNCH:'на земле', ASCENT:'подъём', FLOAT:'зависание', BURST:'шар лопнул', DESCENT:'спуск', LANDED:'приземлился'};
const pts = []; let info = null, follow = true;
const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const fmt = (v, d=0) => v == null ? '—' : v.toLocaleString('ru-RU', {minimumFractionDigits:d, maximumFractionDigits:d});

let map = null, line = null, marker = null, hereMarker = null;
if (window.L) {
  map = L.map('map', {zoomControl:true}).setView([15, 101], 6);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {maxZoom:19, attribution:'© OpenStreetMap'}).addTo(map);
  line = L.polyline([], {color:'#5ab0f7', weight:3}).addTo(map);
  map.on('dragstart', () => { follow = false; });
} else { $('map').style.display = 'none'; $('grid').style.display = 'block'; }

function sizeCanvas(c) { const r = c.getBoundingClientRect(), k = devicePixelRatio || 1;
  c.width = Math.max(10, r.width * k); c.height = Math.max(10, r.height * k);
  const g = c.getContext('2d'); g.setTransform(k, 0, 0, k, 0, 0); return [g, r.width, r.height]; }

function chart(id, key, colour) {
  const [g, w, h] = sizeCanvas($(id)); g.clearRect(0, 0, w, h);
  if (pts.length < 2) return;
  const t0 = pts[0].t, t1 = pts[pts.length - 1].t; let lo = Infinity, hi = -Infinity;
  for (const p of pts) { lo = Math.min(lo, p[key]); hi = Math.max(hi, p[key]); }
  if (hi - lo < 1) { hi += 1; lo -= 1; }
  const L0 = 46, B = 16, x = t => L0 + (w - L0 - 4) * (t - t0) / Math.max(1, t1 - t0), y = v => 12 + (h - B - 16) * (1 - (v - lo) / (hi - lo));
  g.font = '11px ui-sans-serif, system-ui'; g.fillStyle = css('--dim'); g.strokeStyle = css('--line'); g.lineWidth = 1;
  for (const v of [lo, (lo + hi) / 2, hi]) { g.beginPath(); g.moveTo(L0, y(v)); g.lineTo(w - 4, y(v)); g.stroke(); g.textAlign = 'right'; g.fillText(fmt(v), L0 - 6, y(v) + 4); }
  g.textAlign = 'left'; g.fillText('0', L0, h - 3); g.textAlign = 'right'; g.fillText(Math.round((t1 - t0) / 60) + ' мин', w - 4, h - 3);
  g.strokeStyle = colour; g.lineWidth = 1.6; g.beginPath();
  pts.forEach((p, i) => i ? g.lineTo(x(p.t), y(p[key])) : g.moveTo(x(p.t), y(p[key]))); g.stroke();
}

function grid() {   // the track on a plain grid, when there is no internet for the map
  const [g, w, h] = sizeCanvas($('grid')); g.clearRect(0, 0, w, h);
  const fix = pts.filter(p => p.gps_fix); g.fillStyle = css('--dim'); g.font = '12px ui-sans-serif, system-ui';
  g.fillText('Нет интернета для карты — трек на сетке, север сверху', 12, 20);
  if (!fix.length) return;
  const lat0 = info.here ? info.here[0] : fix[0].lat, lon0 = info.here ? info.here[1] : fix[0].lon, kx = 111.32 * Math.cos(lat0 * Math.PI / 180);
  const xy = fix.map(p => [(p.lon - lon0) * kx, (p.lat - lat0) * 111.32]); xy.push([0, 0]);
  let r = 0.5; for (const [a, b] of xy) r = Math.max(r, Math.abs(a), Math.abs(b));
  const s = (Math.min(w, h) / 2 - 30) / r, X = a => w / 2 + a * s, Y = b => h / 2 - b * s;
  const step = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200].find(v => v * s > 45) || 500;
  g.strokeStyle = css('--line'); g.lineWidth = 1;
  for (let k = step; k < r * 1.5; k += step) { g.beginPath(); g.arc(w / 2, h / 2, k * s, 0, 7); g.stroke(); g.fillText(fmt(k, k < 1 ? 1 : 0) + ' км', w / 2 + 4, h / 2 - k * s - 3); }
  g.beginPath(); g.moveTo(0, h / 2); g.lineTo(w, h / 2); g.moveTo(w / 2, 0); g.lineTo(w / 2, h); g.stroke();
  g.strokeStyle = css('--a'); g.lineWidth = 2; g.beginPath(); xy.slice(0, -1).forEach(([a, b], i) => i ? g.lineTo(X(a), Y(b)) : g.moveTo(X(a), Y(b))); g.stroke();
  const [la, lb] = xy[xy.length - 2]; g.fillStyle = css('--b'); g.beginPath(); g.arc(X(la), Y(lb), 6, 0, 7); g.fill();
  g.fillStyle = css('--ok'); g.beginPath(); g.arc(w / 2, h / 2, 5, 0, 7); g.fill();
}

function show() {
  const now = info.now, age = info.receiver.alive ? now - info.receiver.alive : null, last = pts[pts.length - 1];
  $('recv').textContent = info.receiver.note || (age == null ? 'приёмник молчит' : age > 12 ? `приёмник молчит ${Math.round(age)} с` : 'приёмник на связи');
  const s = info.stats; $('count').textContent = `${s.good} / ${s.lost} / ${s.bad_radio + s.bad_packet}`;
  $('noise').textContent = info.receiver.noise == null ? '—' : info.receiver.noise + ' дБм';
  if (!last) { $('dot').style.background = css('--dim'); $('linktext').textContent = 'пакетов ещё не было'; return; }
  const quiet = now - last.t, colour = quiet < 20 ? '--ok' : quiet < 90 ? '--warn' : '--bad';
  $('dot').style.background = css(colour);
  $('linktext').textContent = `последний пакет ${quiet < 100 ? Math.round(quiet) + ' с' : Math.round(quiet / 60) + ' мин'} назад · №${last.seq}`;
  $('state').textContent = RU[last.state] || last.state;
  const f = []; if (!last.gps_fix) f.push('нет GPS'); if (!last.baro_ok) f.push('нет барометра'); if (!last.out_ok) f.push('нет наружного термометра');
  if (!last.log_ok) f.push('чёрный ящик не пишет'); if (last.resumed) f.push('была перезагрузка');
  $('flags').innerHTML = f.map(x => `<span class="warn">${x}</span>`).join(' · ');
  $('alt').textContent = fmt(last.alt_m); $('vz').textContent = (last.vz_mps > 0 ? '+' : '') + fmt(last.vz_mps, 1) + ' м/с';
  $('gpsalt').textContent = fmt(last.gps_alt_m) + ' м'; $('sats').textContent = last.sats;
  $('temp').textContent = `${last.out_temp_c}° / ${last.board_temp_c}°`; $('vbat').textContent = last.vbat_v ? fmt(last.vbat_v, 2) + ' В' : 'не измеряется';
  $('uptime').textContent = Math.floor(last.uptime_s / 60) + ' мин ' + (last.uptime_s % 60) + ' с';
  $('rssi').textContent = `${last.rssi} дБм / ${last.snr > 0 ? '+' : ''}${last.snr} дБ`;
  const fix = pts.filter(p => p.gps_fix), g = fix[fix.length - 1];
  if (g) {
    $('coords').textContent = g.lat.toFixed(6) + ', ' + g.lon.toFixed(6);
    $('gmaps').href = `https://www.google.com/maps?q=${g.lat.toFixed(6)},${g.lon.toFixed(6)}`;
    $('distlabel').textContent = info.here_is_launch ? 'От точки старта' : 'От меня';
    $('dist').textContent = g.dist_m == null ? '—' : fmt(g.dist_m / 1000, 1) + ' км';
    $('aim').textContent = g.bearing == null ? '—' : `${fmt(g.bearing)}° / ${fmt(g.elev)}°`;
    if (map) {
      line.setLatLngs(fix.map(p => [p.lat, p.lon]));
      if (!marker) marker = L.circleMarker([g.lat, g.lon], {radius:8, color:'#f59e5b', fillOpacity:.9}).addTo(map);
      marker.setLatLng([g.lat, g.lon]);
      if (info.here && !hereMarker) hereMarker = L.circleMarker([info.here[0], info.here[1]], {radius:6, color:'#4cc38a', fillOpacity:.9}).addTo(map);
      if (follow) { hereMarker ? map.fitBounds(L.latLngBounds([marker.getLatLng(), hereMarker.getLatLng()]).pad(0.3), {maxZoom:15}) : map.setView([g.lat, g.lon], Math.max(map.getZoom(), 12)); }
    }
  }
  if (!map) grid();
  chart('c_alt', 'alt_m', css('--a')); chart('c_rssi', 'rssi', css('--b'));
}
$('copy').onclick = () => { const t = $('coords').textContent; navigator.clipboard && navigator.clipboard.writeText(t).catch(() => {}); };
async function tick() {
  try { const r = await fetch('/state.json?since=' + pts.length, {cache:'no-store'}); info = await r.json();
        if (info.total < pts.length) pts.length = 0; else pts.push(...info.points); show(); }
  catch (e) { $('dot').style.background = css('--bad'); $('linktext').textContent = 'программа на ноутбуке не отвечает'; }
  setTimeout(tick, 1000);
}
addEventListener('resize', () => info && show()); tick();
</script></body></html>
"""


def serve(station, port):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/state.json"):
                try:
                    since = max(0, int(self.path.partition("since=")[2] or 0))
                except ValueError:
                    since = 0
                body, kind = station.state_json(since), "application/json"
            elif self.path in ("/", "/index.html"):
                body, kind = PAGE.encode(), "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)   # this laptop only
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main():
    ap = argparse.ArgumentParser(description="HAB-1 ground station")
    ap.add_argument("--port", help="serial port of the receiver board (default: the first /dev/cu.usbmodem*)")
    ap.add_argument("--here", help="where I stand: LAT,LON or LAT,LON,ALT_M")
    ap.add_argument("--demo", action="store_true", help="made-up flight, no radio")
    ap.add_argument("--replay", help="raw log written earlier by this program")
    ap.add_argument("--speed", type=float, default=None, help="demo/replay: times faster than life (0 = at once)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "ground"),
                    help="folder for the table and the raw log")
    ap.add_argument("--no-save", action="store_true", help="do not write files")
    ap.add_argument("--web", type=int, default=8017, help="port of the page (0 = no page)")
    ap.add_argument("--public", action="store_true", help="do not print coordinates in the terminal")
    args = ap.parse_args()

    here = None
    if args.here:
        parts = [float(x) for x in args.here.split(",")]
        here = (parts[0], parts[1], parts[2] if len(parts) > 2 else 0.0)
    if args.demo:
        lines, tag = demo_lines(60.0 if args.speed is None else args.speed), "_demo"
    elif args.replay:
        lines, tag = replay_lines(args.replay, 1.0 if args.speed is None else args.speed), "_replay"
    else:
        lines, tag = serial_lines(args.port), ""
    save = not args.no_save and not args.replay
    station = Station(here, os.path.normpath(args.out) if save else None, tag, args.public,
                      live=not (args.demo or args.replay))
    if args.web:
        serve(station, args.web)
        station.say(f"страница: http://127.0.0.1:{args.web}")
    if station.csv:
        station.say(f"таблица: {station.csv_path}")
    try:
        for when, line in lines:
            station.feed(when, line)
        if args.web:
            while True:
                time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    s = station.stats
    print(f"\nитог: принято {s['good']}, потеряно {s['lost']}, повреждено в эфире {s['bad_radio']}, "
          f"чужих или битых {s['bad_packet']}")


if __name__ == "__main__":
    sys.exit(main())
