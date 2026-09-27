#!/usr/bin/env python3
"""Generate the HAB-1 flight board schematic (KiCad) from SPEC.md.

Every connection is made with net labels / power symbols placed exactly on the
pin ends (no wires), so the netlist is easy to review and to change: edit the
tables below and run

    python3 hardware/flight_pcb/gen_schematic.py

Then check it with KiCad's ERC:

    /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli sch erc \
        hardware/flight_pcb/hab1_flight.kicad_sch

Symbols are copied from the KiCad standard library into the schematic, so the
file opens on any machine.
"""

import json
import os
import re
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = "hab1_flight"
SYMLIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"

# --------------------------------------------------------------------------------------
# 1. Microcontroller pin -> net (from hab_bringup.ioc, plus the NEW pins of SPEC.md)
# --------------------------------------------------------------------------------------
MCU_NETS = {
    "PA13": "SWDIO", "PA14": "SWCLK", "PB3": "SWO",
    "PG10": "NRST",                                   # pin 7 = PG10-NRST (named "PG10" in KiCad)
    "PA2": "VCP_TX", "PA3": "VCP_RX",                 # LPUART1 console
    "PB8": "I2C_SCL", "PB9": "I2C_SDA",               # barometer, OLED
    "PA5": "SPI1_SCK", "PA6": "SPI1_MISO", "PA7": "SPI1_MOSI", "PB6": "FLASH_CS",
    "PC10": "SPI3_SCK", "PC11": "SPI3_MISO", "PC12": "SPI3_MOSI", "PC7": "IMU_CS",
    "PB2": "IMU_INT1",
    "PC4": "USART1_TX", "PC5": "USART1_RX",           # GPS
    "PA10": "DS18B20_DQ",
    "PB13": "SPI2_SCK", "PB14": "SPI2_MISO", "PB15": "SPI2_MOSI", "PB12": "LORA_NSS",
    "PC6": "LORA_BUSY", "PC8": "LORA_DIO1", "PC9": "LORA_NRST",
    "PB10": "LORA_TXEN", "PB11": "LORA_RXEN",
    "PA0": "VBAT_SENSE", "PB0": "LED_ALIVE", "PB1": "LED_FIX",
}
MCU_POWER = {"VDD": "+3V3", "VBAT": "+3V3", "VSS": "GND", "VSSA": "GND",
             "VDDA": "+3V3A", "VREF+": "+3V3A"}

# --------------------------------------------------------------------------------------
# 2. Parts: (ref, lib_id, value, footprint, {pin number: net})
#    Module header pin orders marked "CONFIRM" must be checked against the real modules.
# --------------------------------------------------------------------------------------
R0603 = "Resistor_SMD:R_0603_1608Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
SOCK = "Connector_PinSocket_2.54mm:PinSocket_1x{n:02d}_P2.54mm_Vertical"
XH = "Connector_JST:JST_XH_B{n}B-XH-A_1x{n:02d}_P2.50mm_Vertical"

PARTS = []  # filled by the helpers below


def part(ref, lib_id, value, footprint, pins, block, note=""):
    PARTS.append(dict(ref=ref, lib_id=lib_id, value=value, footprint=footprint,
                      pins=pins, block=block, note=note))


def r(ref, value, a, b, block):
    part(ref, "Device:R", value, R0603, {"1": a, "2": b}, block)


def c(ref, value, a, b, block, fp=C0603):
    part(ref, "Device:C", value, fp, {"1": a, "2": b}, block)


# Power: battery -> external switch -> reverse-polarity P-FET -> LDO 3.3 V
part("J1", "Connector_Generic:Conn_01x02", "BATTERY 4xL91", XH.format(n=2),
     {"1": "VBAT_RAW", "2": "GND"}, "power")
part("J2", "Connector_Generic:Conn_01x02", "POWER SWITCH (on the box)", XH.format(n=2),
     {"1": "VBAT_RAW", "2": "VBAT_SW"}, "power")
part("Q1", "Transistor_FET:AO3401A", "AO3401A", "Package_TO_SOT_SMD:SOT-23",
     {"1": "Q1_GATE", "2": "VBAT_PROT", "3": "VBAT_SW"}, "power",
     "reverse-polarity protection: body diode drain->source")
r("R1", "10k", "Q1_GATE", "GND", "power")
part("U2", "Regulator_Linear:MIC5219-3.3YM5", "MIC5219-3.3", None,
     {"1": "VBAT_PROT", "2": "GND", "3": "VBAT_PROT", "4": "U2_BYP", "5": "+3V3"}, "power",
     "provisional, decide after HW-21")
c("C1", "10u", "VBAT_PROT", "GND", "power", C0805)
c("C2", "10u", "+3V3", "GND", "power", C0805)
c("C3", "470p", "U2_BYP", "GND", "power")
# Battery voltage divider: 7.2 V fresh -> 2.3 V at the ADC
r("R2", "100k", "VBAT_PROT", "VBAT_SENSE", "power")
r("R3", "47k", "VBAT_SENSE", "GND", "power")
c("C4", "100n", "VBAT_SENSE", "GND", "power")

# MCU support: decoupling, VDDA filter, reset
for i in range(5, 10):
    c(f"C{i}", "100n", "+3V3", "GND", "mcu")
c("C10", "4.7u", "+3V3", "GND", "mcu", C0805)
part("FB1", "Device:FerriteBead_Small", "600R@100MHz", "Inductor_SMD:L_0603_1608Metric",
     {"1": "+3V3", "2": "+3V3A"}, "mcu")
c("C11", "1u", "+3V3A", "GND", "mcu")
c("C12", "100n", "+3V3A", "GND", "mcu")
c("C13", "100n", "NRST", "GND", "mcu")
part("SW1", "Switch:SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_TL3342",
     {"1": "NRST", "2": "GND"}, "mcu")

# Programming + console (STDC14, same as the Nucleo's ST-LINK V3 connector)
part("J3", "Connector:Conn_ST_STDC14", "STDC14 SWD+VCP",
     "Connector_PinHeader_1.27mm:PinHeader_2x07_P1.27mm_Vertical_SMD",
     {"1": None, "2": None, "3": "+3V3", "4": "SWDIO", "5": "GND", "6": "SWCLK", "7": "GND",
      "8": "SWO", "9": None, "10": None, "11": "GND", "12": "NRST",
      "13": "VCP_TX", "14": "VCP_RX"}, "mcu",
     "pin 13 VCP_RX (probe input) <- MCU TX; pin 14 VCP_TX -> MCU RX")

# Pull-ups learned on the bench
r("R4", "4.7k", "I2C_SCL", "+3V3", "pullups")
r("R5", "4.7k", "I2C_SDA", "+3V3", "pullups")
r("R6", "4.7k", "DS18B20_DQ", "+3V3", "pullups")
r("R7", "10k", "FLASH_CS", "+3V3", "pullups")
r("R8", "10k", "IMU_CS", "+3V3", "pullups")
r("R9", "10k", "LORA_NSS", "+3V3", "pullups")

# LEDs
r("R10", "1k", "LED_ALIVE", "LED1_A", "leds")
part("D1", "Device:LED", "ALIVE green", "LED_SMD:LED_0603_1608Metric",
     {"1": "GND", "2": "LED1_A"}, "leds")
r("R11", "1k", "LED_FIX", "LED2_A", "leds")
part("D2", "Device:LED", "GPS FIX blue", "LED_SMD:LED_0603_1608Metric",
     {"1": "GND", "2": "LED2_A"}, "leds")

# Module headers (female sockets on the board)
part("J4", "Connector_Generic:Conn_01x06", "SPI FLASH module", SOCK.format(n=6),
     {"1": "SPI1_MOSI", "2": "SPI1_SCK", "3": "GND", "4": "SPI1_MISO", "5": "FLASH_CS",
      "6": "+3V3"}, "modules", "order DI CLK GND DO CS 3V3 as on the bench module")
part("J5", "Connector_Generic:Conn_01x08", "IMU GY-601N1", SOCK.format(n=8),
     {"1": "+3V3", "2": "GND", "3": "SPI3_SCK", "4": "SPI3_MOSI", "5": "SPI3_MISO",
      "6": "IMU_CS", "7": "IMU_INT1", "8": None}, "modules",
     "order VCC GND SCLK SDI SDO CS INT1 INT2 as on the GY-601N1 silkscreen")
part("J6", "Connector_Generic:Conn_01x06", "BAROMETER BMP390", SOCK.format(n=6),
     {"1": "+3V3", "2": "GND", "3": "I2C_SCL", "4": "I2C_SDA", "5": "+3V3", "6": "+3V3"},
     "modules", "CONFIRM order: VCC GND SCK SDI SDO(=high, addr 0x77) CSB(=high, I2C)")
part("J7", "Connector_Generic:Conn_01x04", "GPS MAX-M10S", SOCK.format(n=4),
     {"1": "+3V3", "2": "GND", "3": "USART1_RX", "4": "USART1_TX"}, "modules",
     "CONFIRM order: VCC GND TX(module) RX(module)")
part("J8", "Connector_Generic:Conn_01x03", "DS18B20 probe", XH.format(n=3),
     {"1": "+3V3", "2": "DS18B20_DQ", "3": "GND"}, "modules", "red / yellow / black")
part("J9", "Connector_Generic:Conn_01x12", "LoRa E22-400M22S (adapter)", SOCK.format(n=12),
     {"1": "+3V3", "2": "GND", "3": "LORA_NSS", "4": "SPI2_SCK", "5": "SPI2_MOSI",
      "6": "SPI2_MISO", "7": "LORA_BUSY", "8": "LORA_DIO1", "9": "LORA_NRST",
      "10": "LORA_TXEN", "11": "LORA_RXEN", "12": "GND"}, "modules",
     "rev A: E22 on a header until HW-6 is done")
c("C14", "47u", "+3V3", "GND", "modules", C1206)
part("J10", "Connector_Generic:Conn_01x04", "OLED SSD1306 (bench)", SOCK.format(n=4),
     {"1": "GND", "2": "+3V3", "3": "I2C_SCL", "4": "I2C_SDA"}, "modules",
     "CONFIRM order GND VCC SCL SDA")

# Test points for the oscilloscope / logic analyzer
for i, net in enumerate(["+3V3", "GND", "VBAT_PROT", "I2C_SCL", "I2C_SDA",
                         "USART1_TX", "USART1_RX", "SPI1_SCK", "SPI3_SCK", "SPI2_SCK"], 1):
    part(f"TP{i}", "Connector:TestPoint", net, "TestPoint:TestPoint_Pad_D1.5mm",
         {"1": net}, "test")

# Mounting holes (CubeSat 1U board)
for i in range(1, 5):
    part(f"H{i}", "Mechanical:MountingHole", "M3", "MountingHole:MountingHole_3.2mm_M3",
         {}, "mech")

POWER_NETS = {"+3V3": "power:+3V3", "GND": "power:GND"}
PWR_FLAG_NETS = ["GND", "VBAT_PROT", "+3V3A", "VBAT_RAW"]

# --------------------------------------------------------------------------------------
# S-expression helpers
# --------------------------------------------------------------------------------------


def tokenize(text):
    return re.findall(r'"(?:\\.|[^"\\])*"|\(|\)|[^\s()]+', text)


def parse(tokens, i=0):
    out = []
    while i < len(tokens):
        t = tokens[i]
        if t == "(":
            sub, i = parse(tokens, i + 1)
            out.append(sub)
        elif t == ")":
            return out, i + 1
        else:
            out.append(t)
            i += 1
    return out, i


def dump(node, indent=0):
    if not isinstance(node, list):
        return node
    if all(not isinstance(x, list) for x in node):
        return "(" + " ".join(node) + ")"
    pad = "\t" * (indent + 1)
    parts = [dump(x, indent + 1) for x in node]
    head = []
    rest = []
    for p, x in zip(parts, node):
        (rest if (isinstance(x, list) or rest) else head).append(p)
    return "(" + " ".join(head) + "".join("\n" + pad + p for p in rest) + "\n" + "\t" * indent + ")"


def q(s):
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def uq(s):
    return s[1:-1] if s.startswith('"') else s


def U():
    return str(uuid.uuid4())


_lib_cache = {}


def lib_symbols(lib):
    if lib not in _lib_cache:
        tree, _ = parse(tokenize(open(os.path.join(SYMLIB, lib + ".kicad_sym")).read()))
        root = tree[0]
        _lib_cache[lib] = {uq(n[1]): n for n in root if isinstance(n, list) and n[0] == "symbol"}
    return _lib_cache[lib]


def resolve(lib_id):
    """Return the flattened library symbol (extends resolved), named 'Lib:Name'."""
    lib, name = lib_id.split(":")
    syms = lib_symbols(lib)
    node = syms[name]
    ext = next((n for n in node if isinstance(n, list) and n[0] == "extends"), None)
    if ext:
        parent = resolve(lib + ":" + uq(ext[1]))
        pname = uq(parent[1]).split(":")[1]
        props = {uq(n[1]): n for n in node if isinstance(n, list) and n[0] == "property"}
        flat = [parent[0], q(lib_id)]
        for n in parent[2:]:
            if isinstance(n, list) and n[0] == "property" and uq(n[1]) in props:
                flat.append(props.pop(uq(n[1])))
            elif isinstance(n, list) and n[0] == "symbol":
                flat.append(["symbol", q(uq(n[1]).replace(pname, name, 1))] + n[2:])
            else:
                flat.append(n)
        flat += list(props.values())
        return flat
    return [node[0], q(lib_id)] + node[2:]


def sym_pins(flat):
    """Pins of unit 1 / body style 1: {number: (x, y, angle, name, type)} in lib coords."""
    pins = {}
    for sub in flat:
        if not (isinstance(sub, list) and sub[0] == "symbol"):
            continue
        m = re.search(r"_(\d+)_(\d+)$", uq(sub[1]))
        if not m or m.group(1) not in ("0", "1") or m.group(2) not in ("0", "1"):
            continue
        for p in sub:
            if isinstance(p, list) and p[0] == "pin":
                at = next(x for x in p if isinstance(x, list) and x[0] == "at")
                nm = next(x for x in p if isinstance(x, list) and x[0] == "name")
                nb = next(x for x in p if isinstance(x, list) and x[0] == "number")
                pins[uq(nb[1])] = (float(at[1]), float(at[2]), int(float(at[3])), uq(nm[1]), p[1])
    return pins


def fp_default(flat):
    for n in flat:
        if isinstance(n, list) and n[0] == "property" and uq(n[1]) == "Footprint":
            return uq(n[2])
    return ""


# --------------------------------------------------------------------------------------
# Schematic builder
# --------------------------------------------------------------------------------------
ROOT = U()
items = []
lib_used = {}


def snap(v, grid=2.54):
    return round(round(v / grid) * grid, 2)


def fmt(v):
    return ("%.2f" % v).rstrip("0").rstrip(".")


def effects(size=1.27, justify=None, hide=False):
    e = ["effects", ["font", ["size", fmt(size), fmt(size)]]]
    if justify:
        e.append(["justify"] + justify)
    if hide:
        e.append(["hide", "yes"])
    return e


def place_symbol(lib_id, ref, value, x, y, footprint=None, rot=0, extra_props=()):
    flat = resolve(lib_id)
    lib_used[lib_id] = flat
    pins = sym_pins(flat)
    fp = footprint if footprint is not None else fp_default(flat)
    node = ["symbol", ["lib_id", q(lib_id)], ["at", fmt(x), fmt(y), str(rot)], ["unit", "1"],
            ["exclude_from_sim", "no"], ["in_bom", "yes"], ["on_board", "yes"], ["dnp", "no"],
            ["uuid", q(U())],
            ["property", q("Reference"), q(ref), ["at", fmt(x + 2.54), fmt(y - 5.08), "0"],
             effects(justify=["left"], hide=ref.startswith("#"))],
            ["property", q("Value"), q(value), ["at", fmt(x + 2.54), fmt(y + 5.08), "0"],
             effects(justify=["left"])],
            ["property", q("Footprint"), q(fp), ["at", fmt(x), fmt(y), "0"], effects(hide=True)],
            ["property", q("Datasheet"), q(""), ["at", fmt(x), fmt(y), "0"], effects(hide=True)]]
    for k, v in extra_props:
        node.append(["property", q(k), q(v), ["at", fmt(x), fmt(y), "0"], effects(hide=True)])
    for num in pins:
        node.append(["pin", q(num), ["uuid", q(U())]])
    node.append(["instances", ["project", q(PROJECT),
                               ["path", q("/" + ROOT), ["reference", q(ref)], ["unit", "1"]]]])
    items.append(node)
    abs_pins = {}
    for num, (px, py, ang, name, typ) in pins.items():
        # rotation of the placed symbol (0/90/180/270), lib Y axis points up
        ax, ay, aang = px, py, ang
        for _ in range(rot // 90):
            ax, ay, aang = -ay, ax, (aang + 90) % 360
        abs_pins[num] = (round(x + ax, 2), round(y - ay, 2), aang, name, typ)
    return abs_pins


pwr_count = [0]


def power_at(net, x, y, ang):
    pwr_count[0] += 1
    # draw the symbol pointing away from the pin body
    rot = {"power:GND": {0: 90, 90: 0, 180: 270, 270: 180}}
    lib = POWER_NETS[net]
    if lib == "power:GND":
        r_ = {0: 270, 90: 0, 180: 90, 270: 180}[ang]
    else:
        r_ = {0: 90, 90: 180, 180: 270, 270: 0}[ang]
    place_symbol(lib, "#PWR%03d" % pwr_count[0], net, x, y, footprint="", rot=r_)


def label_at(net, x, y, ang):
    # pin angle points from the connection point into the body -> text goes the other way
    la = {0: 180, 180: 0, 90: 270, 270: 90}[ang]
    just = ["right", "bottom"] if la in (180, 270) else ["left", "bottom"]
    items.append(["label", q(net), ["at", fmt(x), fmt(y), str(la)],
                  ["fields_autoplaced", "yes"], effects(justify=just), ["uuid", q(U())]])


def no_connect(x, y):
    items.append(["no_connect", ["at", fmt(x), fmt(y)], ["uuid", q(U())]])


POINTS = {}


def connect(net, x, y, ang):
    key = (round(x, 2), round(y, 2))
    tag = net if net is not None else "<no-connect>"
    if key in POINTS and POINTS[key] != tag:
        raise SystemExit(f"COLLISION at {key}: {POINTS[key]} vs {tag}")
    POINTS[key] = tag
    if net is None:
        no_connect(x, y)
    elif net in POWER_NETS:
        power_at(net, x, y, ang)
    else:
        label_at(net, x, y, ang)


def text(s, x, y, size=2.5):
    items.append(["text", q(s), ["exclude_from_sim", "no"], ["at", fmt(x), fmt(y), "0"],
                  effects(size=size, justify=["left", "bottom"]), ["uuid", q(U())]])


# --- MCU ---------------------------------------------------------------------------
text("MCU  STM32G474RET6 (same chip and pins as the NUCLEO-G474RE bench)", 60, 40, 3)
mcu = place_symbol("MCU_ST_STM32G4:STM32G474RETx", "U1", "STM32G474RET6", snap(130), snap(150))
done = set()
for num, (x, y, ang, name, typ) in sorted(mcu.items()):
    # a pin name can carry several functions: "PB8-BOOT0", "PG10-NRST", "PC14-OSC32_IN"
    net = next((MCU_NETS[x] for x in [name] + name.split("-") if x in MCU_NETS), None) \
        or MCU_POWER.get(name)
    if (x, y) in done:
        continue  # stacked power pins share one connection point
    done.add((x, y))
    connect(net, x, y, ang)

# --- other parts: flow down columns right of the MCU, never overlapping ---------------
BLOCK_TITLE = {"power": "POWER  battery -> switch -> reverse-polarity FET -> 3.3 V LDO",
               "mcu": "MCU SUPPORT  decoupling, VDDA filter, reset, STDC14",
               "pullups": "PULL-UPS (learned on the bench)",
               "leds": "LEDS", "modules": "MODULE HEADERS (breakouts plug in)",
               "test": "TEST POINTS", "mech": "MOUNTING HOLES"}
COL_X0, COL_W, Y_TOP, Y_BOTTOM = 240.0, 80.0, 55.0, 560.0
cx, cy = COL_X0, Y_TOP
last_block = None
for p in PARTS:
    big = p["lib_id"].startswith(("Connector_Generic", "Connector:Conn_ST", "Regulator"))
    h = 12.0 + 2.54 * max(4, len(p["pins"]) + 2) if big else 22.0
    need = h + (12 if p["block"] != last_block else 0)
    if cy + need > Y_BOTTOM:
        cx, cy = cx + COL_W, Y_TOP
    if p["block"] != last_block:
        text(BLOCK_TITLE[p["block"]], cx, cy, 2.2)
        cy += 12
        last_block = p["block"]
    px, py = snap(cx + 12), snap(cy + h / 2)
    pins = place_symbol(p["lib_id"], p["ref"], p["value"], px, py, footprint=p["footprint"],
                        extra_props=[("Note", p["note"])] if p["note"] else ())
    for num, (x, y, ang, name, typ) in pins.items():
        connect(p["pins"].get(num), x, y, ang)
    if p["note"]:
        text(p["note"], snap(cx + 32), py, 1.3)
    cy += h

# PWR_FLAGs: tell ERC which nets are really powered (battery, filtered VDDA, ground)
fx, fy = snap(60), snap(330)
text("POWER FLAGS (for ERC)", fx, fy - 10, 2)
for i, net in enumerate(PWR_FLAG_NETS):
    pins = place_symbol("power:PWR_FLAG", "#FLG%02d" % (i + 1), "PWR_FLAG", snap(fx + i * 25.4), fy,
                        footprint="")
    (x, y, ang, _, _), = pins.values()
    connect(net, x, y, ang)

# --------------------------------------------------------------------------------------
# Write files
# --------------------------------------------------------------------------------------
sch = ["kicad_sch", ["version", "20231120"], ["generator", q("gen_schematic.py")],
       ["generator_version", q("1.0")], ["uuid", q(ROOT)], ["paper", q("A1")],
       ["title_block", ["title", q("HAB-1 flight board")], ["date", q("2026-09-27")],
        ["rev", q("A-draft")], ["company", q("Cube SAT / hab_bringup")],
        ["comment", "1", q("Generated from hardware/flight_pcb/SPEC.md by gen_schematic.py")]],
       ["lib_symbols"] + list(lib_used.values())]
sch += items
sch.append(["sheet_instances", ["path", q("/"), ["page", q("1")]]])

with open(os.path.join(HERE, PROJECT + ".kicad_sch"), "w") as f:
    f.write(dump(sch) + "\n")
pro = os.path.join(HERE, PROJECT + ".kicad_pro")
if not os.path.exists(pro):
    with open(pro, "w") as f:
        json.dump({"meta": {"filename": PROJECT + ".kicad_pro", "version": 1}}, f, indent=2)
print(f"wrote {PROJECT}.kicad_sch: {len(PARTS) + 1} parts, {pwr_count[0]} power symbols")
