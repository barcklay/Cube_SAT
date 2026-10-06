#!/usr/bin/env python3
"""Create the starting PCB (hab1_flight.kicad_pcb) from the schematic's netlist.

Run with KiCad's own Python (it has the pcbnew module):

    /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli sch export netlist \
        --format kicadsexpr -o /tmp/hab1.net hardware/flight_pcb/hab1_flight.kicad_sch
    /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
        hardware/flight_pcb/gen_pcb.py /tmp/hab1.net

What it does: CubeSat 1U outline, 4 copper layers, JLCPCB-friendly rules, every
footprint loaded from the KiCad library with the nets of the schematic (ratsnest),
linked to its schematic symbol (so "Update PCB from Schematic" keeps working),
placed in functional zones, GND plane on In1 and +3V3 plane on In2, and outlines of
the plug-in module bodies on User.Drawings so nothing is placed under them by mistake.
Tracks are NOT routed here.

It refuses to overwrite a board that already has tracks, unless --force is given: the
tracks from route_pcb.py are regenerated anyway (never use --force on hand routing).
"""

import os
import re
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "hab1_flight.kicad_pcb")
FPLIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"

BOARD_W, BOARD_H = 95.89, 90.17   # CubeSat 1U board (PC/104-style), mm
OX, OY = 100.0, 100.0             # board top-left corner on the canvas, mm

# ---------------------------------------------------------------------------------
# Placement: ref -> (x, y, rotation) in board coordinates (mm from the top-left corner)
# ---------------------------------------------------------------------------------
PLACE = {
    # mounting holes: 4 mm from each corner (CHECK against the CubeSat/PC/104 drawing)
    "H1": (4.0, 4.0, 0), "H2": (BOARD_W - 4.0, 4.0, 0),
    "H3": (4.0, BOARD_H - 4.0, 0), "H4": (BOARD_W - 4.0, BOARD_H - 4.0, 0),
    # power, top-left
    "J1": (10.0, 13.0, 90), "J2": (10.0, 22.0, 90),
    "Q1": (19.0, 13.0, 0), "R1": (19.0, 17.5, 0),
    # regulator in SOT-223: pins on the left (GND, OUT, IN top to bottom), the big tab (OUT) on the right
    "U2": (28.0, 14.0, 0), "C1": (23.5, 19.5, 0), "C2": (34.5, 14.0, 90),
    "R2": (19.0, 22.0, 0), "R3": (23.0, 22.0, 0),
    "F1": (18.0, 7.5, 0),      # battery fuse, right after the battery connector
    # MCU in the middle. Its pads are 5.7 mm from the centre; every VDD pin sits at a corner
    # (16, 32, 48, 64), so one 100 nF goes into each corner, 2 mm from its pin.
    "U1": (48.0, 45.0, 0),
    "C5": (42.0, 39.0, 225), "C6": (54.0, 39.0, 135), "C7": (42.0, 51.0, 315), "C8": (54.0, 51.0, 45),
    "C9": (39.6, 41.3, 270),    # VBAT pin 1
    "C15": (39.6, 44.3, 270),   # NRST pin 7
    "C4": (37.0, 47.0, 90),    # battery-sense filter at PA0 (pin 12), not at the divider
    "C10": (45.5, 57.0, 0),
    # VDDA filter: 100 nF right under VSSA/VREF+/VDDA (pins 27..29), then 1 uF, then the bead
    "C12": (50.5, 53.4, 180), "C11": (50.5, 55.2, 180), "FB1": (53.2, 54.4, 90),
    "R13": (45.0, 35.5, 90),   # BOOT0 (PB8, pin 61) pull-down
    # programming connector + reset at the bottom edge
    "J3": (48.0, 80.0, 0), "SW1": (63.0, 82.0, 0), "C13": (56.0, 76.0, 0),
    # pull-ups next to the MCU
    "R4": (35.0, 60.0, 90), "R5": (37.0, 60.0, 90), "R6": (39.0, 60.0, 90),
    "R7": (33.0, 45.0, 90), "R8": (33.0, 49.0, 90), "R9": (64.0, 52.0, 90),
    "R22": (66.5, 55.0, 90), "R23": (68.5, 55.0, 90),
    # LEDs at the top edge (visible)
    "R10": (40.0, 9.0, 0), "D1": (40.0, 5.0, 0), "R11": (46.0, 9.0, 0), "D2": (46.0, 5.0, 0),
    # module sockets
    "J7": (68.5, 19.0, 270),   # GPS: body goes from the pin row up to the top edge, antenna cable leaves there
    "J9": (90.0, 22.0, 0),     # LoRa: right edge, far from the GPS antenna side
    "C14": (86.0, 20.0, 90),
    "J5": (8.0, 56.0, 0),      # IMU
    "J6": (37.5, 22.0, 90),    # barometer: free on both sides of the row (which side its body takes is unknown)
    "J4": (8.0, 44.7, 180),    # SPI flash, pin 1 at the bottom: its body must go inwards, not over the edge
    "J8": (14.0, 85.0, 90),    # DS18B20 probe connector at the edge (probe goes outside)
    "J10": (74.0, 86.0, 90),   # OLED (bench only)
    "J12": (89.5, 56.0, 0),    # expansion header: right edge, under the LoRa adapter
    "R16": (85.5, 63.62, 0), "R17": (85.5, 66.16, 0), "R18": (85.5, 68.70, 0),
    "R19": (85.5, 71.24, 0), "R20": (85.5, 73.78, 0), "R21": (85.5, 76.32, 0),
    "R14": (14.0, 74.0, 0), "R15": (14.0, 76.2, 0),   # DS18B20 cable protection
    # recovery buzzer: cable connector at the bottom edge, FET and parts next to it
    "J11": (30.0, 86.0, 90), "Q2": (37.0, 81.0, 0), "R12": (37.0, 77.0, 0), "D3": (41.0, 84.0, 90),
    "F2": (33.5, 78.0, 90),
    # test points along the bottom-right
    "TP1": (70.0, 60.0, 0), "TP2": (74.0, 60.0, 0), "TP3": (78.0, 60.0, 0),
    "TP4": (70.0, 64.0, 0), "TP5": (74.0, 64.0, 0), "TP6": (78.0, 64.0, 0),
    "TP7": (70.0, 68.0, 0), "TP8": (74.0, 68.0, 0), "TP9": (78.0, 68.0, 0), "TP10": (82.0, 68.0, 0),
}

# Silkscreen: what plugs in where and which way round. ref -> (name, pin names in pin order,
# side of the pin row the names go to: L/R for a vertical row, U/D for a horizontal one,
# per_pin). JST connectors (per_pin False) get one line along the connector instead.
LABELS = {
    "J4": ("FLASH", "DI CLK GND DO CS 3V3", "L", True),
    "J5": ("IMU", "VCC GND SCLK SDI SDO CS INT1 INT2", "L", True),
    "J6": ("BARO", "INT CS SDO SDI SCK VCC GND", "D", True),
    "J7": ("GPS", "VCC GND TX RX PPS", "D", True),
    "J9": ("LORA", "3V3 GND NSS SCK MOSI MISO BUSY DIO1 NRST TXEN RXEN GND", "R", True),
    "J10": ("OLED", "GND VDD SCK SDA", "U", True),
    "J12": ("EXP", "VBAT! GND 3V3 A1 A2 A3 A4 PWM1 PWM2 GND", "R", True),
    "J1": ("BATTERY", "+ GND", "L", False),
    "J2": ("SWITCH", "", "L", False),
    "J8": ("TEMP", "3V3 DQ GND", "U", False),
    "J11": ("BUZZER", "+ -", "U", False),
}
TP_NAMES = ["3V3", "GND", "VBAT", "SCL", "SDA", "GTX", "GRX", "SCK1", "SCK3", "SCK2"]

# Plug-in module boards, measured from photos of the real modules with the 2.54 mm pin pitch
# as the ruler (2026-10-06): ref -> (name, length along the pin row, depth from the row,
# side of the row the board takes when walking from pin 1 to the last pin: "R", "L" or
# "both" when unknown, measured or guessed). The pin row sits ~1.2 mm inside one edge.
# The LoRa adapter is not bought yet: its rectangle is a guess.
MODULES = {
    "J4": ("FLASH", 15.5, 14.0, "R", "measured"),
    "J5": ("IMU GY-601N1", 20.1, 18.1, "L", "measured"),
    "J6": ("BARO BMP390", 22.0, 11.0, "both", "measured, side unknown"),
    "J7": ("GPS MAX-M10S", 13.5, 16.0, "R", "measured"),
    "J9": ("LoRa E22 adapter", 34.0, 21.0, "R", "GUESS"),
}
ROW_INSET = 1.2
# parts tall enough to hit a module board that sits ~8.5 mm up on its socket
TALL = ["J1", "J2", "J8", "J11", "J3", "SW1", "J10", "J12"]


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def parse_netlist(path):
    t = open(path).read()
    comps = {}
    tc = t[t.index("(components"):t.index("(libparts") if "(libparts" in t else t.index("(nets")]
    for body in re.split(r"\n\t\t\(comp\n", tc)[1:]:
        ref = re.search(r'\(ref "([^"]+)"\)', body).group(1)
        val = re.search(r'\(value "([^"]*)"\)', body)
        fp = re.search(r'\(footprint "([^"]*)"\)', body)
        ts = [x for x in re.findall(r'\(tstamps "([^"]*)"\)', body) if x != "/"]
        comps[ref] = dict(value=val.group(1) if val else "", footprint=fp.group(1) if fp else "",
                          tstamp=ts[-1] if ts else "")
    nets = {}
    tn = t[t.index("(nets"):]
    for blk in re.split(r"\n\t\t\(net\n", tn)[1:]:
        name = re.search(r'\(name "([^"]*)"\)', blk).group(1)
        nets[name] = re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)', blk)
    return comps, nets


def main():
    force = "--force" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--force"]
    if len(args) != 1:
        sys.exit(__doc__)
    sys.argv[1:] = args
    if os.path.exists(OUT):
        old = pcbnew.LoadBoard(OUT)
        if len(old.GetTracks()) > 0 and not force:
            sys.exit("hab1_flight.kicad_pcb already has tracks: not overwriting manual work")

    comps, nets = parse_netlist(sys.argv[1])
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(4)

    # JLCPCB 4-layer standard process is fine with these
    ds = board.GetDesignSettings()
    ds.m_TrackMinWidth = mm(0.15)
    ds.m_MinClearance = mm(0.15)
    ds.m_ViasMinSize = mm(0.45)
    ds.m_MinThroughDrill = mm(0.3)
    ds.m_CopperEdgeClearance = mm(0.3)

    # outline
    corners = [(0, 0), (BOARD_W, 0), (BOARD_W, BOARD_H), (0, BOARD_H)]
    for (x1, y1), (x2, y2) in zip(corners, corners[1:] + corners[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pt(x1, y1))
        seg.SetEnd(pt(x2, y2))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(mm(0.1))
        board.Add(seg)

    # nets
    netinfo = {}
    for name in nets:
        ni = pcbnew.NETINFO_ITEM(board, name)  # incl. "unconnected-(...)" for no-connect pins
        board.Add(ni)
        netinfo[name] = ni
    pad_net = {}
    for name, nodes in nets.items():
        for ref, pin in nodes:
            pad_net[(ref, pin)] = name

    # footprints
    missing = []
    for ref, c in sorted(comps.items()):
        lib, name = c["footprint"].split(":")
        fp = pcbnew.FootprintLoad(os.path.join(FPLIB, lib + ".pretty"), name)
        if fp is None:
            missing.append(c["footprint"])
            continue
        fp.SetFPID(pcbnew.LIB_ID(lib, name))  # keep the library name, like the schematic
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["tstamp"]))
        x, y, rot = PLACE.get(ref, (BOARD_W + 10, 10 + 5 * len(missing), 0))
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        if ref.startswith("H"):
            fp.Reference().SetVisible(False)  # hole labels would be cut by the board edge
        board.Add(fp)
        for pad in fp.Pads():
            net = pad_net.get((ref, pad.GetNumber()))
            if net in netinfo:
                pad.SetNet(netinfo[net])
    if missing:
        sys.exit("missing footprints: %s" % missing)

    # module body outlines on User.Drawings, computed from the real pad positions, and a check
    # that no module board hangs over the edge or lands on another module or a tall part
    def body_rect(ref):
        name, length, depth, side, _ = MODULES[ref]
        pads = sorted(board.FindFootprintByReference(ref).Pads(), key=lambda p: int(p.GetNumber()))
        x1, y1 = pcbnew.ToMM(pads[0].GetPosition().x) - OX, pcbnew.ToMM(pads[0].GetPosition().y) - OY
        x2, y2 = pcbnew.ToMM(pads[-1].GetPosition().x) - OX, pcbnew.ToMM(pads[-1].GetPosition().y) - OY
        row = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        dx, dy = (x2 - x1) / row, (y2 - y1) / row
        rx, ry = -dy, dx                      # to the right of the walk from pin 1 to pin N
        over = (length - row) / 2
        lo = -(depth - ROW_INSET) if side in ("L", "both") else -ROW_INSET
        hi = (depth - ROW_INSET) if side in ("R", "both") else ROW_INSET
        pts = [(x1 - dx * over + rx * t, y1 - dy * over + ry * t) for t in (lo, hi)] + \
              [(x2 + dx * over + rx * t, y2 + dy * over + ry * t) for t in (lo, hi)]
        xs, ys = [q[0] for q in pts], [q[1] for q in pts]
        return min(xs), min(ys), max(xs), max(ys)

    def overlap(a, b):
        return min(a[2], b[2]) - max(a[0], b[0]) > 0.2 and min(a[3], b[3]) - max(a[1], b[1]) > 0.2

    bodies = {ref: body_rect(ref) for ref in MODULES}
    problems = []
    for ref, (x0, y0, x1, y1) in bodies.items():
        name = MODULES[ref][0]
        rect = pcbnew.PCB_SHAPE(board)
        rect.SetShape(pcbnew.SHAPE_T_RECT)
        rect.SetStart(pt(x0, y0))
        rect.SetEnd(pt(x1, y1))
        rect.SetLayer(pcbnew.Dwgs_User)
        rect.SetWidth(mm(0.15))
        board.Add(rect)
        txt = pcbnew.PCB_TEXT(board)
        txt.SetText("%s board (%s)" % (name, MODULES[ref][4]))
        txt.SetPosition(pt((x0 + x1) / 2, (y0 + y1) / 2))
        txt.SetLayer(pcbnew.Dwgs_User)
        txt.SetTextSize(pcbnew.VECTOR2I(mm(0.9), mm(0.9)))
        board.Add(txt)
        if x0 < 0 or y0 < 0 or x1 > BOARD_W or y1 > BOARD_H:
            problems.append("%s hangs over the board edge" % name)
        for other, r2 in bodies.items():
            if other > ref and overlap((x0, y0, x1, y1), r2):
                problems.append("%s overlaps %s" % (name, MODULES[other][0]))
        for t in TALL:
            bb = board.FindFootprintByReference(t).GetBoundingBox(False, False)
            r2 = (pcbnew.ToMM(bb.GetLeft()) - OX, pcbnew.ToMM(bb.GetTop()) - OY,
                  pcbnew.ToMM(bb.GetRight()) - OX, pcbnew.ToMM(bb.GetBottom()) - OY)
            if overlap((x0, y0, x1, y1), r2):
                problems.append("%s sits over the tall part %s" % (name, t))
        for h in ("H1", "H2", "H3", "H4"):
            q = board.FindFootprintByReference(h).GetPosition()
            hx, hy = pcbnew.ToMM(q.x) - OX, pcbnew.ToMM(q.y) - OY
            if overlap((x0, y0, x1, y1), (hx - 3.5, hy - 3.5, hx + 3.5, hy + 3.5)):
                problems.append("%s covers the mounting hole %s" % (name, h))
        print("  %-18s x %5.1f..%5.1f  y %5.1f..%5.1f" % (name, x0, x1, y0, y1))
    print("module fit: " + ("OK, no overlaps" if not problems else "; ".join(problems)))

    # silkscreen: module names, pin names, pin-1 marks, test point nets
    def silk(text, x, y, size=0.8, angle=0, just=0):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetPosition(pcbnew.VECTOR2I(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
        t.SetTextThickness(mm(size * 0.16))
        t.SetTextAngleDegrees(angle)
        t.SetHorizJustify((pcbnew.GR_TEXT_H_ALIGN_CENTER, pcbnew.GR_TEXT_H_ALIGN_LEFT,
                           pcbnew.GR_TEXT_H_ALIGN_RIGHT)[just])
        board.Add(t)

    for ref, (name, pins, side, per_pin) in LABELS.items():
        fp = board.FindFootprintByReference(ref)
        fp.Reference().SetVisible(False)  # the name printed next to it says more than "J6"
        pads = sorted(fp.Pads(), key=lambda p: int(p.GetNumber()))
        pos = [p.GetPosition() for p in pads]
        names = pins.split()
        vertical = abs(pos[-1].y - pos[0].y) > abs(pos[-1].x - pos[0].x)
        sign = -1 if side in "LU" else 1
        if per_pin:
            for q, pin_name in zip(pos, names):
                if vertical:    # names read left to right, next to each pin
                    silk(pin_name, q.x + sign * mm(1.9), q.y, 0.8, 0, 2 if sign < 0 else 1)
                else:           # names read bottom to top, above or below each pin
                    silk(pin_name, q.x, q.y + sign * mm(1.9), 0.8, 90, 1 if sign < 0 else 2)
            # module name beyond pin 1, where the row begins
            dx, dy = pos[0].x - pos[1].x, pos[0].y - pos[1].y
            k = mm(3.8) / (abs(dx) + abs(dy))
            silk(name, pos[0].x + int(dx * k), pos[0].y + int(dy * k), 1.0)
        else:
            # one line along the connector: "NAME: a b" in the physical order of the pins
            order = sorted(zip(pos, names + [""] * len(pos)),
                           key=lambda t: -t[0].y if vertical else t[0].x)
            text = name + (": " + " ".join(n for _, n in order) if names else "")
            cx = sum(q.x for q in pos) // len(pos)
            cy = sum(q.y for q in pos) // len(pos)
            if vertical:
                silk(text, cx + sign * mm(4.6), cy, 0.8, 90)
            else:
                silk(text, cx, cy + sign * mm(4.6), 0.8)
    for i, tp_name in enumerate(TP_NAMES, 1):
        q = board.FindFootprintByReference("TP%d" % i).GetPosition()
        board.FindFootprintByReference("TP%d" % i).Reference().SetVisible(False)
        silk(tp_name, q.x, q.y + mm(1.85), 0.8)

    # silkscreen title
    title = pcbnew.PCB_TEXT(board)
    title.SetText("HAB-1 flight board rev A  2026")
    title.SetPosition(pt(52.0, BOARD_H - 2.5))  # between the buzzer (J11) and OLED (J10) sockets
    title.SetLayer(pcbnew.F_SilkS)
    title.SetTextSize(pcbnew.VECTOR2I(mm(1.2), mm(1.2)))
    board.Add(title)

    # planes: GND on In1, +3V3 on In2 (whole board, 0.5 mm from the edge)
    for layer, net in ((pcbnew.In1_Cu, "GND"), (pcbnew.In2_Cu, "+3V3")):
        zone = pcbnew.ZONE(board)
        zone.SetLayer(layer)
        zone.SetNet(netinfo[net])
        outline = zone.Outline()
        outline.NewOutline()
        for x, y in ((0.5, 0.5), (BOARD_W - 0.5, 0.5), (BOARD_W - 0.5, BOARD_H - 0.5), (0.5, BOARD_H - 0.5)):
            outline.Append(mm(OX + x), mm(OY + y))
        zone.SetMinThickness(mm(0.25))
        zone.SetLocalClearance(mm(0.3))
        board.Add(zone)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    pcbnew.SaveBoard(OUT, board)
    print("wrote %s: %d footprints, %d nets" % (os.path.basename(OUT), len(comps),
          len([n for n in netinfo if not n.startswith("unconnected-")])))


main()
