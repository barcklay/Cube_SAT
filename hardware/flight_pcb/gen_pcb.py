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
    "U2": (27.0, 13.0, 0), "C1": (27.0, 8.5, 0), "C2": (32.0, 13.0, 90), "C3": (27.0, 17.5, 0),
    "R2": (19.0, 22.0, 0), "R3": (23.0, 22.0, 0), "C4": (27.0, 22.0, 0),
    # MCU in the middle with its decoupling close to the pins
    "U1": (48.0, 45.0, 0),
    "C5": (40.0, 37.0, 45), "C6": (56.0, 37.0, 135), "C7": (40.0, 53.0, 135), "C8": (56.0, 53.0, 45),
    "C9": (48.0, 36.0, 0), "C10": (48.0, 56.0, 0),
    # VDDA filter right under the VDDA pins 28/29 (bottom edge of U1)
    "C12": (51.5, 56.5, 90), "C11": (53.5, 56.5, 90), "FB1": (55.5, 56.5, 90),
    # programming connector + reset at the bottom edge
    "J3": (48.0, 80.0, 0), "SW1": (63.0, 82.0, 0), "C13": (56.0, 76.0, 0),
    # pull-ups next to the MCU
    "R4": (35.0, 60.0, 90), "R5": (37.0, 60.0, 90), "R6": (39.0, 60.0, 90),
    "R7": (33.0, 45.0, 90), "R8": (33.0, 49.0, 90), "R9": (64.0, 52.0, 90),
    # LEDs at the top edge (visible)
    "R10": (40.0, 9.0, 0), "D1": (40.0, 5.0, 0), "R11": (46.0, 9.0, 0), "D2": (46.0, 5.0, 0),
    # module sockets
    "J7": (64.0, 6.0, 90),     # GPS: top edge, antenna cable leaves at the top
    "J9": (90.0, 22.0, 0),     # LoRa: right edge, far from the GPS antenna side
    "C14": (86.0, 20.0, 90),
    "J5": (8.0, 56.0, 0),      # IMU
    "J6": (22.0, 70.0, 0),     # barometer
    "J4": (8.0, 32.0, 0),      # SPI flash
    "J8": (14.0, 85.0, 90),    # DS18B20 probe connector at the edge (probe goes outside)
    "J10": (74.0, 86.0, 90),   # OLED (bench only)
    "J12": (91.0, 56.0, 0),    # expansion header: right edge, under the LoRa adapter
    # recovery buzzer: cable connector at the bottom edge, FET and parts next to it
    "J11": (30.0, 86.0, 90), "Q2": (37.0, 81.0, 0), "R12": (37.0, 77.0, 0), "D3": (41.0, 84.0, 90),
    # test points along the bottom-right
    "TP1": (70.0, 60.0, 0), "TP2": (74.0, 60.0, 0), "TP3": (78.0, 60.0, 0),
    "TP4": (70.0, 64.0, 0), "TP5": (74.0, 64.0, 0), "TP6": (78.0, 64.0, 0),
    "TP7": (70.0, 68.0, 0), "TP8": (74.0, 68.0, 0), "TP9": (78.0, 68.0, 0), "TP10": (82.0, 68.0, 0),
}

# Approximate plug-in module bodies (x, y, w, h in board mm) -> drawn on User.Drawings
MODULE_BODIES = {
    "GPS MAX-M10S": (58.0, 4.0, 26.0, 22.0),
    "LoRa E22 adapter": (70.0, 18.0, 24.0, 34.0),
    "IMU GY-601N1": (4.0, 52.0, 20.0, 22.0),
    "BARO BMP390": (18.0, 66.0, 16.0, 16.0),
    "FLASH": (4.0, 28.0, 20.0, 18.0),
}


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

    # module body outlines + names on User.Drawings
    for label, (x, y, w, h) in MODULE_BODIES.items():
        rect = pcbnew.PCB_SHAPE(board)
        rect.SetShape(pcbnew.SHAPE_T_RECT)
        rect.SetStart(pt(x, y))
        rect.SetEnd(pt(x + w, y + h))
        rect.SetLayer(pcbnew.Dwgs_User)
        rect.SetWidth(mm(0.15))
        board.Add(rect)
        txt = pcbnew.PCB_TEXT(board)
        txt.SetText(label + " (module body, approx.)")
        txt.SetPosition(pt(x + w / 2, y + h / 2))
        txt.SetLayer(pcbnew.Dwgs_User)
        txt.SetTextSize(pcbnew.VECTOR2I(mm(1.0), mm(1.0)))
        board.Add(txt)

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
