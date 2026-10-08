#!/usr/bin/env python3
"""Breakout board for the Ebyte E22-400M22S LoRa module: 1.27 mm castellated pads -> two rows
of ordinary 2.54 mm pins, 0.8 inch apart (fits a breadboard). For the bench and the ground
station; the flight board carries the module itself.

Run with KiCad's own Python (Java and the Freerouting jar as for the flight board):

    /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
        hardware/lora_adapter/gen_adapter.py

Writes e22_adapter.kicad_pcb (placed, routed, ground pours on both sides) and fab/ with the
gerber zip, BOM and CPL for JLCPCB. There is no schematic: the connections are the table below.
"""
import csv
import glob
import os
import subprocess
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "e22_adapter.kicad_pcb")
FAB = os.path.join(HERE, "fab")
FPLIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
OWNLIB = os.path.join(HERE, "..", "flight_pcb")          # hab1.pretty with the E22 footprint
KC = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
JAVA = "/opt/homebrew/opt/openjdk/bin/java"
JARS = sorted(glob.glob(os.path.expanduser("~/.local/share/freerouting/freerouting-*.jar")))

HALF_W, TOP, BOTTOM = 15.8, -14.0, 11.2     # board edges, mm from the module centre
OX, OY = 100.0, 100.0
ROW_X = 10.16                               # pin rows 20.32 mm (0.8 inch) apart

# The two pin rows, top to bottom, in the same order as the module's own pads on that side.
LEFT = ["GND", "DIO1", "BUSY", "NRST", "MISO", "MOSI", "SCK", "NSS"]
RIGHT = ["GND", "3V3", "DIO2", "TXEN", "RXEN", "GND", "GND", "GND"]
# module pin -> net (Ebyte manual v1.2 section 3); 21 = ANT stays open (antenna on the IPX connector)
MODULE = {1: "GND", 2: "GND", 3: "GND", 4: "GND", 5: "GND", 6: "RXEN", 7: "TXEN", 8: "DIO2", 9: "3V3",
          10: "GND", 11: "GND", 12: "GND", 13: "DIO1", 14: "BUSY", 15: "NRST", 16: "MISO", 17: "MOSI",
          18: "SCK", 19: "NSS", 20: "GND", 22: "GND"}
PARTS = [  # ref, library, footprint, value, x, y, rotation, {pad: net}
    ("U1", "hab1", "E22-400M22S", "E22-400M22S", 0, 0, 0, {str(k): v for k, v in MODULE.items()}),
    ("J1", "Connector_PinHeader_2.54mm", "PinHeader_1x08_P2.54mm_Vertical", "LEFT", -ROW_X, -8.89, 0,
     {str(i + 1): n for i, n in enumerate(LEFT)}),
    ("J2", "Connector_PinHeader_2.54mm", "PinHeader_1x08_P2.54mm_Vertical", "RIGHT", ROW_X, -8.89, 0,
     {str(i + 1): n for i, n in enumerate(RIGHT)}),
    ("C1", "Capacitor_SMD", "C_0805_2012Metric", "4.7u", 2.6, -11.7, 0, {"1": "3V3", "2": "GND"}),
    ("C2", "Capacitor_SMD", "C_0603_1608Metric", "100n", 6.3, -11.7, 0, {"1": "3V3", "2": "GND"}),
]
LCSC = {"E22-400M22S": ("C411291", "E22-400M22S LoRa module (Ebyte, SX1268)"),
        "4.7u": ("C1779", "0805 4.7uF 25V X5R"), "100n": ("C14663", "0603 100nF 50V X7R")}


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def silk(board, text, x, y, size=0.8, just=0, layer=pcbnew.F_SilkS):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(pt(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(size * 0.16))
    t.SetHorizJustify((pcbnew.GR_TEXT_H_ALIGN_CENTER, pcbnew.GR_TEXT_H_ALIGN_LEFT,
                       pcbnew.GR_TEXT_H_ALIGN_RIGHT)[just])
    if layer == pcbnew.B_SilkS:
        t.SetMirrored(True)
    board.Add(t)


def build():
    board = pcbnew.CreateEmptyBoard()
    board.SetCopperLayerCount(2)
    corners = [(-HALF_W, TOP), (HALF_W, TOP), (HALF_W, BOTTOM), (-HALF_W, BOTTOM)]
    for (x1, y1), (x2, y2) in zip(corners, corners[1:] + corners[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pt(x1, y1))
        seg.SetEnd(pt(x2, y2))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(mm(0.1))
        board.Add(seg)
    nets = {}
    for name in ["GND", "3V3"] + [n for n in LEFT + RIGHT if n not in ("GND", "3V3")]:
        if name not in nets:
            nets[name] = pcbnew.NETINFO_ITEM(board, name)
            board.Add(nets[name])
    for ref, lib, name, value, x, y, rot, pads in PARTS:
        libdir = OWNLIB if lib == "hab1" else FPLIB
        fp = pcbnew.FootprintLoad(os.path.join(libdir, lib + ".pretty"), name)
        fp.SetFPID(pcbnew.LIB_ID(lib, name))
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(rot)
        fp.Reference().SetVisible(False)   # the pin names say more than J1, U1, C1
        board.Add(fp)
        for pad in fp.Pads():
            if pad.GetNumber() in pads:
                pad.SetNet(nets[pads[pad.GetNumber()]])
    # pin names outside each row, and the same on the back (read from below when it is plugged in)
    for i, (l, r) in enumerate(zip(LEFT, RIGHT)):
        y = -8.89 + 2.54 * i
        silk(board, l, -ROW_X - 1.7, y, 0.8, 2)
        silk(board, r, ROW_X + 1.7, y, 0.8, 1)
    silk(board, "E22-400M22S  HAB-1", 0.0, 10.4, 0.8, 0, pcbnew.B_SilkS)
    silk(board, "3V3 only!", -7.5, -12.4, 0.8)
    # no tracks or vias under the module on the top side (its underside is not insulated)
    keep = pcbnew.ZONE(board)
    keep.SetIsRuleArea(True)
    keep.SetDoNotAllowTracks(True)
    keep.SetDoNotAllowVias(True)
    keep.SetDoNotAllowZoneFills(False)
    keep.SetDoNotAllowPads(False)
    keep.SetDoNotAllowFootprints(False)
    keep.SetLayer(pcbnew.F_Cu)
    ko = keep.Outline()
    ko.NewOutline()
    for dx, dy in ((-5.6, -9.8), (5.6, -9.8), (5.6, 9.8), (-5.6, 9.8)):
        ko.Append(mm(OX + dx), mm(OY + dy))
    board.Add(keep)
    # ground pour on both sides
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(nets["GND"])
        o = z.Outline()
        o.NewOutline()
        for x, y in ((-HALF_W + 0.4, TOP + 0.4), (HALF_W - 0.4, TOP + 0.4), (HALF_W - 0.4, BOTTOM - 0.4),
                     (-HALF_W + 0.4, BOTTOM - 0.4)):
            o.Append(mm(OX + x), mm(OY + y))
        z.SetMinThickness(mm(0.2))
        z.SetLocalClearance(mm(0.2))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)   # solid: the pours carry the ground
        board.Add(z)
    ns = board.GetDesignSettings().m_NetSettings
    d = ns.GetDefaultNetclass()
    d.SetTrackWidth(mm(0.3))
    d.SetClearance(mm(0.2))
    d.SetViaDiameter(mm(0.6))
    d.SetViaDrill(mm(0.3))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(OUT, board)
    return board


def route(board):
    dsn, ses = OUT.replace(".kicad_pcb", ".dsn"), OUT.replace(".kicad_pcb", ".ses")
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        raise SystemExit("DSN export failed")
    subprocess.run([JAVA, "-jar", JARS[-1], "-de", dsn, "-do", ses, "-mp", "50", "--gui.enabled=false"],
                   capture_output=True, text=True)
    if not os.path.exists(ses) or not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("routing failed")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(OUT, board)
    for f in (dsn, ses):
        os.remove(f)


def fab():
    g = os.path.join(FAB, "gerber") + os.sep
    os.makedirs(g, exist_ok=True)
    for f in os.listdir(g):
        os.remove(g + f)
    subprocess.run([KC, "pcb", "export", "gerbers", "--no-x2", "--subtract-soldermask", "--layers",
                    "F.Cu,B.Cu,F.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts", "-o", g, OUT],
                   check=True, capture_output=True)
    subprocess.run([KC, "pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm",
                    "--excellon-separate-th", "-o", g, OUT], check=True, capture_output=True)
    with zipfile.ZipFile(os.path.join(FAB, "e22_adapter_gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(g)):
            z.write(g + f, f)
    raw = os.path.join(FAB, "pos_raw.csv")
    subprocess.run([KC, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "front",
                    "-o", raw, OUT], check=True, capture_output=True)
    rows = [r for r in csv.DictReader(open(raw)) if r["Val"] in LCSC]
    os.remove(raw)
    with open(os.path.join(FAB, "bom_jlcpcb.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for r in rows:
            w.writerow([LCSC[r["Val"]][1], r["Ref"], r["Package"], LCSC[r["Val"]][0]])
    with open(os.path.join(FAB, "cpl_jlcpcb.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in rows:
            w.writerow([r["Ref"], "%.3fmm" % float(r["PosX"]), "%.3fmm" % float(r["PosY"]), "Top",
                        "%g" % float(r["Rot"])])
    print("fab: gerber zip, BOM and CPL with %d parts for JLCPCB; the two pin rows are soldered by hand"
          % len(rows))


def main():
    board = build()
    route(board)
    b = pcbnew.LoadBoard(OUT)
    items = list(b.GetTracks())
    print("board %.1f x %.1f mm, %d track segments, %d vias" % (2 * HALF_W, BOTTOM - TOP,
          len([t for t in items if t.GetClass() == "PCB_TRACK"]), len([t for t in items if t.GetClass() == "PCB_VIA"])))
    fab()


main()
