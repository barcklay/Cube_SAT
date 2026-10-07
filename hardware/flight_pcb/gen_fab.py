#!/usr/bin/env python3
"""Make the files JLCPCB needs from hab1_flight.kicad_pcb (run after route_pcb.py):

    python3 hardware/flight_pcb/gen_fab.py

    fab/hab1_flight_gerbers.zip   copper, mask, paste, silkscreen, outline, drill files
    fab/bom_jlcpcb.csv            parts JLCPCB solders, with their LCSC numbers
    fab/cpl_jlcpcb.csv            where each of those parts sits and how it is turned
    fab/hand_soldered.txt         through-hole parts that are NOT in the order

The LCSC numbers below were checked in the JLCPCB parts catalogue on 2026-10-07 (part name,
package, stock). Stock changes: check the BOM page of the order form again before paying.
Part rotations are corrected with the usual KiCad -> JLCPCB table; the order form shows a
picture of the placed parts: look at pin 1 of U1, U2, Q1, Q2 and the LEDs and diodes there.
"""
import csv
import os
import subprocess
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "hab1_flight.kicad_pcb")
FAB = os.path.join(HERE, "fab")
KC = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"

# (value, footprint) -> (LCSC number, what it is, library: basic / extended)
LCSC = {
    ("10k", "R_0603_1608Metric"): ("C25804", "0603 10k 1%", "basic"),
    ("100k", "R_0603_1608Metric"): ("C25803", "0603 100k 1%", "basic"),
    ("47k", "R_0603_1608Metric"): ("C25819", "0603 47k 1%", "basic"),
    ("4.7k", "R_0603_1608Metric"): ("C23162", "0603 4.7k 1%", "basic"),
    ("1k", "R_0603_1608Metric"): ("C21190", "0603 1k 1%", "basic"),
    ("100", "R_0603_1608Metric"): ("C22775", "0603 100R 1%", "basic"),
    ("100n", "C_0603_1608Metric"): ("C14663", "0603 100nF 50V X7R", "basic"),
    ("1u", "C_0603_1608Metric"): ("C15849", "0603 1uF 50V X5R", "basic"),
    ("10u", "C_0805_2012Metric"): ("C15850", "0805 10uF 25V X5R", "basic"),
    ("10u 16V", "C_0805_2012Metric"): ("C15850", "0805 10uF 25V X5R", "basic"),
    ("4.7u", "C_0805_2012Metric"): ("C1779", "0805 4.7uF 25V X5R", "basic"),
    ("600R@100MHz", "L_0603_1608Metric"): ("C1002", "0603 ferrite bead 600R GZ1608D601TF", "basic"),
    ("AO3401A", "SOT-23"): ("C15127", "AO3401A P-FET SOT-23", "basic"),
    ("AO3400A", "SOT-23"): ("C20917", "AO3400A N-FET SOT-23", "basic"),
    ("1N4148W", "D_SOD-123"): ("C81598", "1N4148W SOD-123", "basic"),
    ("ALIVE red", "LED_0603_1608Metric"): ("C2286", "KT-0603R red LED", "basic"),
    ("GPS FIX yellow", "LED_0603_1608Metric"): ("C2287", "KT-0603Y yellow LED", "extended"),
    ("PTC 0.5A 24V", "Fuse_1206_3216Metric"): ("C492011", "SMD1206P050TF/24 resettable fuse", "extended"),
    ("PTC 0.2A 24V", "Fuse_1206_3216Metric"): ("C20984", "SMD1206P020TF resettable fuse 24V", "extended"),
    ("LDL1117S33R", "SOT-223-3_TabPin2"): ("C435835", "LDL1117S33R 3.3V LDO SOT-223", "extended"),
    ("STM32G474RET6", "LQFP-64_10x10mm_P0.5mm"): ("C521608", "STM32G474RET6 LQFP-64", "extended"),
    ("RESET", "SW_SPST_TL3342"): ("C2886898", "TL3342F160QG push button", "extended"),
    ("STDC14 SWD+VCP", "PinHeader_2x07_P1.27mm_Vertical_SMD"): ("C7465995", "FTSH-107-01-F-DV-K keyed 2x7 1.27 mm header", "extended"),
}
# KiCad -> JLCPCB rotation corrections (degrees, added to KiCad's angle)
ROTATE = {"SOT-23": 180, "SOT-223-3_TabPin2": 180, "LQFP-64_10x10mm_P0.5mm": 270}
SKIP_PREFIX = ("TP", "H")   # test points and mounting holes are not parts


def main():
    os.makedirs(os.path.join(FAB, "gerber"), exist_ok=True)
    g = os.path.join(FAB, "gerber") + os.sep
    for f in os.listdir(g):
        os.remove(g + f)
    subprocess.run([KC, "pcb", "export", "gerbers", "--no-x2", "--subtract-soldermask", "--layers",
                    "F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts",
                    "-o", g, PCB], check=True, capture_output=True)
    subprocess.run([KC, "pcb", "export", "drill", "--format", "excellon", "--excellon-units", "mm",
                    "--excellon-separate-th", "-o", g, PCB], check=True, capture_output=True)
    with zipfile.ZipFile(os.path.join(FAB, "hab1_flight_gerbers.zip"), "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(os.listdir(g)):
            z.write(g + f, f)
    raw = os.path.join(FAB, "pos_raw.csv")
    subprocess.run([KC, "pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "front",
                    "-o", raw, PCB], check=True, capture_output=True)
    rows = list(csv.DictReader(open(raw)))
    os.remove(raw)

    bom, cpl, hand, unknown = {}, [], [], []
    for r in rows:
        ref, key = r["Ref"], (r["Val"], r["Package"])
        if ref.startswith(SKIP_PREFIX):
            continue
        if key not in LCSC:
            (hand if ref.startswith("J") else unknown).append("%s  %s  %s" % (ref, r["Val"], r["Package"]))
            continue
        bom.setdefault(LCSC[key][0], (key, []))[1].append(ref)
        rot = (float(r["Rot"]) + ROTATE.get(r["Package"], 0)) % 360
        cpl.append([ref, "%.3fmm" % float(r["PosX"]), "%.3fmm" % float(r["PosY"]), "Top", "%g" % rot])
    with open(os.path.join(FAB, "bom_jlcpcb.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Comment", "Designator", "Footprint", "LCSC Part #"])
        for key, refs in bom.values():
            w.writerow([LCSC[key][1], ",".join(sorted(refs, key=lambda s: (s.rstrip("0123456789"), int("".join(c for c in s if c.isdigit()))))),
                        key[1], LCSC[key][0]])
    with open(os.path.join(FAB, "cpl_jlcpcb.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        w.writerows(cpl)
    with open(os.path.join(FAB, "hand_soldered.txt"), "w") as f:
        f.write("Through-hole parts, soldered by hand, not in the JLCPCB order:\n" + "\n".join(hand) + "\n")
    ext = sorted(n for n, (k, _) in bom.items() if LCSC[k][2] == "extended")
    print("gerbers: %d files in hab1_flight_gerbers.zip" % len(os.listdir(g)))
    print("BOM: %d lines, %d parts placed by JLCPCB; extended parts (setup fee each): %d  %s"
          % (len(bom), len(cpl), len(ext), " ".join(ext)))
    print("hand-soldered: %d connectors" % len(hand))
    if unknown:
        raise SystemExit("NO LCSC NUMBER for: " + "; ".join(unknown))


main()
