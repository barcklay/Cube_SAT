#!/usr/bin/env python3
"""Route hab1_flight.kicad_pcb automatically with Freerouting.

Run with KiCad's own Python after gen_pcb.py has placed the parts:

    /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
        hardware/flight_pcb/route_pcb.py [--reroute]

Needs Java (brew install openjdk) and the Freerouting jar in
~/.local/share/freerouting/ (https://github.com/freerouting/freerouting/releases).

What it does: marks In1 (GND) and In2 (+3V3) as plane layers so no signal is routed
there, gives the battery and buzzer nets wider tracks, exports the board to Specctra
DSN, lets Freerouting route it, imports the result, refills the planes and saves.
A board that already has tracks is left alone unless --reroute is given (then all
tracks and vias are removed first).
"""

import glob
import math
import os
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, "hab1_flight.kicad_pcb")
JAVA = "/opt/homebrew/opt/openjdk/bin/java"
JARS = sorted(glob.glob(os.path.expanduser("~/.local/share/freerouting/freerouting-*.jar")))
PASSES = 100

# Battery and buzzer currents (E22 TX peaks ~120 mA): wider tracks. GND and +3V3 are planes.
POWER_NETS = ["/VBAT_RAW", "/VBAT_SW", "/VBAT_PROT", "/BUZ_N", "+3V3"]  # names as in the board


def mm(v):
    return pcbnew.FromMM(v)


# ---- fan-out: a short track + via from every SMD ground/3V3 pad to its plane ----------
PLANE_NETS = ("GND", "+3V3")
VIA_D, VIA_DRILL, FAN_W, CLEAR = 0.6, 0.3, 0.25, 0.2  # mm (0.25: fits between 0.5 mm-pitch QFP pins)


def rect_dist(px, py, box):
    """Distance from a point to a rectangle (x0, y0, x1, y1); 0 inside."""
    dx = max(box[0] - px, 0, px - box[2])
    dy = max(box[1] - py, 0, py - box[3])
    return math.hypot(dx, dy)


def fanout(board):
    obstacles = []  # (box in mm, net) of every pad
    for fp in board.Footprints():
        for pad in fp.Pads():
            bb = pad.GetBoundingBox()
            obstacles.append(((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                               pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())),
                              pad.GetNetname()))
    edge = board.GetBoardEdgesBoundingBox()
    ex0, ey0 = pcbnew.ToMM(edge.GetLeft()) + 1.0, pcbnew.ToMM(edge.GetTop()) + 1.0
    ex1, ey1 = pcbnew.ToMM(edge.GetRight()) - 1.0, pcbnew.ToMM(edge.GetBottom()) - 1.0
    vias = []  # (x, y, net) of placed fan-out vias
    placed = failed = 0
    later = []  # pads with no room for their own via: tie them to a neighbour's via

    def clear_path(px, py, vx, vy, net, own=None):
        for k in range(1, 8):
            sx, sy = px + (vx - px) * k / 8, py + (vy - py) * k / 8
            if own is not None and rect_dist(sx, sy, own) == 0:
                continue  # still on the pad itself
            if not all(n == net or rect_dist(sx, sy, box) > FAN_W / 2 + CLEAR for box, n in obstacles):
                return False
        return True

    def add_stub(pad, vx, vy):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pad.GetPosition())
        t.SetEnd(pcbnew.VECTOR2I(mm(vx), mm(vy)))
        t.SetWidth(mm(FAN_W))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(pad.GetNet())
        board.Add(t)

    for fp in board.Footprints():
        cx, cy = pcbnew.ToMM(fp.GetPosition().x), pcbnew.ToMM(fp.GetPosition().y)
        for pad in fp.Pads():
            net = pad.GetNetname()
            if net not in PLANE_NETS or pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            px, py = pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y)
            pb = pad.GetBoundingBox()
            own = (pcbnew.ToMM(pb.GetLeft()), pcbnew.ToMM(pb.GetTop()),
                   pcbnew.ToMM(pb.GetRight()), pcbnew.ToMM(pb.GetBottom()))
            base = math.atan2(py - cy, px - cx) if (px, py) != (cx, cy) else -math.pi / 2
            if len(fp.Pads()) > 8:
                # IC pins: go straight out along the pin, so the via does not sit in front
                # of the neighbouring pins' way out
                w, h = own[2] - own[0], own[3] - own[1]
                if h > w:
                    base = math.pi / 2 if py > cy else -math.pi / 2
                else:
                    base = 0.0 if px > cx else math.pi
            spot = None
            for d in (0.9, 1.1, 1.4, 1.8, 2.3, 2.8):
                for da in (0, 30, -30, 60, -60, 90, -90, 135, -135, 180):
                    a = base + math.radians(da)
                    vx, vy = px + d * math.cos(a), py + d * math.sin(a)
                    if not (ex0 < vx < ex1 and ey0 < vy < ey1):
                        continue
                    ok = all(n == net or rect_dist(vx, vy, box) > VIA_D / 2 + CLEAR
                             for box, n in obstacles)
                    ok = ok and all(math.hypot(vx - x, vy - y) > VIA_D + CLEAR for x, y, _ in vias)
                    # the stub from pad to via must not pass over another net's pad
                    ok = ok and clear_path(px, py, vx, vy, net, own)
                    if ok:
                        spot = (vx, vy)
                        break
                if spot:
                    break
            if not spot:
                later.append((fp, pad, px, py, net, own))
                continue
            netinfo = pad.GetNet()
            add_stub(pad, spot[0], spot[1])
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(mm(spot[0]), mm(spot[1])))
            v.SetWidth(mm(VIA_D))
            v.SetDrill(mm(VIA_DRILL))
            v.SetNet(netinfo)
            board.Add(v)
            vias.append((spot[0], spot[1], net))
            obstacles.append(((spot[0] - VIA_D / 2, spot[1] - VIA_D / 2,
                               spot[0] + VIA_D / 2, spot[1] + VIA_D / 2), net))
            placed += 1
    tied = 0
    def box_of(p):
        bb = p.GetBoundingBox()
        return (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))

    # a same-net pad as target: aim at its point nearest to our pad, so the stub leaves
    # the QFP straight between the neighbouring pins instead of at an angle
    same_net_pads = []
    for f in board.Footprints():
        for p in f.Pads():
            bx = box_of(p)
            same_net_pads.append((bx, p.GetNetname()))
    for fp, pad, px, py, net, own in later:
        # nearest same-net via, or a same-net pad (e.g. the decoupling capacitor next to it)
        targets = [(x, y) for x, y, n in vias if n == net]
        for bx, n in same_net_pads:
            if n == net and bx != own:
                inset = 0.2  # a point a little inside the pad
                targets.append((min(max(px, bx[0] + inset), bx[2] - inset),
                                min(max(py, bx[1] + inset), bx[3] - inset)))
        near = sorted((math.hypot(x - px, y - py), x, y) for x, y in targets)
        for d, x, y in near:
            if d < 6.0 and clear_path(px, py, x, y, net, own):
                add_stub(pad, x, y)
                tied += 1
                break
        else:
            failed += 1
            print("  no room for a fan-out via at %s pad %s (%s)" % (fp.GetReference(),
                                                                   pad.GetNumber(), net))
    print("fan-out: %d vias to the GND/3V3 planes, %d pads tied to a neighbour's via, %d without"
          " room" % (placed, tied, failed))


def main():
    if not JARS:
        sys.exit("Freerouting jar not found in ~/.local/share/freerouting/")
    board = pcbnew.LoadBoard(PCB)
    tracks = list(board.GetTracks())
    if tracks:
        if "--reroute" not in sys.argv:
            sys.exit("board already has %d tracks/vias: run with --reroute to redo" % len(tracks))
        for t in tracks:
            board.Remove(t)

    # Inner layers are solid planes, not routing layers.
    board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)
    board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_POWER)

    # Default rules: 0.25 mm tracks, 0.2 mm clearance, 0.6/0.3 mm vias (JLCPCB standard).
    ns = board.GetDesignSettings().m_NetSettings
    default = ns.GetDefaultNetclass()
    default.SetTrackWidth(mm(0.25))
    default.SetClearance(mm(0.2))
    default.SetViaDiameter(mm(0.6))
    default.SetViaDrill(mm(0.3))
    power = pcbnew.NETCLASS("Power")
    power.SetTrackWidth(mm(0.5))
    power.SetClearance(mm(0.2))
    power.SetViaDiameter(mm(0.8))
    power.SetViaDrill(mm(0.4))
    ns.SetNetclass("Power", power)
    for name in POWER_NETS:
        ns.SetNetclassPatternAssignment(name, "Power")
    ns.RecomputeEffectiveNetclasses()

    # Plane clearance 0.2 mm (JLCPCB minimum is 0.1): with 0.3 the via antipads around the
    # MCU cut the ground plane into islands and some ground vias ended up unconnected.
    for z in board.Zones():
        z.SetLocalClearance(mm(0.2))
        z.SetMinThickness(mm(0.2))
    fanout(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())

    dsn = os.path.join(HERE, "hab1_flight.dsn")
    ses = os.path.join(HERE, "hab1_flight.ses")
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        sys.exit("DSN export failed")
    print("routing with %s (%d passes max)..." % (os.path.basename(JARS[-1]), PASSES), flush=True)
    run = subprocess.run([JAVA, "-jar", JARS[-1], "-de", dsn, "-do", ses, "-mp", str(PASSES),
                          "--gui.enabled=false"], capture_output=True, text=True)
    tail = [l for l in (run.stdout + run.stderr).splitlines() if l.strip()][-6:]
    print("\n".join(tail))
    if not os.path.exists(ses):
        sys.exit("Freerouting produced no session file")
    if not pcbnew.ImportSpecctraSES(board, ses):
        sys.exit("SES import failed")

    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)
    for f in (dsn, ses):
        os.remove(f)
    items = list(pcbnew.LoadBoard(PCB).GetTracks())  # re-read: the SWIG list breaks after SES import
    vias = [t for t in items if t.GetClass() == "PCB_VIA"]
    tracks = [t for t in items if t.GetClass() == "PCB_TRACK"]
    length = sum(t.GetLength() for t in tracks) / 1e6
    print("saved: %d track segments (%.0f mm), %d vias" % (len(tracks), length, len(vias)))


main()
