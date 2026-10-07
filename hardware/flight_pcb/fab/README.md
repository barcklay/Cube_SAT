# Files for JLCPCB (flight board rev A)

Made by `python3 hardware/flight_pcb/gen_fab.py` from the routed board. Do not edit by hand.

| File | What it is | Where it goes in the JLCPCB order form |
|---|---|---|
| `hab1_flight_gerbers.zip` | the board itself: 4 copper layers, mask, paste, silkscreen, outline, drill | "Add gerber file" |
| `bom_jlcpcb.csv` | the 49 parts JLCPCB solders, with LCSC numbers | PCB Assembly -> "Add BOM File" |
| `cpl_jlcpcb.csv` | where each part sits and how it is turned | PCB Assembly -> "Add CPL File" |
| `hand_soldered.txt` | 11 through-hole connectors that are NOT ordered: soldered by hand | - |

## Order settings

- Layers: 4. Size 95.89 x 90.17 mm. Thickness 1.6 mm. Solder mask: **red**. Silkscreen: white.
- Smallest features on the board: track 0.15 mm, gap 0.2 mm, via 0.6 mm pad / 0.3 mm hole.
  All inside the standard 4-layer process.
- Assembly: top side only, "Economic" is enough (no parts on the bottom, nothing finer than 0.5 mm pitch).

## Parts (checked in the JLCPCB catalogue on 2026-10-07)

15 of the 22 BOM lines are basic parts. 7 are extended (each adds a one-time loading fee):
STM32G474RET6 (C521608, 386 in stock), LDL1117S33R (C435835), the two resettable fuses
(C492011, C20984), the yellow LED (C2287), the reset button (C2886898) and the keyed
programming header FTSH-107-01-F-DV-K (C7465995, 274 in stock).

## Before paying

1. **Do not order until HW-162 is closed**: the pin order of the LoRa socket J9 is still a guess.
2. Stock changes: look at the BOM page of the order form, nothing may be "shortfall".
3. On the placement picture check which way these sit (pin 1 / polarity): U1, U2, Q1, Q2,
   D1, D2, D3, J3. The rotations in the CPL use the usual KiCad -> JLCPCB corrections and
   have not been confirmed on a real order.
4. Not checked by JLCPCB's own DFM yet: it runs when the gerbers are uploaded (free, no order needed).
