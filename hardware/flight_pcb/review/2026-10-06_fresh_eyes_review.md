# HAB-1 flight board rev A: fresh-eyes review (2026-10-06)

Reviewed: `hardware/flight_pcb/` (`SPEC.md`, `gen_schematic.py`, `gen_pcb.py`, `route_pcb.py`, `hab1_flight.kicad_sch`, `hab1_flight.kicad_pcb`), cross-checked against `hab_bringup/hab_bringup.ioc` and `Core/Src/main.c`. Nothing in the repository was modified.

Sources: "datasheet" means I read the document during this review (MIC5219 DS20006021A, ST UM2505 rev 4, ST UM2448). "Memory" means not re-read today.

## VERDICT

**Order after fixing the blockers.** There is one confirmed wiring error (console TX/RX swapped on the STDC14). The regulator choice and the module mechanical fit should also be settled before paying for a board, although neither is a proven failure.

## BLOCKERS

### B1. STDC14 pins 13 and 14 (VCP) are swapped: the console will not work and two outputs drive each other
- **Where:** J3 pins 13/14, `gen_schematic.py` lines 114-119. Generated netlist: `/VCP_TX` = J3.13 + U1.14 (PA2, LPUART1_TX); `/VCP_RX` = J3.14 + U1.17 (PA3, LPUART1_RX).
- **Evidence (datasheet):**
  - UM2448 (STLINK-V3SET): "T_VCP_RX (or RX) signal is the RX for the target (TX for the STLINK-V3SET), T_VCP_TX (or TX) signal is the TX for the target"; STDC14 pin 13 = T_VCP_RX, pin 14 = T_VCP_TX.
  - UM2505 (NUCLEO-G474RE) Table 14: CN4 pin 13 = "T_VCP_RX (PA3 by default)", pin 14 = "T_VCP_TX (PA2 by default)".
  - So the names are target-centred: pin 13 must go to the MCU **RX** (PA3) and pin 14 to the MCU **TX** (PA2). The comment in the generator ("pin 13 VCP_RX (probe input) <- MCU TX") reads them probe-centred, which is the wrong way round.
- **Effect:** SWD programming still works. The 115200 console (`telemetry.py`, `logdump.py`) does not, and the probe's TX output is tied directly to the MCU's TX output.
- **Fix:** in the J3 pin map set `"13": "VCP_RX", "14": "VCP_TX"`, then regenerate and reroute.

## SHOULD FIX

### S1. MIC5219-3.3YM5 (U2) cannot carry the planned load from 4 cells, and its output is all-ceramic
- **Thermal (datasheet):** theta-JA for SOT-23-5 is 220 C/W on a minimum footprint, Tj max 125 C. That allows 0.455 W at 25 C ambient.
  - At 7.2 V in: (7.2 - 3.3) V gives **117 mA continuous**. At 6.0 V in: 168 mA.
  - SPEC section 5 asks for ">= 300 mA". An E22 TX burst on top of MCU + GPS (about 200 mA) dissipates about 0.8 W, a steady-state rise of about 170 C.
  - The layout is worse than the datasheet's minimum footprint: there is no top copper pour, the GND pin has one 0.25 mm stub to one via, and OUT has a 0.9 mm x 0.25 mm stub to one via at (28.83, 11.47).
  - On the bench at room temperature with fresh cells, long LoRa packets can push it into thermal shutdown, which shows up as a 3V3 dropout and reset. At 30 km there is almost no convection, so the sea-level figure is optimistic.
- **Stability (datasheet section 4.3):** "ESR of about 1 ohm or less ... Ultra-low ESR capacitors could cause oscillation and/or under-damped transient response"; every application circuit shows tantalum. The board has about 62 uF of MLCC directly on the output (C2 10u, C14 47u, C10 4.7u, plus the 100n caps). I did not prove it oscillates; it is outside what the datasheet recommends.
- **Checked and fine:** pin order 1 IN / 2 GND / 3 EN / 4 BYP / 5 OUT matches the datasheet; Vin operating range is 2.5 to 12 V; EN tied to IN is allowed; BYP 470 pF with Cout >= 2.2 uF is as recommended; dropout is 400 mV max at 150 mA, so it regulates down to about 0.95 V per cell.
- **Fix, in order of preference:**
  1. A small buck converter (also roughly halves the battery current).
  2. A ceramic-stable LDO in SOT-223 or DPAK with a real copper area.
  3. As a minimum: add a top-side pour with several vias on the GND and IN pins, wire OUT to C2 with a 0.5 mm track and at least two vias, and look at 3V3 on a scope during a load step.

### S2. The buzzer will sound whenever the MCU is in reset, in the bootloader, or blank
- **Where:** Q2 gate, net `/BUZZER` on PB4; R12 100k to GND (`gen_schematic.py` line 141, comment "keeps it silent during reset / boot").
- **Evidence (memory, STM32G474 datasheet pin-table footnote, consistent with ST community threads found today):** PB4 is NJTRST after reset with the internal pull-up on, about 40k (25 to 55k). Against 100k that puts 2.1 to 2.6 V on the gate; AO3400A Vgs(th) is 1.45 V max, so the FET is on.
- **Reason for doubt:** PB4 is also UCPD1_CC2, and the dead-battery circuit is active until firmware disables it (`stm32g4xx_hal_msp.c` line 76). I could not establish whether that adds a pull-down on PB4.
- **Settle it on the Nucleo:** hold the reset button and measure D5 (PB4) with a 100k resistor to GND.
- **Fix:** make R12 4.7k (gate at most about 0.5 V), or move the buzzer to a pin with no reset pull, for example PC2 (TIM1_CH3) or PA9.

### S3. BOOT0 on PB8 with a 4.7k pull-up: every new board boots the ROM bootloader until the option byte is set
- **Where:** R4 on `/I2C_SCL` = U1.61 (PB8-BOOT0).
- **Evidence (memory):** factory option bytes have nSWBOOT0 = 1, so BOOT0 is read from the pin, which is high.
- **Effect:** not damaging and always recoverable over SWD, which stays available in the bootloader. But a freshly flashed board looks dead after a power cycle until `nSWBOOT0=0, nBOOT0=1` is written with STM32CubeProgrammer. The firmware does not set it (no `OB_`/`nSWBOOT` in `Core/Src`).
- **Fix:** make the option-byte write a written step of the flashing procedure, and have the firmware print a warning if the bit is still 1. To remove the dependency completely, move SCL to PA15 (I2C1_SCL, currently free) and give PB8 a pull-down.

### S4. The plan "program with the Nucleo's ST-LINK and one cable" is not a supported mode
- **Evidence (datasheet):** UM2505 section 6.3 describes CN4 only as the way to connect an **external** debug tool to the **on-board** STM32. I found no description of the reverse use and no jumpers that isolate the on-board G474 from CN4.
- **Effect:** with a straight cable, both MCUs share SWDIO, SWCLK and NRST, the VCP lines collide, and CN4 pin 3 ties the Nucleo's 3V3 to this board's +3V3 (two regulator outputs in parallel).
- **Fix:** buy a STLINK-V3MINIE. Its T_VCC pin only senses the target voltage, so the board must be powered from its battery while programming. Fit a shrouded, keyed STDC14 header (Samtec FTSH-107-01-L-DV-K style); the bare 2x7 pin header used now allows the cable to go in reversed.

### S5. Module bodies collide according to the designer's own outlines, and GPS sits next to LoRa
- **Where:** `gen_pcb.py` `MODULE_BODIES`, lines 79-85.
- **Evidence (measured from the files):**
  - GPS (x 58..84, y 4..26) and LoRa adapter (x 70..94, y 18..52) overlap by 14 x 8 mm.
  - IMU (x 4..24, y 52..74) and BARO (x 18..34, y 66..82) overlap by 6 x 8 mm.
  - The BARO rectangle does not even contain its own socket: J6 pins run to y = 85.24.
  - J6 (x = 22) sits between J8 (courtyard x 11.1..17.9) and J11 (x 27.1..33.9). A 7-pin barometer module wider than about 4 mm past its pin row hangs over one of those two vertical JST plugs and their wires.
- **Also:** SPEC section 6 asks for GPS and LoRa at opposite edges. J7 (top, x 64..74) and J9 (right, y 22..50) share the top-right corner, so the GPS receiver sits beside a 22 dBm transmitter.
- **Reason for doubt:** I do not have the real module dimensions; the rectangles are labelled "approx.".
- **Fix:** measure each module (pin row to each edge, and which side the body extends to), redraw the rectangles from the socket positions, and move J6 or J8/J11 and the GPS if they still overlap.

### S6. No silkscreen tells a person which module goes where or which way round
- **Evidence:** the F.Silkscreen plot contains only reference designators and the title. There are no module names, no pin names, no +/- on J1, J11 or J8, and the test points read "TP1..TP10" rather than their nets. SPEC section 6 asks for pin names on every header.
- **Effect:** every 1xN socket accepts its module rotated by 180 degrees. On J12, pin 3 carries battery voltage with no marking.
- **Fix:** add module name, a "1"/VCC label and a body-outline corner at each socket; polarity at the JSTs; net names at the test points; "VBAT!" at J12.3.

### S7. Cables that leave the box are unprotected, and the battery has no fuse
- **J11 buzzer:** pin 1 is `/VBAT_PROT` straight from the battery. A pinched cable shorts the pack through Q1 and 0.5 mm tracks, which ends the flight.
- **J8 DS18B20:** pin 1 is +3V3 directly and DQ goes straight to PA10. A short on the probe cable collapses the 3V3 rail, and a static discharge has a direct path into the MCU.
- **J12 expansion:** pin 3 (`/VBAT_PROT`, about 7 V) sits next to pin 2 (GND) and pin 4 (EXP_A1 = PA1).
- **Fix:**
  - a PTC or fuse of about 0.5 to 1 A after J1;
  - a 100 to 220 ohm resistor or small PTC in the buzzer feed;
  - about 100 ohm in series with the probe's 3V3, and 100 ohm plus a TVS/ESD diode on DQ;
  - move VBAT away from the signal pins on J12, or put 1k in series with the EXP pins.

### S8. Decoupling and reset parts are not at their pins
All coordinates are from the board file.
- **VDD caps:** C5 to C9 are 4.5 to 5.1 mm from the nearest VDD pin (for example C7 pad at (40.55, 53.55), pin 16 at (42.32, 48.75)), and there is no pin-to-cap track. Each pin and each cap drops to the planes through its own via. It will work, but it is not "next to every VDD pin" as SPEC section 4 says.
- **VDDA/VREF+ (pins 28/29):** a 0.25 mm track runs first to FB1 (about 7 mm), then on to C11, then C12. The 100 nF is about 11 mm from the pin and after the ferrite.
- **NRST:** C13 is at (55.2, 76.0) and pin 7 is at (42.3, 44.3), with about 65 mm of track in total, part of it running towards the LoRa side. The cap is at the button, not at the MCU, so RF pickup into a 40k node is plausible.
- **VBAT_SENSE:** 54 mm of track from the divider, with C4 at the divider end.
- **Fix:** move C5 to C9 to within 1 to 2 mm of the pins (PLACE table, `gen_pcb.py` lines 49-52), put C12 right at pins 28/29, put a 100 nF at pin 7, and move C4 to PA0.

### S9. HSI-only clock for the GPS UART in the cold
- **Evidence (memory):** HSI16 is about +/-1 % over 0 to 85 C and about -2/+1.5 % over -40 to 125 C. A UART tolerates roughly 2 to 3 % in total. The firmware uses `RCC_HSICALIBRATION_DEFAULT`.
- **Not affected:** LoRa (the E22 has its own TCXO), SPI and I2C.
- **Reason for doubt:** the box interior should stay above -20 C, where the error is small.
- **Fix:** add an unpopulated 32.768 kHz crystal footprint on PC14/PC15 (free), or trim HSI in firmware from GPS PPS on PA8.

## NICE TO HAVE

- +3V3 vias came out 0.6 mm pad with 0.4 mm drill (0.1 mm ring), because the Power netclass drill was combined with the fan-out pad size. Make them 0.8/0.4.
- Pull-downs on LORA_TXEN / LORA_RXEN, which float during reset; a pull-up on LORA_NRST if the adapter has none.
- The blue LED with 1k from 3.3 V has only 0.3 to 0.5 mA and will be very dim in the cold.
- Bring PA11/PA12 to pads: USB DFU would be a second way to recover a board.
- Test points for NRST, SWDIO/SWCLK and the LoRa SPI data lines; label them by net.
- A 2-pin jumper in series with the battery for current measurement (J2 can serve for this).
- The schematic has no LCSC part-number fields, so the JLCPCB BOM must be mapped by hand. STM32G474RET6 and MIC5219 are extended parts; TL3342 (SW1) and the 1.27 mm SMD header may not be stocked. Check before ordering.
- Thirteen silkscreen DRC warnings (reference designators over pads around C10..C12, FB1, R4..R6): cosmetic.
- J9 (LoRa adapter) pin order is invented, as the spec admits. If the adapter is not chosen before ordering, expect to hand-wire it.

## CHECKED AND FOUND OK

- **ERC and DRC:** ERC 0 violations. DRC with schematic parity: 0 unconnected, 0 parity issues, 13 silkscreen warnings only.
- **Netlist against the generator tables:** every net in the exported netlist matches `MCU_NETS`, `MCU_POWER` and `PARTS`.
- **STM32G474RET6 LQFP64 power pins (pin numbers from memory, match the KiCad symbol):** VBAT 1; VDD 16/32/48/64; VSS 15/31/47/63; VSSA 27; VREF+ 28; VDDA 29; NRST pin 7 with 100 nF and button.
- **Pin functions (memory):** LPUART1 PA2/PA3, I2C1 PB8/PB9, SPI1 PA5-7, SPI3 PC10-12, USART1 PC4/PC5, SPI2 PB13-15, TIM1_CH1 PA8, TIM3_CH1 PB4, ADC on PA0/PA1/PA4/PC0/PC1. Bench pins in the `.ioc` agree with the board.
- **AO3401A (Q1):** 1 G / 2 S / 3 D; drain to the battery, source to the load, gate through 10k to GND, which is the correct reverse-polarity arrangement. Vgs is 7.3 V against a +/-12 V limit (memory).
- **AO3400A (Q2):** 1 G / 2 S / 3 D, low-side switch; a 3.3 V gate is enough.
- **D3:** cathode (pin 1) on VBAT_PROT, anode on BUZ_N: correct flyback direction.
- **LEDs:** cathode (pin 1) to GND.
- **VBAT_SENSE:** 100k/47k gives 2.33 V at 7.3 V; about 50 uA, and only while the switch is on; 100 nF present.
- **Pull-ups:** I2C 4.7k x2, 1-Wire 4.7k, chip-select 10k x3.
- **GPS direction:** module TX (J7.3) to PC5 (USART1_RX), module RX (J7.4) to PC4.
- **BMP390:** CS and SDO tied high, which gives I2C at 0x77, matching the firmware.
- **STDC14 pins 3 to 12:** match UM2448/UM2505. J3 pads are odd on the left and even on the right with pin 1 at the top-left, matching the symbol.
- **Socket pin counts:** 6 / 8 / 7 / 5 / 4 / 12 / 10 match the tables. Pad 1 positions follow the PLACE rotations with no mirroring; all parts are on the top side.
- **Planes:** In1 (GND) and In2 (+3V3) are each one solid outline with no slots (In1 plot inspected); through-hole pads have thermal reliefs; all five VDD pins and all VSS pins have their own via.
- **Tracks:** battery tracks are 0.5 mm, fine for under 0.5 A.
- **Clearances:** nearest copper to a mounting hole is 7.4 mm; nearest track to the board edge is 2.8 mm.
- **Watchdog:** IWDG is used in the firmware.

## NOT CHECKED

- **Real pin orders of the flash, IMU, barometer, GPS and OLED modules.** No photos or drawings were available. I only confirmed that the board matches what the generator claims, not that the claims are true. Whether each module's component side faces up with this pin order (S5) is also unverified.
- **J9 / E22 adapter:** it does not exist yet.
- **AO3401A, AO3400A, STM32G474 and E22 datasheet values:** from memory, not re-read today.
- **PB4 behaviour with UCPD dead-battery active (S2):** needs the bench measurement.
- **MIC5219 stability with this capacitor bank:** needs a scope.
- **JLCPCB stock and basic/extended status, Gerber/CPL rotation offsets, the CubeSat/PC104 hole pattern** (the spec itself marks the 4 mm corner offset as unverified).
- **RF:** GPS desense by the 433 MHz transmitter, antenna placement.
- **Firmware:** not reviewed beyond pin cross-checks.
