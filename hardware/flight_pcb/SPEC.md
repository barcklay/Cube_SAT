# HAB-1 flight board — specification (rev A draft)

Status: **draft for the schematic**. Source of truth for pins: the working bench setup
`hab_bringup` (NUCLEO-G474RE + breadboard), firmware commit `e69263b`.
Linear: HW-24 (mounting the stack / own PCB), phase 3 HW-20.

## 1. Decisions taken

| Topic | Decision | Why |
|---|---|---|
| Microcontroller | **STM32G474RET6, LQFP64 — the same chip as on the Nucleo**, placed by JLCPCB (PCBA) | firmware and pinout move over almost unchanged |
| Sensors | **existing breakout modules on female headers** (GPS, barometer, IMU, flash); E22 LoRa as SMD footprint or on headers (see open questions) | reuse tested modules; a broken module is swapped without soldering |
| Size | **CubeSat 1U board outline ≈ 95.9 × 90.2 mm**, 4 mounting holes (PC/104-style) | portfolio "real CubeSat" look, reusable for CanSat/CubeSat later |
| Clock | **internal HSI + PLL, 170 MHz** (as now) — no crystal needed | bench firmware already runs on HSI |
| Programming / console | **STDC14 connector** (2×7, 1.27 mm), the same as CN4 "DEBUG" on the Nucleo's ST-LINK | program the flight board with the Nucleo's ST-LINK V3 + one cable; STDC14 also carries the virtual COM port (LPUART1), so `telemetry.py`/`logdump.py` keep working |

## 2. Block diagram

```
 3–4× Energizer L91 ─┬─ power switch ─ reverse-polarity FET ─ 3.3 V regulator ─┬─ 3V3 bus
                     └─ divider ─► ADC (battery voltage)                        │
                                                                                │
  STDC14 (SWD + VCP) ─── STM32G474RET6 ────────────────────────────────────────┤
                          │  I2C1  ── barometer BMP390 module (header)          │
                          │  SPI1  ── SPI flash module (header)                 │
                          │  SPI3  ── IMU ICM-42688 module (header)             │
                          │  USART1── GPS MAX-M10S module (header) + antenna    │
                          │  GPIO  ── DS18B20 probe (3-pin connector)           │
                          │  SPI2  ── LoRa E22-400M22S + IPEX antenna           │
                          │  GPIO  ── LED "alive", LED "fix"                    │
                          └  I2C1  ── spare 4-pin header (OLED SSD1306, bench)
```

## 3. Pin map (same as the bench unless marked NEW)

| STM32 pin | Signal | Goes to | Notes |
|---|---|---|---|
| PA13 / PA14 | SWDIO / SWCLK | STDC14 | programming |
| PB3 | SWO | STDC14 | optional trace |
| NRST | reset | STDC14 + 100 nF to GND + button | |
| PA2 / PA3 | LPUART1 TX / RX | STDC14 VCP pins | console 115200 |
| PA15 | I2C1_SCL | barometer SCK, spare I2C header | **4.7 kΩ pull-up to 3V3 on the board** (the module has none on SCK). **Differs from the Nucleo bench, where SCL is on PB8**: the flight build of the firmware must use PA15 (AF4) |
| PB8 (BOOT0) | not used | 10 kΩ pull-down | BOOT0 held low, so the board always boots from flash, also with factory option bytes |
| PB9 | I2C1_SDA | barometer SDI, spare I2C header | 4.7 kΩ pull-up to 3V3 |
| PA5 / PA6 / PA7 | SPI1 SCK / MISO / MOSI | flash CLK / DO / DI | flash **alone** on SPI1 |
| PB6 | FLASH_CS | flash CS | 10 kΩ pull-up (idle high) |
| PC10 / PC11 / PC12 | SPI3 SCK / MISO / MOSI | IMU SCLK / SDO / SDI | IMU **alone** on SPI3 (it corrupted the flash on a shared bus) |
| PC7 | IMU_CS | IMU CS | 10 kΩ pull-up (idle high) |
| PC4 / PC5 | USART1 TX / RX | GPS RX / TX | GPS runs at 38400 baud (auto-detected) |
| PA8 | GPS_PPS | GPS PPS | **NEW**, 1 pulse per second, TIM1_CH1 input capture for precise time |
| PA10 | DS18B20 (open drain) | probe data | 4.7 kΩ pull-up to 3V3 |
| PB13 / PB14 / PB15 | SPI2 SCK / MISO / MOSI | E22 SCK / MISO / MOSI | **NEW**, LoRa on its own bus |
| PB12 | LORA_NSS | E22 NSS | **NEW**, 10 kΩ pull-up |
| PC6 | LORA_BUSY | E22 BUSY | **NEW**, input |
| PC8 | LORA_DIO1 | E22 DIO1 | **NEW**, input + interrupt |
| PC9 | LORA_NRST | E22 NRST | **NEW** |
| PB10 / PB11 | LORA_TXEN / LORA_RXEN | E22 TXEN / RXEN | **NEW**, RF switch control (check E22 datasheet) |
| PA0 | VBAT_SENSE | battery divider | **NEW**, ADC; divider e.g. 100 k / 47 k + 100 nF |
| PB0 | LED_ALIVE | LED + 1 kΩ | **NEW**, heartbeat |
| PB1 | LED_FIX | LED + 1 kΩ | **NEW**, GPS fix |
| PB4 | BUZZER | AO3400A low-side FET → 2-pin JST to the buzzer on the box wall | **NEW**, recovery beeper from the battery rail, only after LANDED; TIM3_CH1 if a passive buzzer is used |
| PA1 / PA4 / PC0 / PC1 | EXP_A1…A4 | expansion header J12 through 1 kΩ | **NEW**, spare analog-capable pins (ADC): more temperature probes, a sun sensor, a heater thermistor |
| PB5 / PB7 | EXP_PWM1 / EXP_PWM2 | expansion header J12 through 1 kΩ | **NEW**, spare timer outputs (TIM3_CH2 / TIM4_CH2), e.g. a battery heater switch |

The NEW pins are proposals: check in CubeMX that each one offers the needed function
before the schematic is final, then add them to `hab_bringup.ioc` so the flight board
and the bench stay one firmware.

## 4. MCU support circuit (per ST AN5093 "Getting started with STM32G4 hardware")

- 100 nF next to every VDD pin + one 4.7 µF on the 3V3 bus near the MCU.
- VDDA / VREF+: 100 nF + 1 µF, fed from 3V3 through a ferrite bead.
- NRST: 100 nF to GND (internal pull-up is enough), reset button.
- BOOT0/PB8: no strap needed (see option bytes above).
- No HSE/LSE crystal (HSI). Leave PF0/PF1/PC14/PC15 unconnected or as test pads.

## 5. Power

- 3V3 bus current estimate: MCU ~40 mA at 170 MHz, GPS ~25–35 mA (+ active antenna),
  IMU ~1 mA, barometer < 1 mA, flash up to ~25 mA while writing, **E22 TX peak ~110–120 mA**
  → plan for **≥ 300 mA** regulator.
- Battery (HW-22): Energizer L91 lithium primaries — only chemistry that works at −60 °C.
- Regulator (HW-23) — **open question**: 3× L91 fall towards ~3.0–3.3 V late in life and in
  the cold, which is too low for an LDO to make 3.3 V. Options: 4× L91 + LDO, or a
  buck-boost. Decide from the power budget measurements (HW-21, HW-2).
- Reverse-polarity protection (P-FET), power switch reachable from outside (HW-25),
  bulk capacitor near the E22 (TX bursts).

## 6. Layout rules

- **4 layers** (signal / GND / 3V3 / signal): solid ground plane under GPS and LoRa;
  JLCPCB 4-layer prototypes are cheap.
- Keep the GPS module/antenna and the LoRa antenna at opposite edges (HW-45 RF test).
- E22 antenna on IPEX; keep copper-free area per datasheet under the RF part.
- DS18B20 on a 3-pin locking connector at the board edge (probe goes outside the box).
- Test pads on every bus (SCL, SDA, SPI1/2/3, UART) for the oscilloscope / logic analyzer (HW-144).
- Silkscreen: pin names on every header, board name, revision, date.
- No dupont wires in flight: headers are soldered, modules are glued/strapped (HW-25).

## 7. Open questions before ordering

1. **LoRa E22**: not brought up yet (HW-6 waits for the second module, HW-143). Either
   finish HW-6 first, or put E22 on headers on rev A to avoid a wrong footprint.
2. **Regulator and battery count** (section 5) — after HW-21.
3. **STM32G474RET6 stock at JLCPCB** (extended part) — check before the layout.
4. Mounting holes: take exact positions from the CubeSat/PC/104 board drawing.
5. Flash chip: GigaDevice on the bench; original W25Q128 for flight (HW-128).

## 8. Steps

1. Install **KiCad** (free) — `brew install --cask kicad`.
2. Schematic from this spec → ERC.
3. Footprints (JLCPCB parts where assembled), board outline 95.9 × 90.2 mm.
4. Layout → DRC → 3D view check against real modules.
5. Gerbers + BOM + CPL → JLCPCB order (PCBA for MCU and passives; headers and modules by hand).
6. Bring-up of the new board with the same firmware; then HW-24 check: the full stack
   works after the move.

## Changes after the independent review of 2026-10-06 (review/2026-10-06_fresh_eyes_review.md)

- STDC14 console pins 13/14 were swapped: fixed (pin 13 = target RX = PA3, pin 14 = target TX = PA2).
- Buzzer gate pull-down 100 k -> 4.7 k (PB4 has an internal pull-up during reset).
- Resettable fuses: F1 0.5 A on the battery input, F2 0.2 A on the buzzer cable.
- DS18B20 cable: 100 Ω in series with its 3V3 and with the data line.
- Expansion header J12: battery rail moved to pin 1 with GND next to it, 1 kΩ in series with every signal.
- I2C SCL moved from PB8 (BOOT0) to PA15; PB8 pulled down.
- 100 nF moved to 2 mm from every VDD pin; 100 nF at NRST and at VDDA/VREF+ next to the pins.
- 100 k pull-downs on LORA_TXEN / LORA_RXEN.
- Silkscreen: module name and every pin name at each socket, polarity at the JST connectors,
  net names at the test points.

- Regulator: MIC5219 (SOT-23-5, ~117 mA from 7.2 V before overheating) replaced by
  **LDL1117S33R** (ST, SOT-223, 2.5-18 V in, 1.2 A, low dropout, for ceramic output capacitors,
  JLCPCB C435835); bypass capacitor C3 removed. Heat at 7.2 V in: 0.39 W at 100 mA average,
  0.78 W during a 200 mA peak; the tab goes to the +3V3 plane. NOT yet verified from the
  datasheet PDF (the ST site did not let it be downloaded here): pin order (taken as the LD1117:
  1 GND, 2 + tab OUT, 3 IN), minimum output capacitance, thermal resistance of the package.

Still open from that review:
real module outlines (GPS vs LoRa, IMU vs barometer), a separate STLINK-V3MINIE for programming,
J9 pin order (adapter not bought yet).
