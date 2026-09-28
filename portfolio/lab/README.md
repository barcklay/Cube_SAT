# HAB-1 Lab — interactive 3D page (HW-145)

Full-screen Three.js scene: the payload in the stratosphere (sky and Earth by altitude),
balloon, parachute, the foam enclosure (concept HW-34) and the flight PCB from KiCad inside.
The flight is a model: two atmosphere profiles (tropical / ISA), 5 m/s ascent, burst at
7.9 m balloon diameter, parachute descent. Sensor limits and the barometric formula are
the real ones from the `hab_bringup` firmware.

- `index.html` — the page (open via any static server: `python3 -m http.server` here).
- `board.js` — the PCB as base64 GLB; rebuild after PCB changes with `./build_board.sh`.

Published copy: https://claude.ai/artifact/VP8YPhxGNXckYP5Keu93rK
