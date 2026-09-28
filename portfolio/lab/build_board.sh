#!/bin/sh
# Rebuild board.js (the flight PCB as a base64 GLB) from the KiCad board.
# The page loads board.js instead of board.glb because the artifact host does not serve .glb.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
KICAD=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
TMP=$(mktemp -d)
"$KICAD" pcb export glb --subst-models --include-tracks --include-pads \
    --include-silkscreen --include-soldermask \
    -o "$TMP/board.glb" "$HERE/../../hardware/flight_pcb/hab1_flight.kicad_pcb"
python3 -c "import base64,sys; print('window.BOARD_GLB_B64=\"%s\";' % base64.b64encode(open(sys.argv[1],'rb').read()).decode())" \
    "$TMP/board.glb" > "$HERE/board.js"
rm -rf "$TMP"
echo "board.js: $(wc -c < "$HERE/board.js") bytes"
