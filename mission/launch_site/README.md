# Launch site study (Thailand)

Picks launch points far from airports, military sites, land borders and the sea,
on flat ground, then checks where the balloon would land with the SondeHub/Tawhiri
predictor (GFS forecast).

## Rules used
- airports with IATA/ICAO code >= 50 km, other aerodromes >= 20 km (OpenStreetMap)
- military features (landuse=military, military=*) >= 25 km (OpenStreetMap, incomplete!)
- land border and coast >= 150 km (Natural Earth 10m countries)
- relief <= 25 m over an 8 x 8 km square (SRTM 90 m via opentopodata.org)

## Result (2026-09-28)
Chosen: **15.800 N 102.500 E** — Waeng Noi district, Khon Kaen, Khorat plateau.
Border/coast 178 km, airport 80 km, OSM military 52 km, relief 16 m, ground 186 m.
Alternatives: 15.55 N 101.10 E (Pa Sak valley, Si Thep), 16.0 N 100.55 E (Phichit plain).

The September forecasts drift 25–75 km west (see `pred.json`). February–March winds
differ: rerun `predict.py` a few days before the launch.

Before launch: permission from CAAT (unmanned free balloon), NOTAM, landowner of the
field; check the military list with local people — OSM is not complete.

## Rerun
    curl -A hab1 --data-urlencode data@overpass_query.txt https://overpass-api.de/api/interpreter -o osm.json
    curl -L -o countries.geojson https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_admin_0_countries.geojson
    python3 -m venv venv && venv/bin/pip install shapely numpy pyproj
    venv/bin/python grid.py && venv/bin/python flat.py && venv/bin/python predict.py
