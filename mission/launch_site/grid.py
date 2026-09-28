import json, numpy as np
from shapely.geometry import shape, Point
from shapely.ops import transform
from pyproj import Transformer
from shapely import STRtree
# Thailand in UTM 47N-ish metric projection (EPSG:32647) for distances in km
tr=Transformer.from_crs(4326,32647,always_xy=True).transform
C=json.load(open('countries.geojson'))
th=[f for f in C['features'] if f['properties']['ADM0_A3']=='THA'][0]
TH=transform(tr,shape(th['geometry']))
edge=TH.boundary
# land borders only (not coast): shared edges with neighbours
nb=[transform(tr,shape(f['geometry'])) for f in C['features'] if f['properties']['ADM0_A3'] in ('MMR','LAO','KHM','MYS')]
land_border=edge.intersection(__import__('shapely').ops.unary_union([n.buffer(2000) for n in nb]))
d=json.load(open('osm.json'))['elements']
major=[];minor=[];mil=[]
for e in d:
    t=e.get('tags',{}); c=e.get('center') or {'lat':e.get('lat'),'lon':e.get('lon')}
    p=Point(tr(c['lon'],c['lat']))
    if t.get('aeroway')=='aerodrome': (major if (t.get('iata') or t.get('icao')) else minor).append(p)
    else: mil.append(p)
def mind(pts,p): return min(p.distance(q) for q in pts)/1000
res=[]
for lat in np.arange(5.8,20.5,0.1):
    for lon in np.arange(97.4,105.7,0.1):
        p=Point(tr(lon,lat))
        if not TH.contains(p): continue
        r=dict(lat=round(lat,2),lon=round(lon,2),border=land_border.distance(p)/1000,coast=edge.distance(p)/1000,
               air=mind(major,p),airmin=mind(minor,p),mil=mind(mil,p))
        res.append(r)
json.dump(res,open('grid.json','w'))
ok=[r for r in res if r['air']>=50 and r['airmin']>=20 and r['mil']>=25]
print(len(res),'land points,',len(ok),'pass airport/military filters')
ok.sort(key=lambda r:-min(r['border'],r['coast']))
for r in ok[:25]: print({k:(round(v,1) if isinstance(v,float) else v) for k,v in r.items()})
