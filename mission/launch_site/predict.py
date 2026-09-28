import json, time, urllib.request, datetime as dt
from shapely.geometry import shape, Point
from shapely.ops import transform, unary_union
from pyproj import Transformer
tr=Transformer.from_crs(4326,32647,always_xy=True).transform
C=json.load(open('countries.geojson'))
TH=transform(tr,shape([f for f in C['features'] if f['properties']['ADM0_A3']=='THA'][0]['geometry']))
d=json.load(open('osm.json'))['elements']
major=[];mil=[]
for e in d:
    t=e.get('tags',{}); c=e.get('center') or {'lat':e.get('lat'),'lon':e.get('lon')}
    p=Point(tr(c['lon'],c['lat']))
    if t.get('aeroway')=='aerodrome':
        if t.get('iata') or t.get('icao'): major.append(p)
    else: mil.append(p)
def check(lat,lon):
    p=Point(tr(lon,lat))
    return dict(inTH=TH.contains(p),edge=round(TH.boundary.distance(p)/1000),air=round(min(p.distance(q) for q in major)/1000),mil=round(min(p.distance(q) for q in mil)/1000))
CANDS={"A Si Thep (Pa Sak valley)":(15.55,101.10),"B Khorat plateau S of Khon Kaen":(16.0,102.4),"C Phichit plain":(16.0,100.55)}
now=dt.datetime.now(dt.timezone.utc).replace(hour=0,minute=0,second=0,microsecond=0)
res={}
for name,(la,lo) in CANDS.items():
    res[name]=[]
    for day in range(1,8):
        t=(now+dt.timedelta(days=day)).strftime('%Y-%m-%dT00:00:00Z')
        u=f"https://api.v2.sondehub.org/tawhiri?launch_latitude={la}&launch_longitude={lo}&launch_datetime={t}&ascent_rate=5&burst_altitude=30000&descent_rate=5"
        try:
            r=json.load(urllib.request.urlopen(urllib.request.Request(u,headers={'User-Agent':'HAB-1-site-study'}),timeout=60))
        except Exception as e:
            print(name,t,'ERR',e); continue
        pr=r['prediction']; asc=pr[0]['trajectory']; des=pr[1]['trajectory']
        bu=asc[-1]; la2=des[-1]; lon2=la2['longitude']-360 if la2['longitude']>180 else la2['longitude']
        dist=Point(tr(lo,la)).distance(Point(tr(lon2,la2['latitude'])))/1000
        c=check(la2['latitude'],lon2)
        res[name].append(dict(date=t[:10],land=(round(la2['latitude'],3),round(lon2,3)),km=round(dist),**c))
        print(name,t[:10],'land',round(la2['latitude'],3),round(lon2,3),f'{dist:.0f} km',c)
        time.sleep(0.5)
json.dump(res,open('pred.json','w'),indent=1)
