import json, time, urllib.request, numpy as np
g=json.load(open('grid.json'))
ok=[r for r in g if r['air']>=50 and r['airmin']>=20 and r['mil']>=25 and min(r['border'],r['coast'])>=150]
print(len(ok),'candidates >=150 km from border/coast')
out=[]
# 3x3 samples, +-4 km, per candidate; batch 99 locations (11 candidates) per call
batch=[]
def run(batch):
    locs="|".join(f"{la:.4f},{lo:.4f}" for c in batch for la,lo in c['pts'])
    u="https://api.opentopodata.org/v1/srtm90m?locations="+locs
    r=json.load(urllib.request.urlopen(urllib.request.Request(u,headers={'User-Agent':'HAB-1-site-study'}),timeout=60))
    el=[x['elevation'] for x in r['results']]
    for i,c in enumerate(batch):
        e=np.array(el[i*9:(i+1)*9],dtype=float); c['elev']=float(e[4]); c['relief']=float(e.max()-e.min()); del c['pts']; out.append(c)
    time.sleep(1.1)
for r in ok:
    k=0.036; c=dict(r); c['pts']=[(r['lat']+a*k,r['lon']+b*k) for a in (-1,0,1) for b in (-1,0,1)]
    batch.append(c)
    if len(batch)==11: run(batch); batch=[]
if batch: run(batch)
json.dump(out,open('flat.json','w'))
flat=[c for c in out if c['relief']<=25]
print(len(flat),'flat (relief <= 25 m over 8x8 km)')
flat.sort(key=lambda r:-min(r['border'],r['coast']))
for r in flat[:20]: print({k:(round(v,1) if isinstance(v,float) else v) for k,v in r.items()})
