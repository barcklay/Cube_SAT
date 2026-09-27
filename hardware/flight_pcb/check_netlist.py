"""Check a KiCad netlist against the tables in gen_schematic.py.

Usage:
    kicad-cli sch export netlist --format kicadsexpr -o /tmp/hab1.net hab1_flight.kicad_sch
    python3 check_netlist.py /tmp/hab1.net gen_schematic.py
"""
import re, sys
netfile, gen = sys.argv[1], sys.argv[2]
t=open(netfile).read(); t=t[t.index("(nets"):]
nets={}
for blk in re.split(r'\n\t\t\(net\n', t)[1:]:
    name=re.search(r'\(name "([^"]*)"\)',blk).group(1).lstrip('/')
    nets[name]=re.findall(r'\(ref "([^"]+)"\)\s*\(pin "([^"]+)"\)(?:\s*\(pinfunction "([^"]*)"\))?',blk)
pin2net={(r,p):(n,f) for n,nodes in nets.items() for r,p,f in nodes}
src=open(gen).read(); ns={"__file__":gen}
exec(src.split("# --------------------------------------------------------------------------------------\n# S-expression helpers")[0], ns)
bad=0; mcu={}
for (r,p),(n,f) in pin2net.items():
    if r=="U1" and f:
        key=re.sub(r"_\d+$","",f).split('-')[0]
        mcu.setdefault(key,set()).add(n)
for pname,want in list(ns["MCU_NETS"].items())+list(ns["MCU_POWER"].items()):
    if mcu.get(pname)!={want}: print("MCU",pname,"want",want,"got",mcu.get(pname)); bad+=1
for part in ns["PARTS"]:
    for pin,want in part["pins"].items():
        if want is None: continue
        got=pin2net.get((part["ref"],pin),("<unconnected>",None))[0]
        if got!=want: print(part["ref"],pin,"want",want,"got",got); bad+=1
used=set(ns["MCU_NETS"])|set(ns["MCU_POWER"])
free=sorted(k for k,v in mcu.items() if k not in used)
print("nets: %d | MCU pins checked: %d | part pins checked: %d | mismatches: %d" % (
    len([n for n in nets if not n.startswith("unconnected")]), len(used),
    sum(1 for p in ns["PARTS"] for v in p["pins"].values() if v), bad))
print("single-pin nets:",[n for n,nd in nets.items() if len(nd)==1 and not n.startswith("unconnected")])
print("MCU pins left free:",", ".join(free))
