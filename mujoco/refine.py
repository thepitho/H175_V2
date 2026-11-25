"""En iyi ayarlarin cevresinde evrimsel arama:  refine.py SET hiz_kaynak hiz_hedef [--gen 6] -> best_SET_hiz.json"""
import sys, json, numpy as np, multiprocessing as mp
from sweep2 import one, SETS
RNG=dict(Ts=(0.28,0.6),dsr=(0.03,0.25),zh=(0.74,0.82),stepH=(0.035,0.09),heel=(0.0,0.18),widthz=(0.65,1.0),arm=(0.0,1.0))
WR=dict(kdcm=(0.3,3.5),kscale=(0.8,2.5),kpitch=(100,1200),kroll=(100,1200),droll=(10,120),dpitch=(10,120),mu=(0.5,0.9))
def mutate(rng,b,v,sc):
    g=dict(b['g']); w=dict(b['w'])
    for D,R in ((g,RNG),(w,WR)):
        for k,(lo,hi) in R.items():
            val=D.get(k,{'droll':40,'dpitch':40,'mu':0.7,'arm':0.0,'kpitch':300,'kroll':300}.get(k,(lo+hi)/2))
            if rng.random()<0.5: val+=rng.normal()*sc*(hi-lo)
            D[k]=float(np.clip(val,lo,hi))
    g['vx']=v; g['nSteps']=int(round(8/(v*g['Ts']))); return (g,w)
def score(r):
    if not r['ok']: return r['dist']
    return 2.0 - min(r['egim'],30)/40 - min(r['kayma'],5)/10 - abs(1-r['x']/r['plan'])
if __name__=='__main__':
    name,src,dst=sys.argv[1],float(sys.argv[2]),float(sys.argv[3]); gens=int(sys.argv[sys.argv.index('--gen')+1]) if '--gen' in sys.argv else 6
    try: pop=json.load(open('best_%s_%.2f.json'%(name,src)))
    except FileNotFoundError: pop=json.load(open('sweep_%s_%.2f.json'%(name,src)))
    for p in pop: p['w']['mot']=SETS[name]
    rng=np.random.default_rng(int(dst*100))
    with mp.Pool(10) as pool:
        if src!=dst: pop=pool.map(one,[mutate(rng,p,dst,0.03) for p in pop[:8] for _ in range(5)],chunksize=1)
        for g in range(gens):
            pop.sort(key=score,reverse=True); par=pop[:8]
            kids=pool.map(one,[mutate(rng,par[rng.integers(len(par))],dst,0.12 if g<gens//2 else 0.06) for _ in range(40)],chunksize=1)
            pop=sorted(pop+kids,key=score,reverse=True)[:20]
            b=pop[0]; print('[%s %.2f] nesil %d: basarili %d/20  en iyi ok=%s x=%s/%s egim=%s kayma=%s'%(name,dst,g,sum(p['ok'] for p in pop),b['ok'],b.get('x'),b.get('plan'),b.get('egim'),b.get('kayma')),flush=True)
            json.dump(pop,open('best_%s_%.2f.json'%(name,dst),'w'),indent=1)
