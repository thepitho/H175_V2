"""Sartname hizlari icin kisitli evrimsel optimizasyon (gercekci tork-hiz, anma torku, eklem siniri, uzun kosu).
opt.py SET hiz --min dakika  [--from dosya.json ...]  -> opt_SET_hiz.json"""
import sys, json, glob, time, numpy as np, mujoco, multiprocessing as mp
from sweep2 import SETS
from refine import RNG, WR
WR=dict(WR,kstep=(0.0,2.0),kyaw=(0.0,600.0)); RNG=dict(RNG,Ts=(0.28,0.65),zh=(0.72,0.82))
DIST=12.0
act=json.load(open('/home/yagiz/Documents/H175_V2/actuators.json'))['joints']
_rated=None
def evaluate(a):
    gk,wk=a
    import gait; from wbc import Walker
    try:
        R=gait.make(**gk); w=Walker(R,**wk); m,d=w.m,w.d
        mot=wk.get('mot') or {}
        rated=np.array([act[n]['tau_rated']*(w.Tpk[i]/act[n]['tau_peak']) for i,n in enumerate(w.names)])
        jid=np.array([m.actuator_trnid[i,0] for i in range(m.nu)]); lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
        sq=np.zeros(m.nu); n=0; ex=0.0; flight=0; wmax=np.zeros(m.nu)
        while d.time<w.t[-1] and d.qpos[2]>0.65:
            w.step()
            if d.time>1.5:
                f=d.actuator_force; sq+=f*f; n+=1
                q=d.qpos[w.qa]; ex=max(ex,float(np.max(np.maximum(lo-q,q-hi))))
                wmax=np.maximum(wmax,np.abs(d.qvel[w.va])/w.Wnl)
        ok=bool(d.qpos[2]>0.65)
        rms=float(np.max(np.sqrt(sq/max(n,1))/rated)); x=float(d.qpos[0]); plan=float(R['px'][-1])
        prog=(float(d.time)/float(R['t'][-1])) if not ok else 1.0
        sc=prog if not ok else 2.0-abs(1-x/plan)*3-abs(float(d.qpos[1]))*0.05
        sc-=0.5*max(0,rms-1.0)+1.0*max(0,ex)+0.1*max(0,float(wmax.max())-1.0)
        return dict(g=gk,w=wk,ok=ok,sc=sc,x=round(x,2),plan=round(plan,2),y=round(float(d.qpos[1]),2),t=round(float(d.time),1),rms=round(rms,2),asim_deg=round(float(np.degrees(max(ex,0))),1),wmax=round(float(wmax.max()),2))
    except Exception as e:
        return dict(g=gk,w=wk,ok=False,sc=-9,err=str(e))
def child(rng,a,b,v,sc,mot):
    g={}; w={}
    dflt=dict(droll=40,dpitch=40,mu=0.7,arm=0.0,kpitch=300,kroll=300,kstep=0.3,kyaw=100,kdcm=1.5,kscale=1.0,heel=0.05,widthz=0.85,Ts=0.4,dsr=0.1,zh=0.78,stepH=0.06)
    for D,A,B,R in ((g,a['g'],b['g'],RNG),(w,a['w'],b['w'],WR)):
        for k,(lo,hi) in R.items():
            va=A.get(k,dflt[k]); vb=B.get(k,dflt[k])
            val=va if rng.random()<0.7 else vb
            if rng.random()<0.5: val+=rng.normal()*sc*(hi-lo)
            D[k]=float(np.clip(val,lo,hi))
    g['vx']=v; g['nSteps']=int(round(DIST/(v*g['Ts']))); w['mot']=SETS[mot]
    return (g,w)
if __name__=='__main__':
    name,v=sys.argv[1],float(sys.argv[2]); mins=float(sys.argv[sys.argv.index('--min')+1])
    rng=np.random.default_rng(int(v*100)+len(name))
    seeds=[]
    for f in glob.glob('best_*.json')+glob.glob('sweep_*.json')+glob.glob('opt_*.json'):
        for r in json.load(open(f))[:6]:
            if abs(r['g']['vx']-v)<=0.41: seeds.append(r)
    t0=time.time()
    with mp.Pool(11) as pool:
        pop=pool.map(evaluate,[child(rng,s,s,v,0.02 if k%2 else 0.1,name) for k,s in enumerate([seeds[i] for i in rng.permutation(len(seeds))[:110]])],chunksize=1)
        gen=0
        while time.time()-t0<mins*60:
            pop.sort(key=lambda r:r['sc'],reverse=True); pop=pop[:24]
            kids=[child(rng,pop[rng.integers(len(pop))],pop[rng.integers(len(pop))],v,rng.choice([0.03,0.08,0.2]),name) for _ in range(44)]
            pop+=pool.map(evaluate,kids,chunksize=1); gen+=1
            pop.sort(key=lambda r:r['sc'],reverse=True)
            if gen%3==0 or time.time()-t0>=mins*60:
                b=pop[0]; print('[%s %.1f] nesil %d basarili %d | sc=%.2f ok=%s x=%s/%s rms/anma=%s asim=%s hiz/wnl=%s'%(name,v,gen,sum(r['ok'] for r in pop[:24]),b['sc'],b['ok'],b.get('x'),b.get('plan'),b.get('rms'),b.get('asim_deg'),b.get('wmax')),flush=True)
                json.dump(pop[:24],open('opt_%s_%.2f.json'%(name,v),'w'),indent=1)
