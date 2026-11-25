"""Aday motor setleriyle sartname hizi taramasi:  sweep2.py SET v... --n N"""
import sys, json, numpy as np, multiprocessing as mp
SETS={'mevcut':None,
 # yalniz hiz artisi (tork ayni)
 'hizli':{'hip_pitch':(200,18),'knee':(200,22),'ankle_pitch':(120,16)},
 # hiz + tork artisi
 'guclu':{'hip_pitch':(250,20),'knee':(300,22),'ankle_pitch':(180,16),'hip_roll':(150,10),'hip_yaw':(90,20),'ankle_roll':(70,20)}}
def one(a):
    gk,wk=a
    import gait; from wbc import Walker
    try:
        R=gait.make(**gk); r=Walker(R,**wk).run()
    except Exception as e: return dict(err=str(e),g=gk,w=wk,ok=False,dist=0,egim=99)
    ok=r['dustu'] is None
    return dict(g=gk,w=wk,ok=ok,dist=r['x']/r['plan'] if ok else 0.5*r['dustu']/R['t'][-1],egim=r['egim'],kayma=r['kayma_cm'],x=r['x'],plan=r['plan'],dustu=r['dustu'])
if __name__=='__main__':
    name=sys.argv[1]; N=int(sys.argv[sys.argv.index('--n')+1])
    speeds=[float(v) for v in sys.argv[2:] if '.' in v]
    rng=np.random.default_rng(int(sys.argv[sys.argv.index('--seed')+1]) if '--seed' in sys.argv else 1)
    with mp.Pool(10) as pool:
        for v in speeds:
            jobs=[]
            for _ in range(N):
                c=lambda a: a[rng.integers(len(a))].item()
                gk=dict(vx=v,Ts=c(np.array([0.3,0.35,0.4,0.45])),dsr=c(np.array([0.05,0.1,0.15])),zh=c(np.array([0.74,0.76,0.78,0.80])),
                        stepH=c(np.array([0.04,0.06,0.08])),heel=c(np.array([0.0,0.05,0.1,0.15])),widthz=c(np.array([0.7,0.8,0.9,1.0])),arm=c(np.array([0.0,0.6])),nSteps=int(round(8/(v*0.38))))
                wk=dict(kdcm=c(np.array([0.5,1.0,1.5,2.5])),kscale=c(np.array([1.0,2.0])),kpitch=c(np.array([300.,800.])),kroll=c(np.array([300.,800.])),mot=SETS[name])
                jobs.append((gk,wk))
            res=pool.map(one,jobs,chunksize=1)
            res.sort(key=lambda r:(r['ok'] and r['egim']<10,r['ok'],r['dist']),reverse=True)
            print('[%s] v=%.2f: %d/%d dusmedi, temiz %d'%(name,v,sum(r['ok'] for r in res),N,sum(r['ok'] and r['egim']<10 for r in res)),flush=True)
            for r in res[:2]: print('   ',{k:r.get(k) for k in('ok','x','plan','egim','kayma','dustu','err')},r['g'],{k:v for k,v in r['w'].items() if k!='mot'},flush=True)
            json.dump(res[:20],open('sweep_%s_%.2f.json'%(name,v),'w'),indent=1)
