"""Hiz basina yuruyus parametresi taramasi (paralel).  sweep.py 0.6 0.8 [--n 48]"""
import sys, itertools, json, numpy as np, multiprocessing as mp
def one(a):
    gk,wk=a
    import gait; from wbc import Walker
    try:
        R=gait.make(**gk); w=Walker(R,**wk); r=w.run()
    except Exception as e:
        return dict(err=str(e),g=gk,w=wk,dist=0,ok=False,egim=99)
    ok=r['dustu'] is None
    return dict(g=gk,w=wk,ok=ok,dist=r['x']/r['plan'] if ok else (r['dustu']/R['t'][-1])*0.5,egim=r['egim'],kayma=r['kayma_cm'],x=r['x'],plan=r['plan'],dustu=r['dustu'])
if __name__=='__main__':
    rng=np.random.default_rng(0); N=int(sys.argv[sys.argv.index('--n')+1]) if '--n' in sys.argv else 96
    speeds=[float(v) for v in sys.argv[1:] if v.replace('.','').isdigit() and '.' in v]
    with mp.Pool(max(1,mp.cpu_count()-2)) as pool:
        for v in speeds:
            jobs=[]
            for _ in range(N):
                c=lambda a: a[rng.integers(len(a))].item()
                gk=dict(vx=v,Ts=c(np.array([0.4,0.45,0.5,0.55,0.6,0.7])),dsr=c(np.array([0.1,0.15,0.2,0.3])),zh=c(np.array([0.76,0.78,0.80,0.82])),
                        stepH=c(np.array([0.04,0.06,0.08])),heel=c(np.array([0.0,0.05,0.1])),widthz=c(np.array([0.8,1.0])),nSteps=int(max(14,round(6/(v*0.55)))))
                wk=dict(kdcm=c(np.array([1.0,1.5,2.5])),kscale=c(np.array([1.0,2.0])))
                jobs.append((gk,wk))
            res=pool.map(one,jobs,chunksize=1)
            res.sort(key=lambda r:(r['ok'] and r['egim']<10, r['ok'], r['dist'], -r['egim']),reverse=True)
            print('v=%.2f: %d/%d dusmedi; temiz(egim<10) %d'%(v,sum(r['ok'] for r in res),N,sum(r['ok'] and r['egim']<10 for r in res)))
            for r in res[:3]: print('   ',{k:r.get(k) for k in('ok','x','plan','egim','kayma','dustu','err')},r['g'],r['w'])
            json.dump(res[:10],open('sweep_%.2f.json'%v,'w'),indent=1)
