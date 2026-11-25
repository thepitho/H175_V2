"""best_mevcut_*.json cozumlerini siki dogrulama.  verify.py 1.40 1.60"""
import sys, json, numpy as np, mujoco, gait, multiprocessing as mp
from wbc import Walker
act=json.load(open('/home/yagiz/Documents/H175_V2/actuators.json'))['joints']
def load(v,SET='mevcut'):
    b=json.load(open('/home/yagiz/Documents/H175_V2/mujoco/best_%s_%.2f.json'%(SET,v)))[0]; return b['g'],b['w']
def measure(a):
    v,label,gk,wk,mod=a
    gk=dict(gk); wk=dict(wk)
    if 'steps' in mod: gk['nSteps']=mod['steps']
    R=gait.make(**gk); w=Walker(R,**wk); m,d=w.m,w.d
    if 'dt' in mod: m.opt.timestep=mod['dt']
    if 'fric' in mod: m.geom_friction[:,0]=mod['fric']
    if 'mass' in mod:     # yalniz fizik modeli: kontrolcu kutleyi bilmiyor -> ayri nominal kopya
        pm=Walker(R,**wk).m
        if 'dt' in mod: pm.opt.timestep=mod['dt']
        b=pm.body('torso').id; pm.body_mass[b]+=mod['mass']; mujoco.mj_setConst(pm,mujoco.MjData(pm))
        orig=mujoco.mj_step
        def st(mm,dd):
            pm.actuator_forcerange[:]=mm.actuator_forcerange; orig(pm,dd)
        mujoco.mj_step=st
    pel=m.body('pelvis').id; rng=m.jnt_range; lim=m.jnt_limited
    jid=[m.actuator_trnid[i,0] for i in range(m.nu)]
    T=[];V=[];X=[];t=[];fl=0;n=0;air=0;sw=0;mu=[];pen=0;qex={}; minsep=9
    if 'push' in mod: t0,F=mod['push']
    while d.time<w.t[-1] and d.qpos[2]>0.65:
        if 'push' in mod: d.xfrc_applied[pel,:3]=F if t0<=d.time<t0+0.2 else 0
        tk=d.time; w.step(); sup=w.support(tk); n+=1
        Fz=[0,0]
        for ci in range(d.ncon):
            c=d.contact[ci]; bs={m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]}
            for i in (0,1):
                if bs&set(w.feet[i]):
                    f6=np.zeros(6); mujoco.mj_contactForce(m,d,ci,f6); Fz[i]+=abs(f6[0]); pen=max(pen,-c.dist)
                    if f6[0]>50: mu.append(np.hypot(f6[1],f6[2])/f6[0])
        walking=1.5+gk['Ts']<tk<1.5+gk['Ts']*gk['nSteps']
        if walking:
            if Fz[0]<5 and Fz[1]<5: fl+=1
            T.append(d.actuator_force.copy()); V.append(d.qvel[w.va].copy()); X.append(d.qpos[0]); t.append(d.time)
            for i in range(2):
                if not sup[i]: sw+=1; air+=Fz[i]<5
            pL=d.xpos[w.feet[0][0]]; pR=d.xpos[w.feet[1][0]]
            if abs(pL[0]-pR[0])<0.2: minsep=min(minsep,pL[1]-pR[1])
            for j in jid:
                if lim[j]:
                    q=d.qpos[m.jnt_qposadr[j]]; ex=max(rng[j,0]-q,q-rng[j,1],0)
                    if ex>0.01: nm=m.joint(j).name; qex[nm]=max(qex.get(nm,0),round(float(np.degrees(ex)),1))
    if 'mass' in mod: mujoco.mj_step=orig
    ok=bool(d.qpos[2]>0.65)
    out=dict(v=v,label=label,ok=ok,t=round(float(d.time),1),x=round(float(d.qpos[0]),2),plan=round(float(R['px'][-1]),2))
    if len(X)>10:
        T=np.array(T);V=np.array(V)
        names=w.names; body=[i for i,k in enumerate(names) if not any(s in k for s in('index','middle','ring','little','thumb'))]
        pk=np.abs(T).max(0)/w.Tpk; rms=np.sqrt((T**2).mean(0))/np.array([act[k]['tau_rated'] for k in names]); sp=np.abs(V).max(0)/w.Wnl
        top=lambda a,k=4:[(names[i],round(float(a[i]),2)) for i in sorted(body,key=lambda i:-a[i])[:k]]
        out.update(hiz=round(float((X[-1]-X[0])/(t[-1]-t[0])),2), ucus_pct=round(100*fl/len(X),1), salinim_havada_pct=round(100*air/max(sw,1),0),
                   tork_tepe=top(pk), rms_anma=top(rms), hiz_wnl=top(sp), mu95=round(float(np.percentile(mu,95)),2), penetrasyon_mm=round(float(pen*1000),1),
                   eklem_siniri_asimi_derece=qex, ayak_min_yanal_aralik_cm=round(float(minsep*100),1),
                   guc_ort_W=round(float(np.abs(T*V).sum(1).mean())), guc_tepe_W=round(float(np.abs(T*V).sum(1).max())))
    return out
if __name__=='__main__':
    jobs=[]
    for v in [float(a) for a in sys.argv[1:]]:
        gk,wk=load(v)
        mods=[('nominal',{}),('uzun 60 adim',{'steps':60}),('dt 0.5ms',{'dt':0.0005}),('surtunme 0.8',{'fric':0.8}),('surtunme 0.6',{'fric':0.6}),
              ('govde +3kg',{'mass':3}),('govde +7kg',{'mass':7})]+[('itme %s%d N'%(nm,abs(sum(F))),{'push':(3.5,F)}) for nm,F in (('+x ',[100,0,0]),('-x ',[-100,0,0]),('+y ',[0,100,0]),('-y ',[0,-100,0]))]
        jobs+=[(v,l,gk,wk,md) for l,md in mods]
        print('v=%.2f ayar'%v,{k:round(x,3) for k,x in gk.items()},{k:round(x,2) for k,x in wk.items() if k!='mot'})
    with mp.Pool(10) as pool: res=pool.map(measure,jobs,chunksize=1)
    for r in res:
        if r['label']=='nominal': print(json.dumps(r,ensure_ascii=False,default=lambda o:o.item()))
        else: print(r['v'],r['label'],'OK' if r['ok'] else 'DUSTU t=%s'%r['t'],'x=%s/%s'%(r['x'],r['plan']),'hiz',r.get('hiz'))
    json.dump(res,open('/home/yagiz/Documents/H175_V2/mujoco/verify.json','w'),indent=1,ensure_ascii=False,default=lambda o:o.item())
