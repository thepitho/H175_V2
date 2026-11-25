"""Basarili/basarisiz yuruyuslerde ilk bozulan sinyal: destek fazi, temas kuvveti, kayma, egim, takip hatasi."""
import sys, numpy as np, mujoco, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
from make_model import build
from ctrl import StaticFF
D='/home/yagiz/Documents/H175_V2/mujoco/'
def sim(f,GY=-3.0,GYV=-1.0,clamp=None):
    R=np.genfromtxt(D+f+'.csv',delimiter=',',names=True); t=R['t']
    m=build().compile(); d=mujoco.MjData(m); Kp=-m.actuator_biasprm[:,1]
    jid=[m.actuator_trnid[i,0] for i in range(m.nu)]; names=[m.joint(j).name for j in jid]
    va=np.array([m.jnt_dofadr[j] for j in jid]); qa=np.array([m.jnt_qposadr[j] for j in jid]); idx={n:i for i,n in enumerate(names)}
    sff=StaticFF(m,va); M=m.body_mass.sum(); W=M*9.81
    legj=[c for c in R.dtype.names if c[:2] in('L_','R_')]; ar=[idx['L_ankle_roll'],idx['R_ankle_roll']]
    def ref(tk):
        q=np.zeros(m.nu)
        for c in legj: q[idx[c]]=np.interp(tk,t,R[c])
        return q
    I=lambda c,tk: np.interp(tk,t,R[c])
    d.qpos[:3]=[R['px'][0],R['py'][0],R['pz'][0]+0.0005]; d.qpos[3]=1; d.qpos[qa]=ref(0); mujoco.mj_forward(m,d)
    feet=[(m.body(s+'_ankle_roll').id,m.body(s+'_toe').id) for s in 'LR']
    L={k:[] for k in 't supL supR FL FR slipL slipR tiltL tiltR ex ey sat'.split()}
    anchor=[None,None]; fell=None
    for k in range(int(t[-1]/m.opt.timestep)):
        tk=d.time; sup=(I('fLz',tk)<0.003,I('fRz',tk)<0.003)
        tau=sff(d,sup); ey_=d.subtree_com[1][1]-I('zmpy',tk)
        fb=0.5*W*(GY*ey_+GYV*d.qvel[1])
        if clamp is not None: fb=np.clip(fb,-clamp*W,clamp*W)
        tau[ar]+=fb*np.array(sup)
        d.ctrl[:]=ref(tk)+tau/Kp; mujoco.mj_step(m,d)
        F=[0.,0.]
        for ci in range(d.ncon):
            c=d.contact[ci]; b={m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]}
            for s in (0,1):
                if b&set(feet[s]): f6=np.zeros(6); mujoco.mj_contactForce(m,d,ci,f6); F[s]+=abs(f6[0])
        if k%5==0:
            L['t'].append(d.time)
            for s,n in enumerate('LR'):
                p=d.xpos[feet[s][0]][:2].copy(); loaded=sup[s] and F[s]>0.2*W
                if not loaded: anchor[s]=None; slip=0.
                else:
                    if anchor[s] is None: anchor[s]=p
                    slip=np.linalg.norm(p-anchor[s])
                zax=d.xmat[feet[s][0]].reshape(3,3)[:,2]
                L['sup'+n].append(sup[s]); L['F'+n].append(F[s]/W); L['slip'+n].append(slip)
                L.setdefault('roll'+n,[]).append(abs(np.arcsin(np.clip(zax[1],-1,1))) if sup[s] else 0.)
                L.setdefault('pitch'+n,[]).append(abs(np.arcsin(np.clip(zax[0],-1,1))) if sup[s] else 0.)
                L['tilt'+n].append(np.arccos(np.clip(zax[2],-1,1)) if sup[s] else 0.)
            L['ex'].append(d.qpos[0]-I('px',tk)); L['ey'].append(d.qpos[1]-I('py',tk))
            L['sat'].append(max(abs(d.actuator_force[i]) for i in ar)>52.9)
        if d.qpos[2]<0.7: fell=d.time; break
    return {k:np.array(v) for k,v in L.items()},fell,R
if __name__=='__main__':
    runs={'0.12 m/s Ts1.0 (basarili)':'gait_ref','0.25 m/s Ts0.8':'g25','0.12 m/s Ts0.8':'gB','0.25 m/s Ts1.0':'gA'}
    res={n:sim(f) for n,f in runs.items()}
    base=res['0.12 m/s Ts1.0 (basarili)'][0]
    sig={'yan hata |ey|':lambda L:abs(L['ey']),'ileri hata |ex|':lambda L:abs(L['ex']),
         'kayma L':lambda L:L['slipL'],'kayma R':lambda L:L['slipR'],'taban egimi L':lambda L:L['tiltL'],'taban egimi R':lambda L:L['tiltR']}
    thr={n:1.3*f(base).max() for n,f in sig.items()}
    print('basarili kosu tepe degerleri:',{n:round(float(f(base).max()),3) for n,f in sig.items()}, 'ankle_roll doyma orani %.3f'%base['sat'].mean())
    for name,(L,fell,R) in res.items():
        print('\n==',name,'dustu t=%.2f'%fell if fell else 'dusmedi')
        ev=[]
        for n,f in sig.items():
            i=np.where(f(L)>thr[n])[0]
            if len(i): ev.append((L['t'][i[0]],n+' > %.3f'%thr[n]))
        # yuk uyumsuzlugu: planli destekte <10% agirlik (>40 ms)
        for s in 'LR':
            bad=L['sup'+s]&(L['F'+s]<0.1); run=0
            for i,b in enumerate(bad):
                run=run+1 if b else 0
                if run>=4: ev.append((L['t'][i-3],'planli destek %s yuksuz'%s)); break
        i=np.where(L['sat'])[0]
        if len(i): ev.append((L['t'][i[0]],'ankle_roll ilk doyma (doyma orani %.3f)'%L['sat'].mean()))
        T0=1.5; Ts=np.nan
        for tm,n in sorted(ev)[:7]: print('  t=%.2f  %s'%(tm,n))
    fig,ax=plt.subplots(5,4,figsize=(18,12),sharex='col')
    for c,(name,(L,fell,R)) in enumerate(res.items()):
        t=L['t']; ax[0,c].set_title(name+(' (dustu %.1f s)'%fell if fell else ''))
        for s,col in (('L','C0'),('R','C1')):
            ax[0,c].plot(t,L['F'+s],col,lw=.8,label=s+' kuvvet/agirlik'); ax[0,c].plot(t,L['sup'+s]*1.0,col+'--',lw=.8)
            ax[1,c].plot(t,L['slip'+s]*100,col); ax[2,c].plot(t,np.degrees(L['tilt'+s]),col)
        ax[3,c].plot(t,L['ey']*100,'C3'); ax[3,c].plot(t,L['ex']*100,'C2'); ax[4,c].plot(t,L['sat']*1.0,'k'); ax[4,c].set_xlabel('t [s]')
        ax[0,c].set_ylim(-.1,2); ax[1,c].set_ylim(0,6); ax[2,c].set_ylim(0,15); ax[3,c].set_ylim(-15,15)
    for r,l in enumerate(['Fz/Mg (kesikli: plan)','destek kaymasi [cm]','taban egimi [derece]','takip hatasi [cm] (kirmizi y, yesil x)','ankle_roll doyma']): ax[r,0].set_ylabel(l)
    ax[0,0].legend(fontsize=7); plt.tight_layout(); plt.savefig(D+'diag_walk.png',dpi=90)
