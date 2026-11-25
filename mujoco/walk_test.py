import mujoco, numpy as np, sys, json
sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
from make_model import build
from ctrl import StaticFF
def run(csv='/home/yagiz/Documents/H175_V2/mujoco/gait_ref.csv', verbose=True, gx=0.0, gv=0.0, gy=0.0, gyv=0.0, supmode='ref'):
    R=np.genfromtxt(csv,delimiter=',',names=True); t=R['t']
    m=build().compile(); d=mujoco.MjData(m)
    Kp=-m.actuator_biasprm[:,1].copy()
    jid=[m.actuator_trnid[i,0] for i in range(m.nu)]
    names=[m.joint(j).name for j in jid]; va=np.array([m.jnt_dofadr[j] for j in jid]); qa=np.array([m.jnt_qposadr[j] for j in jid])
    idx={n:i for i,n in enumerate(names)}
    sff=StaticFF(m,va); M=m.body_mass.sum()
    legj=[c for c in R.dtype.names if c[:2] in('L_','R_')]
    def ref(tk):
        qr=np.zeros(m.nu)
        for c in legj: qr[idx[c]]=np.interp(tk,t,R[c])
        return qr
    d.qpos[0]=R['px'][0]; d.qpos[1]=R['py'][0]; d.qpos[2]=R['pz'][0]+0.0005; d.qpos[3]=1
    q0=ref(0); d.qpos[qa]=q0; mujoco.mj_forward(m,d)
    ap=[idx['L_ankle_pitch'],idx['R_ankle_pitch']]; ar=[idx['L_ankle_roll'],idx['R_ankle_roll']]
    log=[]; dt=m.opt.timestep; fell=None
    for k in range(int(t[-1]/dt)):
        tk=d.time; qr=ref(tk)
        fz=(np.interp(tk,t,R['fLz']),np.interp(tk,t,R['fRz']))
        sup=(fz[0]<0.003, fz[1]<0.003)
        if supmode=='contact':
            F=[0.0,0.0]
            for ci in range(d.ncon):
                c=d.contact[ci]; b1=m.geom_bodyid[c.geom1]; b2=m.geom_bodyid[c.geom2]
                for si,sd in enumerate('LR'):
                    if m.body(f'{sd}_ankle_roll').id in (b1,b2) or m.body(f'{sd}_toe').id in (b1,b2):
                        f6=np.zeros(6); mujoco.mj_contactForce(m,d,ci,f6); F[si]+=abs(f6[0])
            sup=(F[0]>0.05*M*9.81, F[1]>0.05*M*9.81)
        ff=sff(d,sup)
        tau=ff.copy()
        if gx or gv or gy or gyv:
            com=d.subtree_com[1]; zx=np.interp(tk,t,R['zmpx']); zy=np.interp(tk,t,R['zmpy'])
            ex=com[0]-np.interp(tk,t,R['px'])-0.0; # (basit) pelvis planina gore degil CoM-ZMP
            ex=com[0]-zx; ey=com[1]-zy
            tau[ap]+= 0.5*M*9.81*(gx*ex+gv*d.qvel[0])*np.array(sup)
            tau[ar]+= 0.5*M*9.81*(gy*ey+gyv*d.qvel[1])*np.array(sup)
        d.ctrl[:]=(qr+tau/Kp)
        mujoco.mj_step(m,d)
        if k%250==0:
            log.append((d.time,d.qpos[0],d.qpos[1],d.qpos[2],np.interp(tk,t,R['px'])))
            if verbose: print(f't={d.time:.2f} x={d.qpos[0]:+.3f} (plan {R["px"][int(min(len(t)-1,tk/0.005))]:+.3f}) y={d.qpos[1]:+.3f} z={d.qpos[2]:.3f}')
        if d.qpos[2]<0.7: fell=d.time; print('DEVRILDI t=%.2f x=%.2f'%(fell,d.qpos[0])); break
    if fell is None: print('YURUDU, son x=%.3f z=%.3f'%(d.qpos[0],d.qpos[2]))
    return fell,np.array(log)
if __name__=='__main__': run()
