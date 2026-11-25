"""Yurume kontrolcusu v2: ataletsel ileri besleme + koni/ayak icinde kalan temas kuvvetleri + DCM geri beslemesi.
tau_ff = M qdd_ref + h  - J^T f ;  f >= 0 surtunme piramidinde, ayak koselerinde (NNLS).
Denge: ZMP kaymasi p = kdcm*(dcm - dcm_ref) istenen govde kuvvetine eklenir (ayak disina cikamaz)."""
import os, sys, json
import numpy as np, mujoco
from scipy.optimize import nnls
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from make_model import build

P=dict(kdcm=1.5, kroll=300.0, droll=40.0, kpitch=300.0, dpitch=40.0, kscale=2.0, mu=0.7,
       cx=(-0.055,0.095), cy=0.040, tn=True, mot=None, kstep=0.0, kyaw=0.0, dyaw=10.0)

class Walker:
    def __init__(s,name='gait_ref',**kw):
        s.p=dict(P,**kw); s.name=name
        R=name if isinstance(name,dict) else np.genfromtxt(f'{HERE}/{name}.csv',delimiter=',',names=True); s.R=R; s.t=t=R['t']
        cols_=list(R.keys()) if isinstance(R,dict) else R.dtype.names
        m=s.m=build().compile(); s.d=mujoco.MjData(m); s.d2=mujoco.MjData(m)
        jid=[m.actuator_trnid[i,0] for i in range(m.nu)]; s.names=[m.joint(j).name for j in jid]
        s.va=np.array([m.jnt_dofadr[j] for j in jid]); s.qa=np.array([m.jnt_qposadr[j] for j in jid])
        s.idx={n:i for i,n in enumerate(s.names)}
        for i,n in enumerate(s.names):          # bacak eklemlerinde daha sert PD (implicit, kararli)
            if any(k in n for k in ('hip','knee','ankle','waist')):
                m.actuator_gainprm[i,0]*=s.p['kscale']; m.actuator_biasprm[i,1:3]*=s.p['kscale']
        s.Kp=-m.actuator_biasprm[:,1]
        act=json.load(open('/home/yagiz/Documents/H175_V2/actuators.json'))['joints']
        s.Tpk=np.array([act[n]['tau_peak'] for n in s.names]); s.Wnl=np.array([act[n]['w_noload'] for n in s.names])
        if s.p['mot']:   # motor varsayimi degistirme: {'knee':(tpk,wnl), ...}
            for i,n in enumerate(s.names):
                k=n[2:] if n[:2] in ('L_','R_') else n
                if k in s.p['mot']:
                    s.Tpk[i],s.Wnl[i]=s.p['mot'][k]; m.actuator_forcerange[i]=[-s.Tpk[i],s.Tpk[i]]
        s.W=m.body_mass.sum()*9.81; s.M=m.body_mass.sum()
        s.feet=[(m.body(a+'_ankle_roll').id,m.body(a+'_toe').id) for a in 'LR']
        s.cor=np.array([[x,y,-0.08] for x in s.p['cx'] for y in (-s.p['cy'],s.p['cy'])])
        mu=s.p['mu']; s.rays=np.array([[mu,0,1],[-mu,0,1],[0,mu,1],[0,-mu,1]]).T
        # referans durum ve turevleri
        n=len(t); Q=np.zeros((n,m.nq)); Q[:,0]=R['px']; Q[:,1]=R['py']; Q[:,2]=R['pz']; Q[:,3]=1
        for c in cols_:
            if c[:2] in('L_','R_'): Q[:,s.qa[s.idx[c]]]=R[c]
        V=np.zeros((n,m.nv)); V[:,0:3]=np.gradient(Q[:,0:3],t,axis=0); V[:,s.va]=np.gradient(Q[:,s.qa],t,axis=0)
        A=np.gradient(V,t,axis=0)
        k=np.ones(9)/9; 
        for j in range(m.nv): A[:,j]=np.convolve(A[:,j],k,mode='same')
        s.Q,s.V,s.A=Q,V,A
        com=np.zeros((n,3))
        for i in range(n):
            s.d2.qpos[:]=Q[i]; mujoco.mj_kinematics(m,s.d2); mujoco.mj_comPos(m,s.d2); com[i]=s.d2.subtree_com[0]
        s.com=com; s.vcom=np.gradient(com,t,axis=0)
        s.reset()
    def reset(s):
        mujoco.mj_resetData(s.m,s.d); s.off=[np.zeros(2),np.zeros(2)]
        s.d.qpos[:]=s.Q[0]; s.d.qpos[2]+=0.0005; mujoco.mj_forward(s.m,s.d)
    def _i(s,A,tk):
        x=np.clip(tk/(s.t[1]-s.t[0]),0,len(s.t)-1.001); i=int(x); a=x-i
        return (1-a)*A[i]+a*A[i+1]
    def support(s,tk):
        return (np.interp(tk,s.t,s.R['fLz'])<0.002, np.interp(tk,s.t,s.R['fRz'])<0.002)
    def step(s):
        m,d,d2,p=s.m,s.d,s.d2,s.p; tk=d.time
        qr=s._i(s.Q,tk); d2.qpos[:]=qr; d2.qvel[:]=s._i(s.V,tk); d2.qacc[:]=s._i(s.A,tk)
        mujoco.mj_kinematics(m,d2); mujoco.mj_comPos(m,d2); mujoco.mj_comVel(m,d2)
        need=np.zeros(m.nv); mujoco.mj_rne(m,d2,1,need)
        need+=m.dof_armature*d2.qacc
        # DCM geri beslemesi
        mujoco.mj_forward(m,d) if False else mujoco.mj_subtreeVel(m,d)
        com=d.subtree_com[0]; vc=d.subtree_linvel[0]
        cr=s._i(s.com,tk); vr=s._i(s.vcom,tk); om=np.sqrt(9.81/cr[2])
        shift=p['kdcm']*((com[:2]+vc[:2]/om)-(cr[:2]+vr[:2]/om))
        F=np.array([*(-s.M*om**2*shift),0.0])
        need[0:3]+=F; need[3:6]+=np.cross(cr-qr[0:3],F)
        # temas kuvvetleri
        sup=s.support(tk); zx=np.interp(tk,s.t,s.R['zmpx']); zy=np.interp(tk,s.t,s.R['zmpy'])
        pos=[d2.xpos[s.feet[i][0]][:2] for i in range(2)]
        u=pos[0]-pos[1]; lamL=np.clip(np.dot(np.array([zx,zy])-pos[1],u)/np.dot(u,u),0,1)
        lam=[lamL,1-lamL]
        if not sup[0]: lam=[0,1]
        if not sup[1]: lam=[1,0]
        cols=[]; reg=[]; jp=np.zeros((3,m.nv))
        for i in range(2):
            if not sup[i]: continue
            b=s.feet[i][0]; Rm=d2.xmat[b].reshape(3,3)
            for c in s.cor:
                mujoco.mj_jac(m,d2,jp,None,d2.xpos[b]+Rm@c,b)
                cols.append(s.rays.T@jp)
                reg+= [1.0/np.sqrt(lam[i]+0.02)]*4
        tau=need[s.va].copy()
        if cols:
            G=np.vstack(cols).T                       # nv x nray
            wt=np.array([1,1,1,3,3,3.0])
            Aeq=wt[:,None]*G[:6]; beq=wt*need[:6]
            x,_=nnls(np.vstack([Aeq,0.02*np.diag(reg)]),np.concatenate([beq,np.zeros(len(reg))]),maxiter=300)
            tau-=(G@x)[s.va]
        # govde dik kalsin: destek kalcalari
        qw,qx,qy,qz=d.qpos[3:7]
        roll=np.arctan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy)); pitch=np.arcsin(np.clip(2*(qw*qy-qz*qx),-1,1))
        yaw=np.arctan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
        qcmd=qr[s.qa].copy()
        if p['kstep']:     # hiz hatasina gore salinan ayagin inis noktasini kaydir (yakalama noktasi)
            ev=(vc[:2]-vr[:2])/om; c_,sn=np.cos(yaw),np.sin(yaw); ex_,ey_=c_*ev[0]+sn*ev[1], -sn*ev[0]+c_*ev[1]
            tgt=np.clip(p['kstep']*np.array([ex_,ey_]),[-0.15,-0.08],[0.15,0.08])
            for i,a in enumerate('LR'):
                if not sup[i]: s.off[i]+=(tgt-s.off[i])*min(1.0,m.opt.timestep/0.04)
                dx,dy=s.off[i]/0.78
                qcmd[s.idx[a+'_hip_pitch']]-=dx; qcmd[s.idx[a+'_ankle_pitch']]+=dx
                qcmd[s.idx[a+'_hip_roll']]+=dy; qcmd[s.idx[a+'_ankle_roll']]-=dy
        for i,a in enumerate('LR'):
            if sup[i]:
                tau[s.idx[a+'_hip_yaw']]+=lam[i]*(p['kyaw']*yaw+p['dyaw']*d.qvel[5]*(p['kyaw']>0))
                tau[s.idx[a+'_hip_roll']] +=lam[i]*(p['kroll']*roll+p['droll']*d.qvel[3])
                tau[s.idx[a+'_hip_pitch']]+=lam[i]*(p['kpitch']*pitch+p['dpitch']*d.qvel[4])
        # konum + hiz referansi (aktuator sonumu referans hizina gore)
        Kd=-m.actuator_biasprm[:,2]
        d.ctrl[:]=qcmd+(tau+Kd*d2.qvel[s.va])/s.Kp
        if p['tn']:      # gercekci tork-hiz: motor modunda (tork ve hiz ayni yonde) tavan 0.5*wnl'den wnl'ye sifira iner; frenlemede tam tepe tork
            w_=d.qvel[s.va]; drive=s.Tpk*np.clip(2*(1-np.abs(w_)/s.Wnl),0,1)
            m.actuator_forcerange[:,1]=np.where(w_>0,drive,s.Tpk)
            m.actuator_forcerange[:,0]=-np.where(w_<0,drive,s.Tpk)
        mujoco.mj_step(m,d)
    def run(s,log=False):
        """Metrikler: dustu_t, x, plan_x, en buyuk destek ayagi egimi (derece), kayma (cm), motor doyma orani"""
        s.reset(); m,d=s.m,s.d; Tpk=s.Tpk
        tilt=0; slip=0; anchor=[None,None]; sat=np.zeros(m.nu); n=0; fell=None; tl=[]
        while d.time<s.t[-1]:
            tk=d.time; s.step()
            sup=s.support(tk); F=[0.,0.]
            for ci in range(d.ncon):
                c=d.contact[ci]; b={m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]}
                for i in (0,1):
                    if b&set(s.feet[i]): f6=np.zeros(6); mujoco.mj_contactForce(m,d,ci,f6); F[i]+=abs(f6[0])
            for i in range(2):
                if sup[i] and F[i]>0.2*s.W:
                    bid=s.feet[i][0]; pt=d.xpos[bid][:2].copy()
                    if anchor[i] is None: anchor[i]=pt
                    slip=max(slip,np.linalg.norm(pt-anchor[i]))
                    tilt=max(tilt,np.arccos(np.clip(d.xmat[bid].reshape(3,3)[2,2],-1,1)))
                else: anchor[i]=None
            sat+=np.abs(d.actuator_force)>0.995*Tpk; n+=1
            if d.qpos[2]<0.7: fell=d.time; break
        j=int(np.argmax(sat))
        return dict(dustu=None if fell is None else round(fell,2), x=round(float(d.qpos[0]),3), plan=round(float(s.R['px'][-1]),3),
                    egim=round(float(np.degrees(tilt)),1), kayma_cm=round(float(slip*100),1), doyma=(s.names[j],round(float(sat[j]/n),3)))
if __name__=='__main__':
    for nm in sys.argv[1:] or ['gait_ref']:
        print(nm,Walker(nm).run())
