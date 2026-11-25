"""h175_gait.m'nin Python hali + tam govde ZMP duzeltmesi (Kajita ikinci asama) + topuk-parmak ZMP ilerlemesi."""
import numpy as np, mujoco, json
from scipy.linalg import solve_discrete_are
from make_model import build, ROOT
G=9.81; HIPY=0.105; ZHIP=0.045; L1=0.415; L2=0.385; ZANK=0.080
DEF=dict(vx=0.6,Ts=0.6,dsr=0.2,zh=0.82,stepH=0.05,nSteps=14,T0=1.5,T1=1.5,dt=0.005,iters=3,heel=0.0,widthz=1.0,arm=0.0)
_M=None
def model():
    global _M
    if _M is None: _M=build().compile()
    return _M

def preview(r,zc,dt):
    Np=int(round(1.6/dt))
    A=np.array([[1,dt,dt*dt/2],[0,1,dt],[0,0,1.]]); B=np.array([[dt**3/6],[dt*dt/2],[dt]]); C=np.array([[1,0,-zc/G]])
    Ab=np.block([[np.eye(1),C@A],[np.zeros((3,1)),A]]); Bb=np.vstack([C@B,B]); Ib=np.array([[1],[0],[0],[0.]])
    Q=np.zeros((4,4)); Q[0,0]=1; R=np.array([[1e-6]])
    P=solve_discrete_are(Ab,Bb,Q,R); inv=1/(R+Bb.T@P@Bb)[0,0]
    K=inv*(Bb.T@P@Ab); Ke=K[0,0]; Kx=K[0,1:]; Ac=Ab-Bb@K
    Gd=np.zeros(Np); Gd[0]=-Ke; X=-Ac.T@P@Ib
    for j in range(1,Np): Gd[j]=inv*(Bb.T@X)[0,0]; X=Ac.T@X
    n=len(r); rr=np.concatenate([r,np.full(Np,r[-1])]); x=np.zeros(3); se=0; out=np.zeros(n); Bv=B[:,0]; Cv=C[0]
    for k in range(n):
        se+=Cv@x-r[k]; u=-Ke*se-Kx@x-Gd@rr[k+1:k+Np+1]
        x=A@x+Bv*u; out[k]=x[0]
    return out

def ik(pel,fx,fz,side):
    k=1 if side=='L' else -1; n=len(fx)
    ft=np.stack([fx,np.full(n,k*HIPY),ZANK+fz],1); hip=pel+np.array([0,k*HIPY,-ZHIP]); r=ft-hip
    D=np.minimum(np.linalg.norm(r,axis=1),L1+L2-2e-3)
    kap=np.arccos(np.clip((D**2-L1**2-L2**2)/(2*L1*L2),-1,1))
    ax=-L2*np.sin(kap); az=-(L1+L2*np.cos(kap))
    phi=np.arcsin(np.clip(r[:,1]/np.abs(az),-1,1))
    th=np.arctan2(r[:,0],r[:,2])-np.arctan2(ax,np.cos(phi)*az); th=np.arctan2(np.sin(th),np.cos(th))
    beta=np.arctan2(-np.sin(th),np.cos(phi)*np.cos(th)); rho=np.arcsin(np.clip(-np.sin(phi)*np.cos(th),-1,1))
    return {side+'_hip_pitch':th,side+'_hip_roll':phi,side+'_knee':kap,side+'_ankle_pitch':beta-kap,side+'_ankle_roll':rho}

def make(**kw):
    o=dict(DEF,**kw); m=model(); d=mujoco.MjData(m)
    dt=o['dt']; Ts=o['Ts']; N=o['nSteps']; L=o['vx']*Ts; T0=o['T0']; dsr=o['dsr']
    t=np.arange(int(round((T0+N*Ts+o['T1'])/dt))+1)*dt; n=len(t)
    landX=(np.arange(1,N+1)-0.5)*L; landX[-1]=landX[-2]
    supY=lambda i: (-1 if i%2==0 else 1)*HIPY*o['widthz']      # i 0-tabanli: cift indeks -> sag destek
    suppX=np.zeros(N); sx={'L':0.,'R':0.}
    for i in range(N):
        sw,st=('L','R') if i%2==0 else ('R','L'); suppX[i]=sx[st]; sx[sw]=landX[i]
    zmp=np.zeros((n,2))
    sm3=lambda a:a*a*(3-2*a)
    for k,tk in enumerate(t):
        if tk<T0: continue
        if tk<T0+N*Ts:
            i=int((tk-T0)//Ts); ph=(tk-T0-i*Ts)/Ts
            cur=np.array([suppX[i]-o['heel']/2,supY(i)])
            prv=np.zeros(2) if i==0 else np.array([suppX[i-1]+o['heel']/2,supY(i-1)])
            if ph<dsr: a=sm3(ph/dsr); zmp[k]=(1-a)*prv+a*cur
            else: zmp[k]=cur+np.array([o['heel']*(ph-dsr)/(1-dsr),0])
        else:
            a=sm3(min((tk-(T0+N*Ts))/0.6,1)); cur=np.array([suppX[-1]+o['heel']/2,supY(N-1)]); zmp[k]=(1-a)*cur+a*np.array([landX[-1],0])
    f={'L':[np.zeros(n),np.zeros(n)],'R':[np.zeros(n),np.zeros(n)]}; pos={'L':0.,'R':0.}
    for i in range(N):
        sw='L' if i%2==0 else 'R'; ts=T0+i*Ts+dsr*Ts; te=T0+(i+1)*Ts
        idx=(t>=ts)&(t<=te); s=(t[idx]-ts)/(te-ts)
        x0,x1=pos[sw],landX[i]
        f[sw][0][idx]=x0+(x1-x0)*s**3*(10-15*s+6*s*s); f[sw][0][t>te]=x1; pos[sw]=x1
        f[sw][1][idx]=o['stepH']*0.5*(1-np.cos(2*np.pi*s))
    zp=o['zh']+ZHIP
    # CoM ofseti: yuruyus pozunda
    off=np.array([-0.013,0,0.03])
    qa={mj.name:m.jnt_qposadr[j] for j in range(m.njnt) for mj in [m.joint(j)]}
    for _ in range(2):
        qj=ik(np.array([[ -off[0],0,zp]]),np.zeros(1),np.zeros(1),'L'); qj.update(ik(np.array([[-off[0],0,zp]]),np.zeros(1),np.zeros(1),'R'))
        d.qpos[:]=0; d.qpos[:3]=[0,0,zp]; d.qpos[3]=1
        for kk,v in qj.items(): d.qpos[qa[kk]]=v[0]
        mujoco.mj_kinematics(m,d); mujoco.mj_comPos(m,d); off=d.subtree_com[0]-d.qpos[:3]
    zc=zp+off[2]
    com=np.stack([preview(zmp[:,0],zc,dt),preview(zmp[:,1],zc,dt)],1)
    def joints(com):
        pel=np.stack([com[:,0]-off[0],com[:,1]-off[1],np.full(n,zp)],1)
        q=ik(pel,f['L'][0],f['L'][1],'L'); q.update(ik(pel,f['R'][0],f['R'][1],'R'))
        if o['arm']:       # karsi kol salinimi
            for sd in 'LR':
                other='R' if sd=='L' else 'L'
                q[sd+'_shoulder_pitch']=-o['arm']*(f[other][0]-com[:,0])
        return pel,q
    adr={k:a for k,a in qa.items()}
    for it in range(o['iters']+1):
        pel,q=joints(com)
        Q=np.zeros((n,m.nq)); Q[:,:3]=pel; Q[:,3]=1
        for kk,v in q.items(): Q[:,adr[kk]]=v
        V=np.zeros((n,m.nv)); jn=[(m.jnt_qposadr[j],m.jnt_dofadr[j]) for j in range(1,m.njnt)]
        pa=[a for a,_ in jn]; va=[b for _,b in jn]
        V[:,:3]=np.gradient(pel,dt,axis=0); V[:,va]=np.gradient(Q[:,pa],dt,axis=0); A=np.gradient(V,dt,axis=0)
        if it==o['iters']: break
        zf=np.zeros((n,2)); need=np.zeros(m.nv); idxs=np.arange(0,n,2)
        for k in idxs:
            d.qpos[:]=Q[k]; d.qvel[:]=V[k]; d.qacc[:]=A[k]
            mujoco.mj_kinematics(m,d); mujoco.mj_comPos(m,d); mujoco.mj_comVel(m,d); mujoco.mj_rne(m,d,1,need)
            F=need[0:3]; Mw=need[3:6]+np.cross(Q[k,:3],F); fzv=max(F[2],0.3*m.body_mass.sum()*G)
            zf[k]=[-Mw[1]/fzv,Mw[0]/fzv]
        for c in range(2): zf[:,c]=np.interp(t,t[idxs],zf[idxs,c])
        e=np.clip(zmp-zf,-0.15,0.15)
        com=com+np.stack([preview(e[:,0],zc,dt),preview(e[:,1],zc,dt)],1)
    R=dict(t=t,px=pel[:,0],py=pel[:,1],pz=pel[:,2],zmpx=zmp[:,0],zmpy=zmp[:,1],fLz=f['L'][1],fRz=f['R'][1],fLx=f['L'][0],fRx=f['R'][0])
    R.update(q)
    return R
