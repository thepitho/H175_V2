import mujoco, numpy as np, sys
sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
from make_model import build
from ctrl import StaticFF
def run(T=10, push=None, verbose=True):
    """push=(t0, [Fx,Fy,Fz], sure) govdeye (pelvis) kuvvet uygular."""
    m=build().compile(); d=mujoco.MjData(m)
    Kp=-m.actuator_biasprm[:,1].copy()
    jid=[m.actuator_trnid[i,0] for i in range(m.nu)]
    va=np.array([m.jnt_dofadr[j] for j in jid]); sff=StaticFF(m,va)
    d.qpos[2]=0.9255; d.qpos[3]=1; mujoco.mj_forward(m,d); zmin=9
    for k in range(int(T/m.opt.timestep)):
        d.ctrl[:]=sff(d)/Kp
        if push and push[0]<=d.time<push[0]+push[2]: d.xfrc_applied[1,:3]=push[1]
        else: d.xfrc_applied[:]=0
        mujoco.mj_step(m,d); zmin=min(zmin,d.qpos[2])
        if verbose and k%int(1/m.opt.timestep)==0: print(f't={d.time:.0f} z={d.qpos[2]:.3f} x={d.qpos[0]:+.3f} y={d.qpos[1]:+.3f}')
        if d.qpos[2]<0.7: print('DEVRILDI t=%.2f'%d.time); return False
    print('AYAKTA z=%.3f x=%+.3f y=%+.3f'%(d.qpos[2],d.qpos[0],d.qpos[1])); return True
if __name__=='__main__':
    run(10)
    for F in (50,100,150,200):
        print('it x',F,end=' '); run(5,push=(1.0,[F,0,0],0.2),verbose=False)
    for F in (50,100,150):
        print('it y',F,end=' '); run(5,push=(1.0,[0,F,0],0.2),verbose=False)
