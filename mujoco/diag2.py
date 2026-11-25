import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import mujoco, numpy as np
from make_model import build
from ctrl import StaticFF
m=build().compile(); d=mujoco.MjData(m)
Tpk=m.actuator_forcerange[:,1].copy(); Kp=-m.actuator_biasprm[:,1].copy()
jid=[m.actuator_trnid[i,0] for i in range(m.nu)]
qa=np.array([m.jnt_qposadr[j] for j in jid]); va=np.array([m.jnt_dofadr[j] for j in jid]); names=[m.joint(j).name for j in jid]
sff=StaticFF(m,va); d.qpos[2]=0.9245; d.qpos[3]=1; mujoco.mj_forward(m,d)
for k in range(1000):
    q=d.qpos[qa]; w=d.qvel[va]; ff=sff(d)
    if k==0:
        print('ff top',[(names[i],round(ff[i],1)) for i in np.argsort(-abs(ff))[:6]], 'Tpk', Tpk[np.argsort(-abs(ff))[:6]])
    d.ctrl[:]=ff/Kp; mujoco.mj_step(m,d)
    if k%100==0:
        o=np.argsort(-abs(q))[:3]; print(f't={d.time:.2f} z={d.qpos[2]:.3f} x={d.qpos[0]:+.3f} ncon={d.ncon}',[(names[i],round(q[i],3)) for i in o])
