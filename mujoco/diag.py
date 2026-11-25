import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import mujoco, numpy as np
from make_model import build
m=build().compile(); d=mujoco.MjData(m)
Tpk=m.actuator_ctrlrange[:,1].copy(); Kp=Tpk/0.15; Kd=0.1*Kp
jid=[m.actuator_trnid[i,0] for i in range(m.nu)]
qa=np.array([m.jnt_qposadr[j] for j in jid]); va=np.array([m.jnt_dofadr[j] for j in jid]); names=[m.joint(j).name for j in jid]
d.qpos[2]=0.9245; d.qpos[3]=1; mujoco.mj_forward(m,d)
for k in range(1500):
    q=d.qpos[qa]; w=d.qvel[va]; a=d.qacc.copy(); d.qacc[:]=0; mujoco.mj_inverse(m,d); ff=d.qfrc_inverse[va].copy(); d.qacc[:]=a
    d.ctrl[:]=np.clip(ff-Kp*q-Kd*w,-Tpk,Tpk); mujoco.mj_step(m,d)
    if k in (0,250,500,1000,1499):
        o=np.argsort(-abs(q))[:5]; print(f't={d.time:.2f} z={d.qpos[2]:.3f} x={d.qpos[0]:+.3f} ncon={d.ncon}', [(names[i],round(q[i],3)) for i in o], 'sat',int(np.sum(abs(d.ctrl)>=Tpk-1e-6)))
q=d.qpos[3:7]; print('quat',q.round(3))
