import json, numpy as np, mujoco, gait
from wbc import Walker
act=json.load(open('/home/yagiz/Documents/H175_V2/actuators.json'))['joints']
for v in (1.2,1.4,1.6):
    R=gait.make(vx=v,Ts=0.4,dsr=0.1,zh=0.78,stepH=0.06,heel=0.1,widthz=0.8,nSteps=14)
    w=Walker(R,tn=False,kscale=2.0,kdcm=1.0)
    i0,i1=int(2.3/0.005),int(6.0/0.005)
    # referans hiz ve ters dinamik tork ihtiyaci
    print('v=%.1f referans eklem hizi / wnl:'%v, {k:round(float(np.abs(w.V[i0:i1,w.va[w.idx['L_'+k]]]).max()/act['L_'+k]['w_noload']),2) for k in ('hip_pitch','hip_roll','knee','ankle_pitch','ankle_roll')})
    m,d=w.m,w.d; T=[];W=[]
    while d.time<w.t[-1] and d.qpos[2]>0.7:
        w.step()
        if 1.5<d.time<4.0: T.append(np.abs(d.actuator_force)/w.Tpk); W.append(np.abs(d.qvel[w.va])/w.Wnl)
    T=np.array(T);W=np.array(W)
    print('   tn egrisi KAPALI:', 'yurudu' if d.qpos[2]>0.7 else 'dustu %.1f'%d.time,' tork:',{k:round(float(max(T[:,w.idx['L_'+k]].max(),T[:,w.idx['R_'+k]].max())),2) for k in ('hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll')},
          '\n      hiz/wnl:',{k:round(float(max(W[:,w.idx['L_'+k]].max(),W[:,w.idx['R_'+k]].max())),2) for k in ('hip_pitch','hip_roll','knee','ankle_pitch')})
