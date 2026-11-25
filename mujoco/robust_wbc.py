"""Gurbuzluk: kazanc, surtunme, govde kutle/CoM hatasi (kontrolcu bilmiyor), itme."""
import numpy as np, mujoco
from wbc import Walker
def trial(name='g25', fric=None, mass=None, comx=0.0, push=None, **kw):
    w=Walker(name,**kw); m=w.m
    # modeli (gercek robot) boz; kontrolcu nominal kopyayi kullanir
    plant=Walker(name,**kw).m
    if fric is not None: plant.geom_friction[:,0]=fric
    if mass is not None or comx:
        b=plant.body('torso').id; plant.body_mass[b]+= mass or 0; plant.body_ipos[b,0]+=comx; mujoco.mj_setConst(plant,mujoco.MjData(plant))
    nom=m; d=w.d
    mujoco.mj_resetData(plant,d); d.qpos[:]=w.Q[0]; d.qpos[2]+=0.0005; mujoco.mj_forward(plant,d)
    orig=mujoco.mj_step
    pel=plant.body('pelvis').id
    while d.time<w.t[-1] and d.qpos[2]>0.7:
        if push:
            t0,F=push; d.xfrc_applied[pel,:3]=F if t0<=d.time<t0+0.2 else 0
        def st(mm,dd): orig(plant,dd)
        mujoco.mj_step=st
        try: w.step()
        finally: mujoco.mj_step=orig
    return ('OK ' if d.qpos[2]>0.7 else 'DUSTU t=%.1f '%d.time)+'x=%.2f/%.2f'%(d.qpos[0],w.R['px'][-1])
if __name__=='__main__':
    for lab,kw in [('nominal',{}),('kdcm 0.5',{'kdcm':0.5}),('kdcm 3',{'kdcm':3.0}),('kdcm 0',{'kdcm':0.0}),('kscale 1',{'kscale':1.0}),('kscale 3',{'kscale':3.0}),
        ('kroll/pitch 0',{'kroll':0,'droll':0,'kpitch':0,'dpitch':0}),('surtunme 0.6',{'fric':0.6}),('surtunme 0.4',{'fric':0.4}),
        ('govde +3 kg',{'mass':3.0}),('govde +7 kg',{'mass':7.0}),('govde CoM +2cm',{'comx':0.02}),('govde CoM -2cm',{'comx':-0.02}),
        ('itme ileri 100N',{'push':(4.0,[100,0,0])}),('itme geri 100N',{'push':(4.0,[-100,0,0])}),('itme yan 100N',{'push':(4.0,[0,100,0])}),('itme yan -100N',{'push':(4.0,[0,-100,0])})]:
        print('%-18s %s'%(lab,trial(**kw)),flush=True)
