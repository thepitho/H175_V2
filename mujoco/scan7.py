import sys, numpy as np; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
from diag_walk import sim
for f in ('gait_ref','g25','gB','gA'):
    for cl in (None,0.045,0.03):
        L,fell,R=sim(f,clamp=cl)
        r=max(L['rollL'].max(),L['rollR'].max()); p=max(L['pitchL'].max(),L['pitchR'].max())
        print(f,'clamp',cl,('devrildi %.2f'%fell) if fell else 'dusmedi','max roll %.1f pitch %.1f deg'%(np.degrees(r),np.degrees(p)),'doyma %.3f'%L['sat'].mean())
