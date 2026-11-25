import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import walk_test as w, numpy as np
f,l=w.run(verbose=False,gy=-3,gyv=-1)
print('plan son x',np.genfromtxt(w.__file__.replace('walk_test.py','gait_ref.csv'),delimiter=',',names=True)['px'][-1])
print('max|y|',abs(l[:,2]).max(),'z min/max',l[:,3].min(),l[:,3].max(), 'zaman',l[-1,0])
for gy,gyv in [(-5,-1.5),(-2,-0.7),(-3,-2)]:
    print(gy,gyv,end=' '); w.run(verbose=False,gy=gy,gyv=gyv)
