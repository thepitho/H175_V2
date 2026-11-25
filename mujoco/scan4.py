import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import walk_test as w
for f in ('g25','g40','g60'):
    for gx in (0.0,1.0):
        print(f,'gx',gx,end=' '); w.run(csv=f'/home/yagiz/Documents/H175_V2/mujoco/{f}.csv',verbose=False,gy=-2.5,gyv=-1,gx=gx,gv=0.3*gx)
