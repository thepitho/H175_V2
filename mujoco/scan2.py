import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import walk_test as w
for gy,gyv in [(1,0.3),(-1,-0.3),(3,1),(-3,-1)]:
    print(gy,gyv,end=' '); w.run(verbose=False,gy=gy,gyv=gyv)
