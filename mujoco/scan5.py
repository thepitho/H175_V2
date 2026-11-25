import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import walk_test as w
D='/home/yagiz/Documents/H175_V2/mujoco/'
for f in ('gait_ref','g25'):
  for mode in ('ref','contact'):
    for gy in (-3,-2):
        print(f,mode,gy,end=' '); w.run(csv=D+f+'.csv',verbose=False,gy=gy,gyv=-1,supmode=mode)
