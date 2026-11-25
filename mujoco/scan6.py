import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import walk_test as w
D='/home/yagiz/Documents/H175_V2/mujoco/'
for f,desc in (('gA','vx0.25 Ts1.0 (adim 25 cm, sure ayni)'),('gB','vx0.12 Ts0.8 (adim 9.6 cm, sure kisa)'),('gC','vx0.12 Ts0.6 (adim 7 cm)')):
    for gy in (-3,-2):
        print(f,desc,'gy',gy,end=' '); w.run(csv=D+f+'.csv',verbose=False,gy=gy,gyv=-1)
