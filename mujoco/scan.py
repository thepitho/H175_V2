import sys; sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
import stand_test as s
for gx,gv in [(0.5,0.3),(1.0,0.5),(1.0,1.0),(0.5,1.0),(0.0,0.0)]:
    s.GX=gx; s.GV=gv; print(gx,gv,end=' '); s.run(T=6,verbose=False)
