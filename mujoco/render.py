import sys, numpy as np, mujoco
sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
from make_model import build
m=build().compile(); d=mujoco.MjData(m); d.qpos[2]=0.925; d.qpos[3]=1; mujoco.mj_forward(m,d)
print('geom sayisi',m.ngeom,'mesh',m.nmesh)
r=mujoco.Renderer(m,480,360); cam=mujoco.MjvCamera(); cam.distance=3.2; cam.azimuth=130; cam.elevation=-10; cam.lookat[:]=[0,0,0.9]
r.update_scene(d,cam); import PIL.Image as I
I.fromarray(r.render()).save('/home/yagiz/Documents/H175_V2/mujoco/gorunum.png')
