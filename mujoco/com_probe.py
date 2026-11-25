import mujoco, numpy as np, sys
sys.path.insert(0,'/home/yagiz/Documents/H175_V2/mujoco')
from make_model import build
m=build().compile(); d=mujoco.MjData(m); d.qpos[2]=0.925; d.qpos[3]=1; mujoco.mj_forward(m,d)
print('com',d.subtree_com[1].round(3)); print('ankle x',d.body('L_ankle_roll').xpos.round(3))
