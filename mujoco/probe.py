import mujoco, os
os.chdir('/home/yagiz/Documents/H175_V2')
m=mujoco.MjModel.from_xml_path('H175_V2.urdf'); d=mujoco.MjData(m); mujoco.mj_forward(m,d)
for n in ['pelvis','L_hip_pitch','L_knee','L_ankle_pitch','L_ankle_roll','L_toe','L_foot','R_foot','L_hand']:
    try: print(n, d.body(n).xpos.round(3), m.body(n).mass)
    except Exception as e: print(n,'yok')
import numpy as np
print([m.body(i).name for i in range(m.nbody) if 'L_' in m.body(i).name and 'finger' not in m.body(i).name and 'thumb' not in m.body(i).name][:30])
z=[ (m.geom(i).name, d.geom_xpos[i][2]) for i in range(m.ngeom)]
print('min geom z', min(z,key=lambda a:a[1]))
