import mujoco, numpy as np, os
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)),'..'))
m = mujoco.MjModel.from_xml_path('H175_V2.urdf')
d = mujoco.MjData(m)
print('nbody',m.nbody,'njnt',m.njnt,'nu',m.nu,'mass',m.body_mass.sum())
mujoco.mj_forward(m,d); print('ok')
