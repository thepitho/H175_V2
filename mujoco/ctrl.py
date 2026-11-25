import mujoco, numpy as np
class StaticFF:
    """Statik ileri besleme: govde 6 DOF'u ayak kosesi kuvvetleriyle dengele, eklem torkunu cikar."""
    def __init__(s,m,va):
        s.m=m; s.va=np.array(va); s.cor=np.array([[x,y,-0.08] for x in (-0.07,0.11) for y in (-0.05,0.05)])
        s.bodies=[m.body('L_ankle_roll').id,m.body('R_ankle_roll').id]
        s.jp=np.zeros((3,m.nv)); s.jr=np.zeros((3,m.nv))
    def __call__(s,d,feet=(True,True)):
        m=s.m; qa=d.qacc.copy(); bias=d.qfrc_bias.copy()
        cols=[]
        for b,on in zip(s.bodies,feet):
            if not on: continue
            R=d.xmat[b].reshape(3,3)
            for c in s.cor:
                p=d.xpos[b]+R@c; mujoco.mj_jac(m,d,s.jp,None,p,b); cols.append(s.jp.copy())
        J=np.vstack(cols)                     # (3*8, nv)
        A=J[:,:6].T                           # taban satirlari (6 x 24)
        f=np.linalg.lstsq(A,bias[:6],rcond=None)[0]
        return bias[s.va]-(J.T@f)[s.va]
