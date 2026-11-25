"""wbc.py dogrulama: ayak gercekten kalkiyor mu, hizlar, tork/hiz kullanimi."""
import sys, numpy as np, mujoco
from wbc import Walker
def swing_check(w):
    w.reset(); m,d=w.m,w.d; sole=[]; air=[0,0]; tot=[0,0]; h=[0,0]; W=[]; T=[]
    while d.time<w.t[-1] and d.qpos[2]>0.7:
        tk=d.time; w.step(); sup=w.support(tk)
        T.append(np.abs(d.actuator_force)/m.actuator_forcerange[:,1]); W.append(np.abs(d.qvel[w.va]))
        for i in range(2):
            if not sup[i]:
                bid=w.feet[i][0]; zc=(d.xpos[bid]+d.xmat[bid].reshape(3,3)@np.array([0.02,0,-0.08]))[2]
                h[i]=max(h[i],zc); tot[i]+=1
                touching=any({m.geom_bodyid[c.geom1],m.geom_bodyid[c.geom2]}&set(w.feet[i]) and c.dist<0 for c in d.contact[:d.ncon])
                air[i]+=not touching
    T=np.array(T); W=np.array(W)
    wnl=__import__('json').load(open('/home/yagiz/Documents/H175_V2/actuators.json'))['joints']
    tp=T.max(0); j=np.argsort(-tp)[:3]; r=W.max(0)/np.array([wnl[n]['w_noload'] for n in w.names]); k=int(np.argmax(r))
    return ('havada %%%.0f/%.0f, tepe yukseklik %.1f/%.1f cm'%(100*air[0]/tot[0],100*air[1]/tot[1],100*h[0],100*h[1]),
            'tork/Tpk '+', '.join('%s %.2f'%(w.names[i],tp[i]) for i in j), 'hiz/wnl %s %.2f'%(w.names[k],r[k]))
for nm in sys.argv[1:]:
    w=Walker(nm); print(nm,w.run()); print('    ',*swing_check(w),sep='\n     ')
