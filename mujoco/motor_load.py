"""Yurume (wbc.py) sirasinda motor yuku. Kullanim: motor_load.py [referanslar...] -> motor_yuk_mujoco.csv"""
import sys, json, csv, numpy as np
from wbc import Walker
act=json.load(open('/home/yagiz/Documents/H175_V2/actuators.json'))['joints']
rows=[]
for nm in sys.argv[1:] or ['g25','v040']:
    w=Walker(nm); m,d=w.m,w.d; T=[]; V=[]
    while d.time<w.t[-1] and d.qpos[2]>0.7:
        w.step(); T.append(d.actuator_force.copy()); V.append(d.qvel[w.va].copy())
    T=np.array(T); V=np.array(V)
    print(nm,'dusmedi' if d.qpos[2]>0.7 else 'DUSTU','toplam mekanik guc ort %.0f W tepe %.0f W'%(np.abs(T*V).sum(1).mean(),np.abs(T*V).sum(1).max()))
    for i,n in enumerate(w.names):
        a=act[n]; rows.append([nm,n,a['motor'],a['tau_peak'],a['tau_rated'],np.abs(T[:,i]).max(),np.sqrt((T[:,i]**2).mean()),np.abs(V[:,i]).max(),a['w_noload']])
    body=[r for r in rows if r[0]==nm and not any(k in r[1] for k in('index','middle','ring','little','thumb'))]
    for r in sorted(body,key=lambda r:-r[5]/r[3])[:10]:
        print('  %-14s %-8s tepe %6.1f/%5.0f = %.2f   rms/anma %.2f'%(r[1],r[2],r[5],r[3],r[5]/r[3],r[6]/r[4]))
    print('  ankle_roll tepe/Tpk',[round(float(r[5]/r[3]),2) for r in body if 'ankle_roll' in r[1]],' rms>anma:',[r[1] for r in body if r[6]>r[4]])
with open('/home/yagiz/Documents/H175_V2/mujoco/motor_yuk_mujoco.csv','w',newline='') as f:
    c=csv.writer(f,delimiter=';'); c.writerow(['referans','eklem','motor','Tpk','Trated','tepe_tork','rms_tork','tepe_hiz','w_noload'])
    for r in rows: c.writerow(r[:3]+[round(float(x),3) for x in r[3:]])
