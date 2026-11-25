"""Pencere acmadan yuruyusu MP4'e kaydet:  venv/bin/python video.py [referans_adi]"""
import os, sys; os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np, mujoco, imageio.v2 as imageio
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from wbc import Walker
name=sys.argv[1] if len(sys.argv)>1 else 'g25'
w=Walker(name); m,d,t=w.m,w.d,w.t
ren=mujoco.Renderer(m,480,640); cam=mujoco.MjvCamera(); cam.distance=3.0; cam.azimuth=140; cam.elevation=-12
out=f'{HERE}/{name}.mp4'; wr=imageio.get_writer(out,fps=30); nxt=0.0
while d.time<t[-1] and d.qpos[2]>0.5:
    w.step()
    if d.time>=nxt:
        cam.lookat[:]=[d.qpos[0],0,0.7]; ren.update_scene(d,cam); wr.append_data(ren.render()); nxt+=1/30
wr.close(); print('yazildi',out,'sure %.1f s'%d.time)
