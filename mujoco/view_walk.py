"""Yuruyusu canli pencerede izle (tek is parcacikli GLFW; mujoco.viewer bu makinede segfault veriyordu).
Kullanim: venv/bin/python view_walk.py [referans_adi]     (g25 varsayilan; gait_ref, g25_30, v040, v060)
Fare: sol surukle=dondur, sag surukle=kaydir, tekerlek=yakinlas.  R=bastan, BOSLUK=duraklat, ESC=cik."""
import os, sys, time
import numpy as np, mujoco, glfw
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
from wbc import Walker
name=sys.argv[1] if len(sys.argv)>1 else 'g25'
name=os.path.splitext(os.path.basename(name))[0]
walker=Walker(name); m,d,R,t=walker.m,walker.d,walker.R,walker.t
def reset(): walker.reset()
def step(): walker.step()

if not glfw.init(): sys.exit('GLFW acilamadi (ekran yok mu?)')
win=glfw.create_window(1100,750,'H175 yuruyus',None,None)
if not win: glfw.terminate(); sys.exit('pencere acilamadi')
glfw.make_context_current(win); glfw.swap_interval(1)
cam=mujoco.MjvCamera(); cam.distance=3.0; cam.azimuth=140; cam.elevation=-12
opt=mujoco.MjvOption(); scn=mujoco.MjvScene(m,maxgeom=2000); con=mujoco.MjrContext(m,mujoco.mjtFontScale.mjFONTSCALE_150)
st={'pause':False,'last':(0,0),'t0':time.time()}
def restart(): reset(); st['t0']=time.time()
def on_key(w,key,sc,action,mods):
    if action!=glfw.PRESS: return
    if key==glfw.KEY_ESCAPE: glfw.set_window_should_close(w,True)
    elif key==glfw.KEY_R: restart()
    elif key==glfw.KEY_SPACE: st['pause']=not st['pause']
def on_move(w,x,y):
    dx,dy=x-st['last'][0],y-st['last'][1]; st['last']=(x,y)
    if glfw.get_mouse_button(w,glfw.MOUSE_BUTTON_LEFT)==glfw.PRESS: cam.azimuth-=0.3*dx; cam.elevation=np.clip(cam.elevation-0.3*dy,-89,89)
    elif glfw.get_mouse_button(w,glfw.MOUSE_BUTTON_RIGHT)==glfw.PRESS:
        a=np.radians(cam.azimuth); s=0.002*cam.distance
        cam.lookat[0]+=s*(-dx*np.sin(a)); cam.lookat[1]+=s*(dx*np.cos(a)); cam.lookat[2]+=s*dy
def on_scroll(w,x,y): cam.distance=max(0.5,cam.distance*(1-0.1*y))
glfw.set_key_callback(win,on_key); glfw.set_cursor_pos_callback(win,on_move); glfw.set_scroll_callback(win,on_scroll)
restart(); cam.lookat[:]=[d.qpos[0],0,0.7]; px=d.qpos[0]
auto=float(os.environ.get('H175_AUTOCLOSE','0')); start=time.time()
while not glfw.window_should_close(win):
    if st['pause']: st['t0']=time.time()-d.time
    else:
        hedef=time.time()-st['t0']; n=0
        while d.time<hedef and d.time<t[-1] and d.qpos[2]>0.4 and n<100: step(); n+=1
    cam.lookat[0]+=d.qpos[0]-px; px=d.qpos[0]     # robotu takip et
    w,h=glfw.get_framebuffer_size(win); vp=mujoco.MjrRect(0,0,w,h)
    mujoco.mjv_updateScene(m,d,opt,None,cam,mujoco.mjtCatBit.mjCAT_ALL,scn); mujoco.mjr_render(vp,scn,con)
    durum='DUSTU' if d.qpos[2]<0.7 else ('bitti' if d.time>=t[-1] else '')
    mujoco.mjr_overlay(mujoco.mjtFont.mjFONT_NORMAL,mujoco.mjtGridPos.mjGRID_TOPLEFT,vp,
                       f'{name}  t={d.time:.1f} s  x={d.qpos[0]:.2f} m  {durum}','R: bastan  BOSLUK: duraklat  ESC: cik',con)
    glfw.swap_buffers(win); glfw.poll_events()
    if auto and time.time()-start>auto: break
print(f't={d.time:.1f} x={d.qpos[0]:.3f} z={d.qpos[2]:.3f}')
con.free(); glfw.destroy_window(win); glfw.terminate()
