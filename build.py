import cadquery as cq, math, json, os, xml.etree.ElementTree as E
from pathlib import Path
P=Path(__file__).parent; os.chdir(P)
# H175 V3 - Asimov-1 (menloresearch) topolojisi referans alinarak: kalca pitch->roll->yaw, diz, ayak bilegi pitch->roll,
# omuz pitch->roll->yaw. Eksenler kalca/omuzda tek noktada kesisir (kuresel eklem); motor govdeleri eksen disinda ofsetlidir.
# Sadece topoloji/oran referans alinmistir; Asimov CAD/STEP dosyalari kopyalanmamistir (donanim lisansi CERN-OHL-S-2.0).
# x ileri, y sol, z yukari; mm; tum koordinatlar notr pozda global.
links={}; joints=[]
def link(n,p,prims,m): links[n]=(p,prims,m)
def box(x,y,z,a,b,c): return ['box',x,y,z,a,b,c]
def cyl(x,y,z,r,h,axis='y'): return ['cyl',x,y,z,r,h,axis]
def j(n,parent,child,axis,lo=-120,hi=120): joints.append((n,parent,child,axis,lo,hi))
# ---- govde ----
link('pelvis',(0,0,925),[box(0,0,925,180,270,100)],5)
link('waist_yaw',(0,0,990),[cyl(0,0,990,65,45,'z')],1); j('waist_yaw','pelvis','waist_yaw',(0,0,1),-60,60)
link('waist_roll',(0,0,1010),[cyl(0,0,1010,62,35,'x')],1); j('waist_roll','waist_yaw','waist_roll',(1,0,0),-20,20)
link('torso',(0,0,1030),[box(0,0,1190,220,350,300),box(-145,0,1185,95,260,235),cyl(0,0,1030,58,100,'y')],13); j('waist_pitch','waist_roll','torso',(0,1,0),-30,45)
link('neck_yaw',(0,0,1365),[cyl(0,0,1365,38,35,'z')],.4); j('neck_yaw','torso','neck_yaw',(0,0,1),-85,85)
link('neck_roll',(0,0,1390),[cyl(0,0,1390,30,24,'x')],.25); j('neck_roll','neck_yaw','neck_roll',(1,0,0),-20,20)
link('head',(0,0,1410),[box(0,0,1585,185,178,330),box(83,0,1600,35,155,95),cyl(0,0,1410,35,65,'y')],2.5); j('neck_pitch','neck_roll','head',(0,1,0),-35,35)
# ---- bacaklar: 7 eksen/bacak ----
for s,k in [('L',1),('R',-1)]:
 sn=lambda n,s=s: s+'_'+n
 hy=k*105
 link(sn('hip_pitch'),(0,hy,880),[cyl(0,hy+k*48,880,45,40,'y'),box(0,hy,880,60,60,70)],2); j(sn('hip_pitch'),'pelvis',sn('hip_pitch'),(0,1,0),-110,45)
 link(sn('hip_roll'),(0,hy,880),[cyl(70,hy,880,45,40,'x'),box(35,hy,880,70,50,50)],2); j(sn('hip_roll'),sn('hip_pitch'),sn('hip_roll'),(1,0,0),-40,40)
 link(sn('hip_yaw'),(0,hy,880),[cyl(0,hy,835,50,50,'z'),box(0,hy,650,105,90,360)],3); j(sn('hip_yaw'),sn('hip_roll'),sn('hip_yaw'),(0,0,1),-45,45)
 link(sn('knee'),(0,hy,465),[cyl(0,hy+k*52,465,50,50,'y'),box(0,hy,275,86,76,320)],3); j(sn('knee'),sn('hip_yaw'),sn('knee'),(0,1,0),0,145)
 link(sn('ankle_pitch'),(0,hy,80),[cyl(0,hy,80,42,65,'y')],1); j(sn('ankle_pitch'),sn('knee'),sn('ankle_pitch'),(0,1,0),-45,45)
 link(sn('ankle_roll'),(0,hy,80),[cyl(-55,hy,100,40,40,'x'),box(20,hy,20,195,115,40)],1.5); j(sn('ankle_roll'),sn('ankle_pitch'),sn('ankle_roll'),(1,0,0),-25,25)
 link(sn('toe'),(117.5,hy,20),[box(145,hy,20,55,110,40)],.3); j(sn('toe'),sn('ankle_roll'),sn('toe'),(0,1,0),-30,45)
# ---- kollar: 10 eksen/kol ----
 y=k*225
 link(sn('scapula'),(0,k*150,1310),[cyl(0,k*150,1310,38,40,'x')],.6); j(sn('scapula'),'torso',sn('scapula'),(1,0,0),-15,15)
 link(sn('clavicle'),(0,k*180,1315),[cyl(0,k*180,1315,35,35,'z')],.5); j(sn('clavicle'),sn('scapula'),sn('clavicle'),(0,0,1),-20,20)
 link(sn('shoulder_pitch'),(0,y,1320),[cyl(0,y+k*50,1320,45,40,'y')],1); j(sn('shoulder_pitch'),sn('clavicle'),sn('shoulder_pitch'),(0,1,0),-170,60)
 link(sn('shoulder_roll'),(0,y,1320),[cyl(55,y,1320,45,40,'x')],1); j(sn('shoulder_roll'),sn('shoulder_pitch'),sn('shoulder_roll'),(1,0,0),*( (-20,160) if k==1 else (-160,20) ))
 link(sn('shoulder_yaw'),(0,y,1320),[cyl(0,y,1270,48,45,'z'),box(0,y,1180,72,70,235)],2.2); j(sn('shoulder_yaw'),sn('shoulder_roll'),sn('shoulder_yaw'),(0,0,1),-90,90)
 link(sn('elbow'),(0,y,1040),[cyl(0,y+k*45,1040,40,45,'y'),box(0,y,1015,60,60,60)],1.5); j(sn('elbow'),sn('shoulder_yaw'),sn('elbow'),(0,1,0),-145,5)
 link(sn('forearm_twist'),(0,y,1000),[cyl(0,y,985,32,35,'z'),box(0,y,890,62,60,200)],.8); j(sn('forearm_twist'),sn('elbow'),sn('forearm_twist'),(0,0,1),-80,80)
 link(sn('wrist_pitch'),(0,y,790),[cyl(0,y,790,30,35,'y')],.3); j(sn('wrist_pitch'),sn('forearm_twist'),sn('wrist_pitch'),(0,1,0),-60,60)
 link(sn('wrist_roll'),(0,y,790),[cyl(0,y,790,28,35,'x')],.3); j(sn('wrist_roll'),sn('wrist_pitch'),sn('wrist_roll'),(1,0,0),-45,45)
 link(sn('wrist_yaw'),(0,y,765),[cyl(0,y,758,26,28,'z'),box(0,y,695,42,94,85)],.65); j(sn('wrist_yaw'),sn('wrist_roll'),sn('wrist_yaw'),(0,0,1),-60,60)
 # parmaklar: avuc icine bakan 15 eksen/el
 for fi,fn in enumerate(['index','middle','ring','little','thumb']):
  yy=y+k*([-32,-10,12,34,-52][fi]); xx=0 if fi<4 else 20; zz=652 if fi<4 else 710
  lengths=[40,27,23] if fi<4 else [31,25,21]; parent=sn('wrist_yaw')
  for a,L in enumerate(lengths):
   name=sn(fn+str(a+1)); pos=(xx,yy,zz); axis=(0,0,1) if fi==4 and a==0 else (0,1,0)
   link(name,pos,[box(xx,yy,zz-L/2,17,17,L-1)],.045);j(name,parent,name,axis,-15,90);parent=name;zz-=L
# V3: motor secimi + elektronik yerlesim + kutle butcesi
exec(open(P/'v3_patch.py',encoding='utf-8').read())
WALLT=WALL
# geometry separate solids, overlapping actuator envelopes deliberately conceptual
A=cq.Assembly(name='H175_V2')
root=E.Element('robot',name='H175_V2')
for n,(p,prims,m) in links.items():
 solids=[]
 for pi_,(typ,x,y,z,*v) in enumerate(prims):
  if typ=='box':
   a,b,c=v; outer=cq.Workplane('XY').box(a,b,c).edges().fillet(min(a,b,c)*.18)
   if n in HOLLOW and pi_ in HOLLOW[n]: outer=outer.cut(cq.Workplane('XY').box(a-2*WALLT,b-2*WALLT,c-2*WALLT))
   sh=outer.val()
  else:
   r,h,axis=v; sh=cq.Solid.makeCylinder(r,h,cq.Vector(0,0,-h/2));
   if axis=='y': sh=sh.rotate((0,0,0),(1,0,0),-90)
   if axis=='x': sh=sh.rotate((0,0,0),(0,1,0),90)
  solids.append(sh.translate(cq.Vector(x-p[0],y-p[1],z-p[2])))
 comp=cq.Compound.makeCompound(solids); A.add(comp,name=n,loc=cq.Location(cq.Vector(*p)),color=cq.Color(.88,.89,.88) if any(q in n for q in ('torso','pelvis','head','upper_arm','forearm','thigh','shin','hand','foot','toe')) else cq.Color(.15,.16,.18))
 # STL in metres for URDF (mesh generated in mm, scale specified in URDF)
 cq.exporters.export(comp,str(P/'meshes'/f'{n}.stl'),tolerance=.5,angularTolerance=.2)
 l=E.SubElement(root,'link',name=n); inert=E.SubElement(l,'inertial')
 # approximate composite box/cylinder inertia, mass distributed by envelope volume, parallel axis around estimated COM
 import numpy as np
 vols=[]; centers=[]; tensors=[]
 for typ,x,y,z,*v in prims:
  centers.append((np.array([x,y,z])-p)/1000)
  if typ=='box':
   a,b,c=np.array(v)/1000; vols.append(a*b*c); tensors.append(np.diag([b*b+c*c,a*a+c*c,a*a+b*b])/12)
  else:
   r,h,axis=v; r/=1000;h/=1000; vols.append(math.pi*r*r*h); t=np.full(3,(3*r*r+h*h)/12);t['xyz'.index(axis)]=r*r/2;tensors.append(np.diag(t))
 elp=[(c_,s_,m_) for (par_,c_,s_,m_,d_) in ELEC.values() if par_==n]; m_el=sum(e[2] for e in elp)
 weights=np.array(vols)/sum(vols)*(m-m_el)
 for c_,s_,m_ in elp:
  a_,b_,c2_=np.array(s_)/1000; centers.append((np.array(c_)-p)/1000); tensors.append(np.diag([b_*b_+c2_*c2_,a_*a_+c2_*c2_,a_*a_+b_*b_])/12); weights=np.append(weights,m_)
 com=np.average(centers,axis=0,weights=weights); I=np.zeros((3,3))
 for w,c,T in zip(weights,centers,tensors): d=c-com; I+=w*(T+np.dot(d,d)*np.eye(3)-np.outer(d,d))
 if n in ARM_BY_LINK:
  k_=int(np.argmax(np.abs(ARM_AXIS[n])))
  for q_ in range(3): I[q_,q_]+= ARM_BY_LINK[n] if q_==k_ else ARM_BY_LINK[n]/2
 E.SubElement(inert,'origin',xyz=' '.join(map(str,com)),rpy='0 0 0'); E.SubElement(inert,'mass',value=str(m)); E.SubElement(inert,'inertia',**{key:str(I[a,b]) for key,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
 for tag in ['visual','collision']:
  el=E.SubElement(l,tag); geo=E.SubElement(el,'geometry'); E.SubElement(geo,'mesh',filename=f'meshes/{n}.stl',scale='0.001 0.001 0.001')
for n,pa,ch,ax,lo,hi in joints:
 el=E.SubElement(root,'joint',name=n,type='revolute'); E.SubElement(el,'parent',link=pa);E.SubElement(el,'child',link=ch); E.SubElement(el,'origin',xyz=' '.join(str((a-b)/1000) for a,b in zip(links[ch][0],links[pa][0])),rpy='0 0 0');E.SubElement(el,'axis',xyz=' '.join(map(str,ax)));E.SubElement(el,'limit',lower=str(math.radians(lo)),upper=str(math.radians(hi)),effort=str(JLIM[n][0]),velocity=str(JLIM[n][1]))
E.indent(root); E.ElementTree(root).write(P/'H175_V2.urdf',encoding='utf-8',xml_declaration=True)
for en,(par_,c_,s_,m_,d_) in ELEC.items(): A.add(cq.Workplane('XY').box(*s_).val(),name='ELEC_'+en,loc=cq.Location(cq.Vector(*c_)),color=cq.Color(.1,.45,.2))
fit=fit_report(); print([r[0] for r in fit if r[9]=='SIGMAZ'] or 'tum elektronik sigiyor'); write_reports()
A.save(str(P/'H175_V2.step'))
(P/'geometry.json').write_text(json.dumps(links,indent=2));print(len(links),len(joints),sum(v[2] for v in links.values()))
