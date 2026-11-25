import cadquery as cq, math, json, os, xml.etree.ElementTree as E
from pathlib import Path
P=Path(__file__).parent; os.chdir(P)
links={}; joints=[]
def link(n,p,prims,m): links[n]=(p,prims,m)
def box(x,y,z,a,b,c): return ['box',x,y,z,a,b,c]
def cyl(x,y,z,r,h,axis='y'): return ['cyl',x,y,z,r,h,axis]
def j(n,parent,child,axis,lo=-120,hi=120): joints.append((n,parent,child,axis,lo,hi))
# x forward, y left, z up; mm; all coordinates global in neutral pose
link('pelvis',(0,0,925),[box(0,0,925,180,270,100)],5)
link('waist_yaw',(0,0,990),[cyl(0,0,990,65,45,'z')],1); j('waist_yaw','pelvis','waist_yaw',(0,0,1),-60,60)
link('torso',(0,0,1030),[box(0,0,1190,190,350,300),cyl(0,0,1030,58,100)],10); j('waist_pitch','waist_yaw','torso',(0,1,0),-30,45)
link('neck_yaw',(0,0,1365),[cyl(0,0,1365,38,35,'z')],.4); j('neck_yaw','torso','neck_yaw',(0,0,1),-85,85)
# head top 1750, neck extended housing / head
link('head',(0,0,1410),[box(0,0,1580,175,170,340),cyl(0,0,1410,35,65)],3); j('neck_pitch','neck_yaw','head',(0,1,0),-35,35)
for s,k in [('L',1),('R',-1)]:
 def add(n,p,pr,m,parent,axis,lo=-120,hi=120):
  name=s+'_'+n; link(name,p,pr,m); j(name,parent if parent in ('pelvis','torso') else s+'_'+parent,name,axis,lo,hi)
 y=k*225
 add('shoulder_yaw',(0,k*196,1320),[cyl(0,k*196,1320,42,32,'z')],.8,'torso',(0,0,1))
 add('shoulder_roll',(0,y,1320),[cyl(0,y,1320,44,38,'x')],.8,'shoulder_yaw',(1,0,0))
 add('upper_arm',(0,y,1320),[cyl(0,y,1320,47,48),box(0,y,1180,72,70,235)],2.2,'shoulder_roll',(0,1,0))
 add('forearm',(0,y,1040),[cyl(0,y,1040,39,52),box(0,y,920,62,60,195)],1.5,'upper_arm',(0,1,0),-145,5)
 add('wrist_yaw',(0,y,790),[cyl(0,y,790,29,30,'z')],.3,'forearm',(0,0,1))
 add('wrist_roll',(0,y,765),[cyl(0,y,765,27,35,'x')],.3,'wrist_yaw',(1,0,0),-45,45)
 add('hand',(0,y,740),[cyl(0,y,740,25,38),box(0,y,670,48,83,120)],.6,'wrist_roll',(0,1,0),-60,60)
 y=k*105
 add('hip_yaw',(0,y,880),[cyl(0,y,880,58,45,'z')],1.2,'pelvis',(0,0,1),-45,45)
 add('hip_roll',(0,y,845),[cyl(0,y,845,60,60,'x')],1.2,'hip_yaw',(1,0,0),-40,40)
 add('thigh',(0,y,845),[cyl(0,y,845,62,70),box(0,y,665,105,90,300)],4,'hip_roll',(0,1,0),-100,45)
 add('shin',(0,y,465),[cyl(0,y,465,55,70),box(0,y,275,86,76,320)],3,'thigh',(0,1,0),0,145)
 add('ankle_pitch',(0,y,80),[cyl(0,y,80,42,65)],.8,'shin',(0,1,0),-45,45)
 add('foot',(0,y,55),[cyl(0,y,55,35,60,'x'),box(45,y,20,245,115,40)],1.2,'ankle_pitch',(1,0,0),-25,25)
# V2: additional articulated axes; all positions in neutral pose.
def insert(n,parent,child,pos,axis,prims,m,lo,hi):
    link(n,pos,prims,m)
    for i,(jn,pa,ch,ax,l,h) in enumerate(joints):
        if ch==child:
            assert pa==parent
            joints[i]=(jn,n,ch,ax,l,h);break
    j(n,parent,n,axis,lo,hi)
insert('waist_roll','waist_yaw','torso',(0,0,1010),(1,0,0),[cyl(0,0,1010,62,35,'x')],1,-20,20)
insert('neck_roll','neck_yaw','head',(0,0,1390),(1,0,0),[cyl(0,0,1390,30,24,'x')],.25,-20,20)
# reshape torso and head into a rounded industrial silhouette
links['head']=((0,0,1410),[box(0,0,1585,185,178,330),box(83,0,1600,35,155,95)],2.5)
links['torso']=((0,0,1030),[box(0,0,1190,220,350,300),box(-145,0,1185,95,260,235)],13)
for side,k in [('L',1),('R',-1)]:
 def sn(n):return side+'_'+n
 y=k*225; h=k*105
 insert(sn('scapula'),'torso',sn('shoulder_yaw'),(0,k*173,1310),(1,0,0),[cyl(0,k*173,1310,35,30,'x')],.5,-15,15)
 insert(sn('arm_twist'),sn('upper_arm'),sn('forearm'),(0,y,1078),(0,0,1),[cyl(0,y,1078,33,32,'z')],.4,-70,70)
 insert(sn('forearm_twist'),sn('forearm'),sn('wrist_yaw'),(0,y,825),(0,0,1),[cyl(0,y,825,27,28,'z')],.3,-80,80)
 insert(sn('hip_aux'),'pelvis',sn('hip_yaw'),(0,h,915),(0,1,0),[cyl(0,h,915,48,34,'y')],.8,-15,15)
 # separate toe articulation, foot shortened so toe geometry does not overlap sole
 links[sn('foot')]=((0,h,55),[cyl(0,h,55,35,60,'x'),box(20,h,20,195,115,40)],1.1)
 link(sn('toe'),(117.5,h,20),[box(145,h,20,55,110,40)],.3);j(sn('toe'),sn('foot'),sn('toe'),(0,1,0),-30,45)
 # palms: fingers point down in neutral CAD pose
 links[sn('hand')]=((0,y,740),[cyl(0,y,740,25,38),box(0,y,695,42,94,85)],.65)
 for fi,fn in enumerate(['index','middle','ring','little','thumb']):
  yy=y+k*([-32,-10,12,34,-52][fi]); xx=0 if fi<4 else 20; zz=652 if fi<4 else 710
  lengths=[40,27,23] if fi<4 else [31,25,21]; parent=sn('hand')
  for a,L in enumerate(lengths):
   name=sn(fn+str(a+1)); pos=(xx,yy,zz); axis=(0,0,1) if fi==4 and a==0 else (0,1,0)
   link(name,pos,[box(xx,yy,zz-L/2,17,17,L-1)],.045);j(name,parent,name,axis,-15,90);parent=name;zz-=L
# geometry separate solids, overlapping actuator envelopes deliberately conceptual
A=cq.Assembly(name='H175_V2')
root=E.Element('robot',name='H175_V2')
for n,(p,prims,m) in links.items():
 solids=[]
 for typ,x,y,z,*v in prims:
  if typ=='box': a,b,c=v; sh=cq.Workplane('XY').box(a,b,c).edges().fillet(min(a,b,c)*.18).val()
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
 weights=np.array(vols)/sum(vols)*m; com=np.average(centers,axis=0,weights=weights); I=np.zeros((3,3))
 for w,c,T in zip(weights,centers,tensors): d=c-com; I+=w*(T+np.dot(d,d)*np.eye(3)-np.outer(d,d))
 E.SubElement(inert,'origin',xyz=' '.join(map(str,com)),rpy='0 0 0'); E.SubElement(inert,'mass',value=str(m)); E.SubElement(inert,'inertia',**{key:str(I[a,b]) for key,a,b in [('ixx',0,0),('ixy',0,1),('ixz',0,2),('iyy',1,1),('iyz',1,2),('izz',2,2)]})
 for tag in ['visual','collision']:
  el=E.SubElement(l,tag); geo=E.SubElement(el,'geometry'); E.SubElement(geo,'mesh',filename=f'meshes/{n}.stl',scale='0.001 0.001 0.001')
for n,pa,ch,ax,lo,hi in joints:
 el=E.SubElement(root,'joint',name=n,type='revolute'); E.SubElement(el,'parent',link=pa);E.SubElement(el,'child',link=ch); E.SubElement(el,'origin',xyz=' '.join(str((a-b)/1000) for a,b in zip(links[ch][0],links[pa][0])),rpy='0 0 0');E.SubElement(el,'axis',xyz=' '.join(map(str,ax)));E.SubElement(el,'limit',lower=str(math.radians(lo)),upper=str(math.radians(hi)),effort='100',velocity='3')
E.indent(root); E.ElementTree(root).write(P/'H175_V2.urdf',encoding='utf-8',xml_declaration=True)
A.save(str(P/'H175_V2.step'))
(P/'geometry.json').write_text(json.dumps(links,indent=2));print(len(links),len(joints),sum(v[2] for v in links.values()))
