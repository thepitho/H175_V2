"""URDF -> MuJoCo: serbest govde, zemin, ayak temasi, motor tork sinirlari (actuators.json)."""
import mujoco, json, os, numpy as np
ROOT='/home/yagiz/Documents/H175_V2'
def build():
    os.chdir(ROOT)
    spec = mujoco.MjSpec.from_file('H175_V2.urdf')
    spec.compiler.fusestatic = 0; spec.compiler.discardvisual = 0
    spec.option.timestep = 0.002; spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    for g in spec.geoms: g.contype=0; g.conaffinity=0
    pel = spec.body('pelvis') if any(b.name=='pelvis' for b in spec.bodies) else spec.worldbody.first_body()
    pel.add_freejoint()
    spec.add_texture(name='dama',type=mujoco.mjtTexture.mjTEXTURE_2D,builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                     rgb1=[.55,.6,.65],rgb2=[.35,.4,.45],width=256,height=256)
    mat=spec.add_material(name='zemin',texrepeat=[2,2],texuniform=True)
    tx=['']*10; tx[mujoco.mjtTextureRole.mjTEXROLE_RGB]='dama'; mat.textures=tx
    spec.worldbody.add_light(pos=[0,-2,3],dir=[0,.5,-1],diffuse=[.8,.8,.8])
    spec.worldbody.add_geom(material='zemin',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[20,20,0.1],contype=1,conaffinity=1,friction=[1.2,0.02,0.002],rgba=[1,1,1,1])
    # ayak: ankle_roll govdesine kutu (taban z=-0.080 ayak bileginin altinda), toe govdesine on kutu
    for s in 'LR':
        spec.body(f'{s}_ankle_roll').add_geom(type=mujoco.mjtGeom.mjGEOM_BOX,pos=[0.02,0,-0.075],size=[0.09,0.05,0.005],contype=1,conaffinity=1,friction=[1.2,0.02,0.002],rgba=[.2,.2,.2,1],margin=0.004,gap=0.004)
        spec.body(f'{s}_toe').add_geom(type=mujoco.mjtGeom.mjGEOM_BOX,pos=[0.045,0,-0.015],size=[0.05,0.05,0.005],contype=1,conaffinity=1,friction=[1.2,0.02,0.002],rgba=[.2,.2,.2,1],margin=0.004,gap=0.004)
    act = json.load(open('actuators.json'))['joints']
    for jn,a in act.items():
        kp=a['tau_peak']/0.15; kd=0.1*kp
        spec.add_actuator(name=jn,target=jn,trntype=mujoco.mjtTrn.mjTRN_JOINT,gear=[1,0,0,0,0,0],
                          gaintype=mujoco.mjtGain.mjGAIN_FIXED,biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                          gainprm=[kp]+[0]*9,biasprm=[0,-kp,-kd]+[0]*7,
                          forcelimited=1,forcerange=[-a['tau_peak'],a['tau_peak']],ctrllimited=0)
    # parmaklar ve motorsuz eklemler: yuksek sonumleme ile sabit tutulur
    for j in spec.joints:
        if j.type==mujoco.mjtJoint.mjJNT_HINGE and j.name not in act:
            j.damping=[0.5,0,0]; j.stiffness=[2.0,0,0]
        elif j.name in act:
            j.damping=[0.3,0,0]; j.frictionloss=0.1   # reduktor surtunmesi (Asimov degerleri mertebesi)
    # kutlede armature zaten URDF atalet degerine dahil -> MuJoCo armature eklenmiyor
    key = spec.add_key(name='stand')
    return spec
if __name__=='__main__':
    spec=build(); m=spec.compile(); d=mujoco.MjData(m)
    print('nq',m.nq,'nv',m.nv,'nu',m.nu,'kutle',round(m.body_mass.sum(),2))
    open('mujoco/h175.xml','w').write(spec.to_xml())
