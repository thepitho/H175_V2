# H175 V2 -> V3 yamasi: motor secimi, kutle butcesi, elektronik yerlesimi, sigma kontrolu.
# build.py icinden exec ile calistirilir (links, joints, cq, math, json, P kullanir).
# UYARI: Katalog degerleri hafizadan / hedef spesifikasyon olarak girilmistir, veri sayfasindan dogrulanmalidir.
import numpy as np, csv

G = 9.81
# ad: (tepe Nm, nominal Nm, bos yuk hiz rad/s, kutle kg, cap mm, boy mm, not)
CATALOG = {
 'AK60-6':  (9.0,   3.0,  36.0, 0.29, 67, 37,  'CubeMars AK60-6 (yaklasik)'),
 'AK70-10': (24.8,  8.3,  38.0, 0.52, 90, 39,  'CubeMars AK70-10 (yaklasik)'),
 'AK10-9':  (53.0,  18.0, 38.0, 0.96, 100, 40, 'CubeMars AK10-9 (yaklasik)'),
 'AK80-64': (120.0, 48.0, 6.3,  0.85, 98, 39,  'CubeMars AK80-64 (yaklasik)'),
 'HT-200':  (200.0, 80.0, 11.0, 2.40, 125, 70, 'Ozel siparis modul, H1 sinifi hedef spesifikasyon'),
 'HT-300':  (300.0, 120.0, 8.0, 3.40, 140, 80, 'Ozel siparis modul, H1 diz sinifi hedef spesifikasyon'),
}
# yansitilmis rotor ataleti (armature, kg*m^2): Asimov sim-model'de eklem basina 0.03-0.11 kullanilir
ARMATURE = {'AK60-6':0.010,'AK70-10':0.025,'AK10-9':0.045,'AK80-64':0.095,'HT-200':0.110,'HT-300':0.150}
MARGIN = 1.25          # tepe tork / ihtiyac
ALU_T = 0.003; RHO = 2700.0

def need_torque(base, M, arm_payload=17.5, lift=35.0):
    """Eklem basina tepe tork ihtiyaci (Nm). Statik kol uzunluklari + yuk senaryolari (tahmin)."""
    mg = M*G
    t = {
     'hip_pitch': mg*0.20, 'hip_roll': mg*0.12, 'hip_yaw': mg*0.05, 'knee': mg*0.22,
     'ankle_pitch': mg*0.11, 'ankle_roll': mg*0.06, 'toe': 8.0,
     'waist_pitch': lift*G*0.45 + 0.35*M*G*0.12, 'waist_roll': 0.5*lift*G*0.2 + 25, 'waist_yaw': 40.0,
     'neck_yaw': 5.0, 'neck_pitch': 8.0, 'neck_roll': 5.0,
     'shoulder_pitch': arm_payload*G*0.75 + 20, 'shoulder_roll': 0.6*(arm_payload*G*0.75+20),
     'shoulder_yaw': 0.3*(arm_payload*G*0.75+20), 'elbow': arm_payload*G*0.40 + 6,
     'scapula': 30.0, 'clavicle': 20.0, 'forearm_twist': 15.0,
     'wrist_pitch': 17.0, 'wrist_roll': 12.0, 'wrist_yaw': 15.0,
    }
    return t.get(base)

def pick(tq):
    for k,v in sorted(CATALOG.items(), key=lambda kv: kv[1][0]):
        if v[0] >= MARGIN*tq: return k
    return None

def is_finger(base): return any(base.startswith(f) for f in ('index','middle','ring','little','thumb'))

def joint_base(n): return n[2:] if n[:2] in ('L_','R_') else n

# parmak eklemi tork siniri (Nm): tendon tahrikli mikro servolar, kuvvet 20 N uc, ~60 mm kol
FINGER_TQ = {'1': 1.5, '2': 0.9, '3': 0.5}
FINGER_VEL = 12.0
HAND_ACT_KG = 0.45   # el basina 15 mikro servo + surucu, on kolda

def structure_mass(prims, skip_first_cyl=True):
    a = 0.0; skipped = False
    for typ,x,y,z,*v in prims:
        if typ == 'cyl' and skip_first_cyl and not skipped: skipped = True; continue
        if typ == 'box':
            A,B,C = [d/1000 for d in v]; a += 2*(A*B+A*C+B*C)
        else:
            r,h,_ = v; r/=1000; h/=1000; a += 2*math.pi*r*h + 2*math.pi*r*r
    return max(a*ALU_T*RHO*0.6, 0.08)   # 0.6: kismi dolgu/kemik yapi

# ---------------- elektronik yerlesim (global mm; x ileri, y sol, z yukari) ----------------
# ad: (parent_link, merkez, boyut, kutle kg, aciklama)
ELEC = {
 'jetson_agx_orin': ('head', (-20,0,1600), (110,110,72), 1.00, 'Jetson AGX Orin dev kit, 110x110x72'),
 'cam_stereo_L':    ('head', (70, 45,1600), (35,30,30), 0.06, 'global shutter RGB 60fps'),
 'cam_stereo_R':    ('head', (70,-45,1600), (35,30,30), 0.06, 'global shutter RGB 60fps'),
 'tof_sensor':      ('head', (72,0,1640),   (30,20,15), 0.03, 'ToF'),
 'head_sensor_hub': ('head', (-20,0,1480),  (100,80,16), 0.12, 'sensor hub / IMU kart'),
 'battery_L':       ('torso',(-145, 62,1185),(80,115,200), 4.10, '48V paket, ~0.74 kWh, hot-swap'),
 'battery_R':       ('torso',(-145,-62,1185),(80,115,200), 4.10, '48V paket, ~0.74 kWh, hot-swap'),
 'pdb_estop':       ('torso',(50,0,1075),   (100,140,40), 0.60, 'guc dagitim + e-stop + kontaktor'),
 'rt_controller':   ('torso',(50,0,1130),   (100,80,30), 0.35, 'gercek zamanli kontrolcu / EtherCAT master'),
 'eth_can_hub':     ('torso',(50,0,1170),   (80,60,20), 0.20, 'Ethernet switch + CAN hub'),
 'torso_imu':       ('torso',(0,0,1190),    (40,40,15), 0.05, 'IMU'),
 'lidar_mid360':    ('torso',(125,0,1290),  (65,65,60), 0.27, 'lidar >=10 m, 360 derece'),
}
# bos hacimler (kasa ic bosluklari), 5 mm duvar; ('kutu merkez', boyut)
WALL = 5
CAVITY = {
 'head':  [((0,0,1585),(185,178,330)), ((83,0,1600),(35,155,95))],
 'torso': [((0,0,1190),(220,350,300)), ((-145,0,1185),(95,260,235))],
}
HOLLOW = {'head': [0], 'torso': [0,1]}   # prims indeksleri (head: 0. kutu, torso: 0 ve 1)

def inside(c, s, boxes, wall=WALL):
    lo = np.array(c)-np.array(s)/2; hi = np.array(c)+np.array(s)/2
    for bc,bs in boxes:
        bl = np.array(bc)-np.array(bs)/2+wall; bh = np.array(bc)+np.array(bs)/2-wall
        if np.all(lo>=bl-1e-9) and np.all(hi<=bh+1e-9): return True
    # kisim: birlesimin icinde (iki bosluk birlesimi) - merkez noktalarla yaklasik kontrol
    return False

def fit_report():
    rows = []
    for n,(par,c,s,m,d) in ELEC.items():
        ok = inside(c,s,CAVITY[par])
        res = 'DIS MONTAJ' if n.startswith('lidar') else ('SIGAR' if ok else 'SIGMAZ')
        rows.append((n,par,*c,*s,m,res,d))
    with open(P/'fit_check.csv','w',newline='',encoding='utf-8') as f:
        w = csv.writer(f,delimiter=';'); w.writerow(['parca','link','cx','cy','cz','sx','sy','sz','kg','sonuc','aciklama']); w.writerows(rows)
    return rows

# ---------------- iteratif motor secimi + kutle ----------------
def run_patch():
    elec_mass = {}
    for n,(par,c,s,m,d) in ELEC.items(): elec_mass[par] = elec_mass.get(par,0)+m
    M = 80.0
    for it in range(6):
        sel = {}; tot_act = 0; link_mass = {}
        for (jn,pa,ch,ax,lo,hi) in joints:
            b = joint_base(jn)
            if is_finger(b): continue
            tq = need_torque(b, M)
            cls = pick(tq)
            if cls is None: cls = 'HT-300'
            sel[jn] = (cls, tq)
        total = 0.0
        for n,(p,prims,m0) in links.items():
            jn = n  # link adi == eklem adi (cogu); toe/foot vs. da ayni
            act = 0.0
            if jn in sel: act = CATALOG[sel[jn][0]][3]
            if is_finger(joint_base(n)): mass = 0.045
            else: mass = structure_mass(prims, skip_first_cyl=(jn in sel)) + act + elec_mass.get(n,0)
            if joint_base(n) == 'forearm_twist': mass += HAND_ACT_KG
            link_mass[n] = mass; total += mass
        # torso/head gibi eklem adi farkli linkler: torso (waist_pitch), head (neck_pitch) -> aktuatoru cocuga ata
        for (jn,pa,ch,ax,lo,hi) in joints:
            if jn in sel and jn != ch:
                link_mass[ch] += CATALOG[sel[jn][0]][3]; total += CATALOG[sel[jn][0]][3]
        if abs(total-M) < 0.2: M = total; break
        M = total
    return sel, link_mass, M

sel, link_mass, M_TOTAL = run_patch()
# link kutlalarini yaz
for n,(p,prims,m0) in list(links.items()): links[n] = (p, prims, round(link_mass[n],3))

# aktuator silindirlerini katalog boyutuna getir (ilk silindir = aktuator, eksen korunur)
for (jn,pa,ch,ax,lo,hi) in joints:
    if jn in sel and jn == ch:
        cls = sel[jn][0]; _,_,_,_,dia,ln,_ = CATALOG[cls]
        p,prims,m = links[ch]; new = []; done = False
        for pr in prims:
            if pr[0]=='cyl' and not done:
                pr = list(pr); pr[4] = dia/2; pr[5] = ln; done = True
            new.append(pr)
        links[ch] = (p,new,m)

# armature: aktuatorlu eklemin cocuk linkine (disk rotor: eksen I, enine I/2)
ARM_BY_LINK = {}; ARM_AXIS = {}
for (jn,pa,ch,ax,lo,hi) in joints:
    if jn in sel: ARM_BY_LINK[ch] = ARMATURE[sel[jn][0]]; ARM_AXIS[ch] = ax
# eklem limitleri (effort/velocity) -> URDF
JLIM = {}
for (jn,pa,ch,ax,lo,hi) in joints:
    b = joint_base(jn)
    if is_finger(b): JLIM[jn] = (FINGER_TQ[b[-1]], FINGER_VEL)
    else:
        cls = sel[jn][0]; JLIM[jn] = (CATALOG[cls][0], CATALOG[cls][2])

def write_reports():
    with open(P/'motor_secimi.csv','w',newline='',encoding='utf-8') as f:
        w = csv.writer(f,delimiter=';')
        w.writerow(['eklem','motor','tepe_Nm','nominal_Nm','bos_hiz_rad_s','kg','cap_mm','boy_mm','ihtiyac_Nm','marj','not'])
        for jn,(cls,tq) in sel.items():
            c = CATALOG[cls]; w.writerow([jn,cls,c[0],c[1],c[2],c[3],c[4],c[5],round(tq,1),round(c[0]/tq,2),c[6]])
        for jn,(e,v) in JLIM.items():
            if is_finger(joint_base(jn)): w.writerow([jn,'mikro servo (tendon)',e,'','' ,'','','','','',''])
    out = {'total_mass_kg': M_TOTAL, 'joints': {jn:{'motor':sel[jn][0] if jn in sel else 'finger','tau_peak':JLIM[jn][0],
           'tau_rated': CATALOG[sel[jn][0]][1] if jn in sel else JLIM[jn][0]*0.5,'armature': ARMATURE[sel[jn][0]] if jn in sel else 0.0,'w_noload':JLIM[jn][1]} for jn in JLIM},
           'link_mass': {n:links[n][2] for n in links}}
    (P/'actuators.json').write_text(json.dumps(out,indent=1))
    counts = {}
    for jn,(cls,_) in sel.items(): counts[cls] = counts.get(cls,0)+1
    print('motor adetleri:',counts,'toplam kutle %.1f kg'%M_TOTAL)
