function G = build_h175_walk(gaitOpt, simulate)
% H175 V3 - serbest govde + zemin temasi + motor modelli yuruyus modeli (Simscape Multibody).
% Girdi : H175_V3_multibody_raw.slx (smimport), actuators.json (build.py ciktisi), h175_gait.m
% Cikti : H175_V3_walk.slx ; G (yuruyus referansi). Hata ayiklama icin simulate=false.
if nargin<1, gaitOpt = struct; end
if nargin<2, simulate = false; end
root = fileparts(mfilename('fullpath')); cd(root);
src = 'H175_V3_multibody_raw'; mdl = 'H175_V3_walk';
load_system(fullfile(root,[src '.slx']));
if bdIsLoaded(mdl), close_system(mdl,0); end
save_system(src, fullfile(root,[mdl '.slx'])); load_system(mdl);
A = jsondecode(fileread(fullfile(root,'actuators.json')));

rj = find_system(mdl,'SearchDepth',1,'LookUnderMasks','all','FollowLinks','on','ReferenceBlock','sm_lib/Joints/Revolute Joint');
n = numel(rj); names = cell(n,1); for i=1:n, names{i}=get_param(rj{i},'Name'); end

%% Yuruyus referansi
G = h175_gait(gaitOpt);
t = G.t; Q = zeros(numel(t),n); q0 = zeros(n,1);
for i=1:n
    if isfield(G.q,names{i}), Q(:,i) = G.q.(names{i}); end
end
q0 = Q(1,:)';
qref = timeseries(Q,t); assignin('base','qref',qref); assignin('base','jointNames',names);
[TFF, LAM] = h175_gravity_ff(G, names, fullfile(root,'H175_V2.urdf')); assignin('base','tauff',timeseries(TFF,t));
assignin('base','lam',timeseries(LAM,t));
dtg = G.t(2)-G.t(1); assignin('base','pref',timeseries([G.pelvis(:,1:2), gradient(G.pelvis(:,1),dtg), gradient(G.pelvis(:,2),dtg)],t));
swL_ = double(G.foot.L(:,3) > 1e-3); swR_ = double(G.foot.R(:,3) > 1e-3); wsm = max(1,round(0.12/dtg));
assignin('base','swing',timeseries([movmean(swL_,wsm), movmean(swR_,wsm)],t));
Mtot = sum(cellfun(@(b) b.Mass, importrobot(fullfile(root,'H175_V2.urdf')).Bodies)); assignin('base','Mtot',Mtot);

%% Kazanclar ve motor parametreleri
Tpk = zeros(n,1); Wnl = zeros(n,1); Kj = zeros(n,1); Bj = zeros(n,1);
for i=1:n
    a = A.joints.(names{i}); Tpk(i)=a.tau_peak; Wnl(i)=a.w_noload;
    if strcmp(a.motor,'finger'), Kj(i)=0.6; Bj(i)=0.05;
    else
        sag = any(strcmp(regexprep(names{i},'^[LR]_',''),{'hip_pitch','knee','ankle_pitch'}));   % sagital: yumusak (cift destek uyumsuzlugu)
        if sag, Kj(i)=Tpk(i)/0.25; Bj(i)=0.15*Kj(i); else, Kj(i)=Tpk(i)/0.06; Bj(i)=0.12*Kj(i); end
    end
end
assignin('base','Kj',Kj); assignin('base','Bj',Bj); assignin('base','Tpk',Tpk); assignin('base','Wnl',Wnl);

%% Serbest govde: World - pelvis arasina 6-DOF eklem
w = [mdl '/World']; pel = [mdl '/pelvis'];
pph = get_param(pel,'PortHandles'); lh = get_param(pph.LConn(1),'Line');
delete_line(lh);
fb = add_block('sm_lib/Joints/6-DOF Joint',[mdl '/FloatingBase'],'Position',[-600 -300 -520 -220]);
fph = get_param(fb,'PortHandles'); wph = get_param(w,'PortHandles');
add_line(mdl, wph.RConn(1), fph.LConn(1),'autorouting','on');
add_line(mdl, fph.RConn(1), pph.LConn(1),'autorouting','on');
% World'e bagli bloklar (mekanizma/cozucu) hala ayni dugume bagli mi?
bs = find_system(mdl,'SearchDepth',1,'Type','block');
for k=1:numel(bs)
    nm_ = get_param(bs{k},'Name');
    if contains(nm_,'Mechanism') || contains(nm_,'Solver')
        ph = get_param(bs{k},'PortHandles');
        if get_param(ph.RConn(1),'Line') < 0, add_line(mdl, wph.RConn(1), ph.RConn(1)); fprintf('yeniden baglandi: %s\n',strrep(nm_,newline,' ')); end
    end
end
% baslangic pozu (pelvis konumu = dunya cercevesi, ayaklar zemin z=0)
set_param(fb,'PxPositionTargetSpecify','on','PxPositionTargetValue',num2str(G.pelvis(1,1)),'PxPositionTargetPriority','High');
set_param(fb,'PyPositionTargetSpecify','on','PyPositionTargetValue',num2str(G.pelvis(1,2)),'PyPositionTargetPriority','High');
set_param(fb,'PzPositionTargetSpecify','on','PzPositionTargetValue',num2str(G.pelvis(1,3)+0.0005),'PzPositionTargetPriority','High');
for ax = {'Px','Py','Pz'}, set_param(fb,[ax{1} 'SensePosition'],'on',[ax{1} 'SenseVelocity'],'on'); end
set_param(fb,'SphSensePosition','on');

%% Zemin
gb = add_block('sm_lib/Body Elements/Brick Solid',[mdl '/Ground'],'Position',[-600 -150 -520 -90], ...
     'BrickDimensions','[30 6 0.2]','BrickDimensionsUnits','m','GraphicDiffuseColor','[0.55 0.58 0.6]');
gt = add_block('sm_lib/Frames and Transforms/Rigid Transform',[mdl '/GroundTf'],'Position',[-700 -150 -640 -110], ...
     'TranslationMethod','Cartesian','TranslationCartesianOffset','[12 0 -0.1]','TranslationCartesianOffsetUnits','m');
fr = @(b) frameport(b);
add_line(mdl, wph.RConn(1), get_param(gt,'PortHandles').LConn(1));
add_line(mdl, get_param(gt,'PortHandles').RConn(1), fr(gb));
gndTfOut = get_param(gt,'PortHandles').RConn(1);

%% Ayak temas kureleri (her ayakta 4)
sph = [-0.070  0.050; -0.070 -0.050; 0.110 0.050; 0.110 -0.050]; zS = -0.074; rS = 0.006;
cnt = 0;
for sd = {'L','R'}
    foot = [mdl '/' sd{1} '_ankle_roll1']; fph_ = get_param(foot,'PortHandles');
    for k=1:4
        cnt = cnt+1; y0 = -400 + cnt*70;
        % alt-sistemin F portu eklem eksenine hizali: AxisInvTransform donusunu tersle, sonra ofseti link eksenlerinde uygula
        axb = [foot '/' sd{1} '_ankle_roll_AxisInvTransform'];
        r0 = add_block('sm_lib/Frames and Transforms/Rigid Transform',sprintf('%s/ct_rot_%s%d',mdl,sd{1},k),'Position',[-700 y0 -640 y0+30], ...
             'RotationMethod','ArbitraryAxis','RotationArbitraryAxis',get_param(axb,'RotationArbitraryAxis'), ...
             'RotationAngle',num2str(-str2double(get_param(axb,'RotationAngle'))),'RotationAngleUnits',get_param(axb,'RotationAngleUnits'));
        tf = add_block('sm_lib/Frames and Transforms/Rigid Transform',sprintf('%s/ct_tf_%s%d',mdl,sd{1},k),'Position',[-600 y0 -540 y0+30], ...
             'TranslationMethod','Cartesian','TranslationCartesianOffset',sprintf('[%g %g %g]',sph(k,1),sph(k,2),zS),'TranslationCartesianOffsetUnits','m');
        add_line(mdl, get_param(r0,'PortHandles').RConn(1), get_param(tf,'PortHandles').LConn(1));
        so = add_block('sm_lib/Body Elements/Spherical Solid',sprintf('%s/ct_sph_%s%d',mdl,sd{1},k),'Position',[-480 y0 -420 y0+30], ...
             'SphereRadius',num2str(rS),'SphereRadiusUnits','m','GraphicDiffuseColor','[0.9 0.2 0.2]','ExportEntireGeometry','on');
        cf = add_block('sm_lib/Forces and Torques/Spatial Contact Force',sprintf('%s/Contact_%s%d',mdl,sd{1},k),'Position',[-380 y0 -320 y0+40], ...
             'NormalStiffness','1e6','NormalDamping','2e3','NormalTransitionRegionWidth','1e-4', ...
             'FrictionType','SmoothStickSlip','CoefficientOfStaticFriction','1.4','CoefficientOfDynamicFriction','1.2', ...
             'FrictionalCriticalVelocity','0.01','SenseNormalForceMagnitude','on','SensePenetrationDepth','on');
        add_line(mdl, fph_.LConn(1), get_param(r0,'PortHandles').LConn(1));
        add_line(mdl, get_param(tf,'PortHandles').RConn(1), fr(so));
        cph = get_param(cf,'PortHandles');
        % her temas cifti icin ayri (grafiksiz) zemin kopyasi: geometri portu dallanmaz
        gc = add_block('sm_lib/Body Elements/Brick Solid',sprintf('%s/gnd_%s%d',mdl,sd{1},k),'Position',[-620 y0 -560 y0+30], ...
             'BrickDimensions','[30 6 0.2]','BrickDimensionsUnits','m','ExportEntireGeometry','on','GraphicType','NONE');
        add_line(mdl, gndTfOut, fr(gc));
        add_line(mdl, get_param(gc,'PortHandles').LConn(1), cph.LConn(1));
        add_line(mdl, get_param(so,'PortHandles').LConn(1), cph.RConn(1));
    end
end

%% Ekleme PD + motor modeli (tork/hiz doygunlugu)
dy = 60; tauPorts = zeros(n,1);
for i=1:n
    jb = rj{i}; nm = names{i}; y = 100 + (i-1)*dy; X = 3000;
    set_param(jb,'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion','SensePosition','on','SenseVelocity','on');
    set_param(jb,'PositionTargetSpecify','on','PositionTargetPriority','High', ...
        'PositionTargetValue',num2str(q0(i),'%.8g'),'PositionTargetValueUnits','rad');
    set_param(jb,'DampingCoefficient','0.3');          % dişli/yatak surtunmesi (viskoz) Nm*s/rad
    pc = get_param(jb,'PortHandles');
    P_ = @(x,dx) [X+dx y X+dx+30 y+20];
    cq = add_block('nesl_utility/PS-Simulink Converter',[mdl '/cq_' nm],'Position',P_(0,0),'Unit','rad');
    cw = add_block('nesl_utility/PS-Simulink Converter',[mdl '/cw_' nm],'Position',P_(0,60),'Unit','rad/s');
    ct = add_block('nesl_utility/Simulink-PS Converter',[mdl '/ct_' nm],'Position',P_(0,900),'Unit','N*m');
    fr_ = add_block('simulink/Signal Routing/From',[mdl '/r_' nm],'Position',P_(0,120),'GotoTag',['qr_' nm]);
    sm  = add_block('simulink/Math Operations/Sum',[mdl '/S_' nm],'Position',P_(0,180),'Inputs','+-');
    stepJ = any(strcmp(regexprep(nm,'^[LR]_',''),{'hip_pitch','ankle_pitch'}));
    if stepJ, set_param(sm,'Inputs','+-+'); fd_ = add_block('simulink/Signal Routing/From',[mdl '/dq_' nm],'Position',P_(0,1360),'GotoTag',['dq_' nm]); end
    gk  = add_block('simulink/Math Operations/Gain',[mdl '/K_' nm],'Position',P_(0,240),'Gain',sprintf('Kj(%d)',i));
    gb  = add_block('simulink/Math Operations/Gain',[mdl '/B_' nm],'Position',P_(0,300),'Gain',sprintf('Bj(%d)',i));
    sm2 = add_block('simulink/Math Operations/Sum',[mdl '/T_' nm],'Position',P_(0,360),'Inputs','+-+');
    sagLeg = any(strcmp(regexprep(nm,'^[LR]_',''),{'hip_pitch','knee','ankle_pitch'}));
    isAnk = any(strcmp(nm,{'L_ankle_pitch','R_ankle_pitch','L_ankle_roll','R_ankle_roll'}));
    if isAnk, set_param(sm2,'Inputs','+-++'); fb_ = add_block('simulink/Signal Routing/From',[mdl '/fb_' nm],'Position',P_(0,1060),'GotoTag',['fb_' nm]); end
    ff_ = add_block('simulink/Signal Routing/From',[mdl '/tf_' nm],'Position',P_(0,1000),'GotoTag',['tf_' nm]);
    % motor: ust sinir = Tpk*sat(1-|w|/wnl,0,1)
    ab  = add_block('simulink/Math Operations/Abs',[mdl '/abs_' nm],'Position',P_(0,420));
    g1  = add_block('simulink/Math Operations/Gain',[mdl '/gw_' nm],'Position',P_(0,480),'Gain',sprintf('-1/Wnl(%d)',i));   % tam tork |w|<=Wnl, 2*Wnl'de sifir (Asimov: sadece efor siniri)
    bi  = add_block('simulink/Math Operations/Bias',[mdl '/bi_' nm],'Position',P_(0,540),'Bias','2');
    sa  = add_block('simulink/Discontinuities/Saturation',[mdl '/sat_' nm],'Position',P_(0,600),'UpperLimit','1','LowerLimit','0');
    gt_ = add_block('simulink/Math Operations/Gain',[mdl '/gT_' nm],'Position',P_(0,660),'Gain',sprintf('Tpk(%d)',i));
    gn  = add_block('simulink/Math Operations/Gain',[mdl '/gn_' nm],'Position',P_(0,720),'Gain','-1');
    mn  = add_block('simulink/Math Operations/MinMax',[mdl '/mn_' nm],'Position',P_(0,780),'Function','min','Inputs','2');
    mx  = add_block('simulink/Math Operations/MinMax',[mdl '/mx_' nm],'Position',P_(0,840),'Function','max','Inputs','2');
    H = @(b) get_param(b,'PortHandles');
    add_line(mdl, pc.RConn(2), H(cq).LConn(1));
    add_line(mdl, pc.RConn(3), H(cw).LConn(1));
    add_line(mdl, H(ct).RConn(1), pc.LConn(2));
    add_line(mdl, H(fr_).Outport(1), H(sm).Inport(1));
    add_line(mdl, H(cq).Outport(1), H(sm).Inport(2));
    if stepJ, add_line(mdl, H(fd_).Outport(1), H(sm).Inport(3)); end
    add_line(mdl, H(sm).Outport(1), H(gk).Inport(1));
    add_line(mdl, H(cw).Outport(1), H(gb).Inport(1));
    if sagLeg
        pk = add_block('simulink/Math Operations/Product',[mdl '/pk_' nm],'Position',P_(0,1120));
        pb = add_block('simulink/Math Operations/Product',[mdl '/pb_' nm],'Position',P_(0,1180));
        f1 = add_block('simulink/Signal Routing/From',[mdl '/sk_' nm],'Position',P_(0,1240),'GotoTag',['sch_' nm(1)]);
        f2 = add_block('simulink/Signal Routing/From',[mdl '/sb_' nm],'Position',P_(0,1300),'GotoTag',['sch_' nm(1)]);
        add_line(mdl, H(gk).Outport(1), H(pk).Inport(1)); add_line(mdl, H(f1).Outport(1), H(pk).Inport(2)); add_line(mdl, H(pk).Outport(1), H(sm2).Inport(1));
    else
        add_line(mdl, H(gk).Outport(1), H(sm2).Inport(1));
    end
    if sagLeg
        add_line(mdl, H(gb).Outport(1), H(pb).Inport(1)); add_line(mdl, H(f2).Outport(1), H(pb).Inport(2)); add_line(mdl, H(pb).Outport(1), H(sm2).Inport(2));
    else
        add_line(mdl, H(gb).Outport(1), H(sm2).Inport(2));
    end
    add_line(mdl, H(ff_).Outport(1), H(sm2).Inport(3));
    if isAnk, add_line(mdl, H(fb_).Outport(1), H(sm2).Inport(4)); end
    add_line(mdl, H(cw).Outport(1), H(ab).Inport(1));
    add_line(mdl, H(ab).Outport(1), H(g1).Inport(1));
    add_line(mdl, H(g1).Outport(1), H(bi).Inport(1));
    add_line(mdl, H(bi).Outport(1), H(sa).Inport(1));
    add_line(mdl, H(sa).Outport(1), H(gt_).Inport(1));
    add_line(mdl, H(gt_).Outport(1), H(gn).Inport(1));
    add_line(mdl, H(sm2).Outport(1), H(mn).Inport(1));
    add_line(mdl, H(gt_).Outport(1), H(mn).Inport(2));
    add_line(mdl, H(mn).Outport(1), H(mx).Inport(1));
    add_line(mdl, H(gn).Outport(1), H(mx).Inport(2));
    add_line(mdl, H(mx).Outport(1), H(ct).Inport(1));
    tauPorts(i) = H(mx).Outport(1);
end


%% Denge geri beslemesi (ayak bilegi stratejisi): pelvis konum/hiz hatasi -> destek ayagi ayak bilegi torku
H_ = @(b) get_param(b,'PortHandles'); sensN = {'X','Y','XDot','YDot'}; sensU = {'m','m','m/s','m/s'}; sig = zeros(1,4);
for q=1:4
    ts = add_block('sm_lib/Frames and Transforms/Transform Sensor',[mdl '/ps_' sensN{q}],'Position',[-900 -300+q*70 -840 -250+q*70]);
    set_param(ts,['Sense' sensN{q}],'on','MeasurementFrame','World');
    add_line(mdl, wph.RConn(1), H_(ts).LConn(1)); add_line(mdl, pph.LConn(1), H_(ts).RConn(1));
    cv = add_block('nesl_utility/PS-Simulink Converter',[mdl '/pc_' sensN{q}],'Position',[-780 -300+q*70 -750 -280+q*70],'Unit',sensU{q});
    add_line(mdl, H_(ts).RConn(2), H_(cv).LConn(1)); sig(q) = H_(cv).Outport(1);
end
wr = add_block('simulink/Sources/From Workspace',[mdl '/pref'],'Position',[-900 -650 -830 -620],'VariableName','pref','Interpolate','on','OutputAfterFinalValue','Holding final value');
dr = add_block('simulink/Signal Routing/Demux',[mdl '/pref_demux'],'Position',[-800 -650 -795 -560],'Outputs','4');
add_line(mdl, H_(wr).Outport(1), H_(dr).Inport(1));
wl = add_block('simulink/Sources/From Workspace',[mdl '/lam'],'Position',[-900 -520 -830 -490],'VariableName','lam','Interpolate','on','OutputAfterFinalValue','Holding final value');
dl = add_block('simulink/Signal Routing/Demux',[mdl '/lam_demux'],'Position',[-800 -520 -795 -470],'Outputs','2');
add_line(mdl, H_(wl).Outport(1), H_(dl).Inport(1));
e = zeros(1,4);   % ex, ey, evx, evy
refIdx = [1 2 3 4];
for q=1:4
    se = add_block('simulink/Math Operations/Sum',[mdl '/err_' sensN{q}],'Position',[-700 -650+q*40 -680 -630+q*40],'Inputs','+-');
    add_line(mdl, sig(q), H_(se).Inport(1)); add_line(mdl, get_param(dr,'PortHandles').Outport(refIdx(q)), H_(se).Inport(2));
    e(q) = H_(se).Outport(1);
end
Kfx = 'Mtot*9.81*3.0'; Kdx = 'Mtot*9.81*1.1'; Kfy = 'Mtot*9.81*2.0'; Kdy = 'Mtot*9.81*0.6';
mkc = @(nm,gn,src,pos) add_block('simulink/Math Operations/Gain',[mdl '/' nm],'Position',pos,'Gain',gn);
gxp = mkc('fb_gxp',Kfx,e(1),[-600 -640 -560 -620]); gxd = mkc('fb_gxd',Kdx,e(3),[-600 -600 -560 -580]);
gyp = mkc('fb_gyp',['-' Kfy],e(2),[-600 -560 -560 -540]); gyd = mkc('fb_gyd',['-' Kdy],e(4),[-600 -520 -560 -500]);
add_line(mdl,e(1),H_(gxp).Inport(1)); add_line(mdl,e(3),H_(gxd).Inport(1)); add_line(mdl,e(2),H_(gyp).Inport(1)); add_line(mdl,e(4),H_(gyd).Inport(1));
sx = add_block('simulink/Math Operations/Sum',[mdl '/fb_sx'],'Position',[-520 -630 -500 -590],'Inputs','++'); add_line(mdl,H_(gxp).Outport(1),H_(sx).Inport(1)); add_line(mdl,H_(gxd).Outport(1),H_(sx).Inport(2));
sy = add_block('simulink/Math Operations/Sum',[mdl '/fb_sy'],'Position',[-520 -550 -500 -510],'Inputs','++'); add_line(mdl,H_(gyp).Outport(1),H_(sy).Inport(1)); add_line(mdl,H_(gyd).Outport(1),H_(sy).Inport(2));
satx = add_block('simulink/Discontinuities/Saturation',[mdl '/fb_satx'],'Position',[-470 -630 -440 -600],'UpperLimit','Mtot*9.81*0.08','LowerLimit','-Mtot*9.81*0.08');
saty = add_block('simulink/Discontinuities/Saturation',[mdl '/fb_saty'],'Position',[-470 -550 -440 -520],'UpperLimit','Mtot*9.81*0.05','LowerLimit','-Mtot*9.81*0.05');
add_line(mdl,H_(sx).Outport(1),H_(satx).Inport(1)); add_line(mdl,H_(sy).Outport(1),H_(saty).Inport(1));
dlp = get_param(dl,'PortHandles'); fbsrc = {satx,'ankle_pitch'; saty,'ankle_roll'};
for si=1:2
    sd = 'LR'; sd = sd(si);
    for k=1:2
        pm = add_block('simulink/Math Operations/Product',[mdl '/fb_pm_' sd num2str(k)],'Position',[-400 -640+(si-1)*120+(k-1)*50 -380 -620+(si-1)*120+(k-1)*50]);
        add_line(mdl,H_(fbsrc{k,1}).Outport(1),H_(pm).Inport(1)); add_line(mdl,dlp.Outport(si),H_(pm).Inport(2));
        g = add_block('simulink/Signal Routing/Goto',[mdl '/fb_g_' sd num2str(k)],'Position',[-340 -640+(si-1)*120+(k-1)*50 -300 -620+(si-1)*120+(k-1)*50],'GotoTag',['fb_' sd '_' fbsrc{k,2}]);
        add_line(mdl,H_(pm).Outport(1),H_(g).Inport(1));
    end
end

%% Kazanc planlamasi: salinan bacak sagital eklemleri sert (x4), destek yumusak
wsw = add_block('simulink/Sources/From Workspace',[mdl '/swing'],'Position',[-900 -780 -830 -750],'VariableName','swing','Interpolate','on','OutputAfterFinalValue','Holding final value');
dsw = add_block('simulink/Signal Routing/Demux',[mdl '/swing_demux'],'Position',[-800 -780 -795 -730],'Outputs','2');
add_line(mdl, get_param(wsw,'PortHandles').Outport(1), get_param(dsw,'PortHandles').Inport(1));
for si=1:2
    sd = 'LR'; sd = sd(si);
    gs = add_block('simulink/Math Operations/Gain',[mdl '/sch_g_' sd],'Position',[-700 -780+si*40 -660 -760+si*40],'Gain','Ksw-1');
    bs = add_block('simulink/Math Operations/Bias',[mdl '/sch_b_' sd],'Position',[-640 -780+si*40 -600 -760+si*40],'Bias','1');
    gg = add_block('simulink/Signal Routing/Goto',[mdl '/sch_t_' sd],'Position',[-580 -780+si*40 -540 -760+si*40],'GotoTag',['sch_' sd]);
    add_line(mdl, get_param(dsw,'PortHandles').Outport(si), get_param(gs,'PortHandles').Inport(1));
    add_line(mdl, get_param(gs,'PortHandles').Outport(1), get_param(bs,'PortHandles').Inport(1));
    add_line(mdl, get_param(bs,'PortHandles').Outport(1), get_param(gg,'PortHandles').Inport(1));
end
assignin('base','Ksw',4);

%% Basamaklama stratejisi: DCM hatasi -> salinan bacak kalca pitch / ayak bilegi duzeltmesi (iniş noktasini uzat/kisalt)
zc_ = G.info.zc; om_ = sqrt(9.81/zc_);
g1 = add_block('simulink/Math Operations/Gain',[mdl '/st_gv'],'Position',[-300 -780 -260 -760],'Gain',num2str(1/om_));
add_line(mdl, e(3), H_(g1).Inport(1));
sxi = add_block('simulink/Math Operations/Sum',[mdl '/st_xi'],'Position',[-240 -780 -220 -740],'Inputs','++');
add_line(mdl, e(1), H_(sxi).Inport(1)); add_line(mdl, H_(g1).Outport(1), H_(sxi).Inport(2));
gk_ = add_block('simulink/Math Operations/Gain',[mdl '/st_k'],'Position',[-200 -780 -160 -760],'Gain','-Kstep/0.72');
add_line(mdl, H_(sxi).Outport(1), H_(gk_).Inport(1));
sst = add_block('simulink/Discontinuities/Saturation',[mdl '/st_sat'],'Position',[-140 -780 -100 -760],'UpperLimit','0.25','LowerLimit','-0.25');
add_line(mdl, H_(gk_).Outport(1), H_(sst).Inport(1));
for si=1:2
    sd = 'LR'; sd = sd(si);
    pm = add_block('simulink/Math Operations/Product',[mdl '/st_pm_' sd],'Position',[-80 -780+si*40 -60 -760+si*40]);
    add_line(mdl, H_(sst).Outport(1), H_(pm).Inport(1)); add_line(mdl, get_param(dsw,'PortHandles').Outport(si), H_(pm).Inport(2));
    gt1 = add_block('simulink/Signal Routing/Goto',[mdl '/st_g1_' sd],'Position',[-40 -780+si*40 0 -760+si*40],'GotoTag',['dq_' sd '_hip_pitch']);
    gn1 = add_block('simulink/Math Operations/Gain',[mdl '/st_n_' sd],'Position',[-40 -740+si*40 0 -720+si*40],'Gain','-1');
    gt2 = add_block('simulink/Signal Routing/Goto',[mdl '/st_g2_' sd],'Position',[20 -740+si*40 60 -720+si*40],'GotoTag',['dq_' sd '_ankle_pitch']);
    add_line(mdl, H_(pm).Outport(1), H_(gt1).Inport(1)); add_line(mdl, H_(pm).Outport(1), H_(gn1).Inport(1)); add_line(mdl, H_(gn1).Outport(1), H_(gt2).Inport(1));
end
assignin('base','Kstep',0.0);   % basamaklama duzeltmesi: denendi, fark yaratmadi -> kapali

%% Referans dagitimi
wp = add_block('simulink/Sources/From Workspace',[mdl '/qref'],'Position',[2000 100 2100 140], ...
     'VariableName','qref','Interpolate','on','OutputAfterFinalValue','Holding final value');
dm = add_block('simulink/Signal Routing/Demux',[mdl '/qref_demux'],'Position',[2200 100 2205 100+n*dy],'Outputs',num2str(n));
add_line(mdl, get_param(wp,'PortHandles').Outport(1), get_param(dm,'PortHandles').Inport(1));
dp = get_param(dm,'PortHandles');
for i=1:n
    g = add_block('simulink/Signal Routing/Goto',[mdl '/g_' names{i}],'Position',[2300 100+(i-1)*dy 2360 120+(i-1)*dy],'GotoTag',['qr_' names{i}]);
    add_line(mdl, dp.Outport(i), get_param(g,'PortHandles').Inport(1));
end
wf = add_block('simulink/Sources/From Workspace',[mdl '/tauff'],'Position',[2000 -100 2100 -60],'VariableName','tauff','Interpolate','on','OutputAfterFinalValue','Holding final value');
df = add_block('simulink/Signal Routing/Demux',[mdl '/tauff_demux'],'Position',[2200 -100 2205 -100+n*dy],'Outputs',num2str(n));
add_line(mdl, get_param(wf,'PortHandles').Outport(1), get_param(df,'PortHandles').Inport(1));
dq = get_param(df,'PortHandles');
for i=1:n
    g = add_block('simulink/Signal Routing/Goto',[mdl '/gf_' names{i}],'Position',[2400 100+(i-1)*dy 2460 120+(i-1)*dy],'GotoTag',['tf_' names{i}]);
    add_line(mdl, dq.Outport(i), get_param(g,'PortHandles').Inport(1));
end
%% Tork kaydi (motor ciktisi): Mux -> To Workspace
mu = add_block('simulink/Signal Routing/Mux',[mdl '/tau_mux'],'Position',[4500 100 4505 100+n*dy],'Inputs',num2str(n));
tw = add_block('simulink/Sinks/To Workspace',[mdl '/tau_log'],'Position',[4600 100 4680 130],'VariableName','tau_log','SaveFormat','Array');
for i=1:n, add_line(mdl, tauPorts(i), get_param(mu,'PortHandles').Inport(i)); end
add_line(mdl, get_param(mu,'PortHandles').Outport(1), get_param(tw,'PortHandles').Inport(1));

%% Cozucu (degisken adim ode23t; yerel Backward-Euler cozucu 6-DOF govdede dejenere kutle hatasi verdi)
set_param(mdl,'SolverType','Variable-step','Solver','ode23t','StopTime',num2str(G.t(end)),'MaxStep','2e-3','RelTol','1e-3','AbsTol','1e-4');
set_param(mdl,'SimscapeLogType','all','SimscapeLogLimitData','off','SimscapeLogDecimation',5);
save_system(mdl, fullfile(root,[mdl '.slx']));
fprintf('Kaydedildi: %s.slx  (govde %d eklem, tEnd=%.1f s)\n',mdl,n,G.t(end));
if simulate
    tic; out = sim(mdl); fprintf('sim %.1f s\n',toc); assignin('base','out',out);
end
end

function p = frameport(b)
ph = get_param(b,'PortHandles'); p = ph.RConn(1);   % solid: R = referans cercevesi (RConn)
end
