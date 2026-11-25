% H175_V2 - Simscape Multibody kontrol modeli uretici
% Girdi : H175_V2_multibody_raw.slx  (smimport ~8 dk surer; bir kez uretilip kaydedildi)
% Cikti : H175_V2_control.slx  -> 72 eklemde PD tork dongusu (MathWorks humanoid walker
%         ornegindeki gibi T = K*(q_ref - q) - B*qd), sabit pelvis, referans yorungesi.
% Not   : Gain/atalet degerleri konsept CAD'e gore tahmindir; dogrulanmamistir.
function build_h175_control(tStop)
if nargin<1, tStop = 4; end
root = fileparts(mfilename('fullpath')); cd(root);
src = 'H175_V2_multibody_raw'; mdl = 'H175_V2_control';
load_system(fullfile(root,[src '.slx']));
if bdIsLoaded(mdl), close_system(mdl,0); end
save_system(src, fullfile(root,[mdl '.slx']));   % kopya; model adi mdl olur
load_system(mdl);

rj = find_system(mdl,'SearchDepth',1,'LookUnderMasks','all','FollowLinks','on', ...
     'ReferenceBlock','sm_lib/Joints/Revolute Joint');
n = numel(rj); names = cell(n,1);
for i=1:n, names{i} = get_param(rj{i},'Name'); end
fprintf('%d revolute eklem\n', n);

%% Gain / sinir tablosu (isme gore)
[K,B] = deal(zeros(n,1));
for i=1:n
    nm = regexprep(names{i},'^[LR]_','');
    if any(strcmp(nm,{'hip_yaw','hip_roll','thigh','shin','ankle_pitch','foot','hip_aux','waist_yaw','waist_pitch','waist_roll'}))
        k=400; b=25;
    elseif any(strcmp(nm,{'toe'})),                  k=20;  b=1;
    elseif any(strcmp(nm,{'shoulder_yaw','shoulder_roll','upper_arm','scapula','arm_twist','forearm'})), k=120; b=8;
    elseif startsWith(nm,{'neck'}),                  k=15;  b=1;
    elseif any(strcmp(nm,{'wrist_yaw','wrist_roll','hand','forearm_twist'})), k=25; b=1.5;
    else,                                            k=0.6; b=0.02;   % parmaklar
    end
    K(i)=k; B(i)=b;
end
assignin('base','Kj',K); assignin('base','Bj',B); assignin('base','jointNames',names);

%% Referans yorungesi (From Workspace, 72 boyutlu)
f = 0.5; t = (0:0.01:tStop)'; q0 = zeros(n,1);
Q = repmat(q0',numel(t),1);
idx = @(nm) find(strcmp(names,nm));
w = 2*pi*f;
for s = {'L','R'}
    sd = s{1}; ph = pi*strcmp(sd,'R');
    Q(:,idx([sd '_thigh'])) = -0.30*sin(w*t+ph) - 0.15;
    Q(:,idx([sd '_shin']))  = 0.35 + 0.35*sin(w*t+ph+pi/2);
    Q(:,idx([sd '_ankle_pitch'])) = 0.15*sin(w*t+ph);
    Q(:,idx([sd '_upper_arm'])) = 0.35*sin(w*t+ph+pi);
    Q(:,idx([sd '_forearm'])) = -0.5 - 0.2*sin(w*t+ph+pi);
end
Q(:,idx('waist_yaw')) = 0.15*sin(w*t);
Q(:,idx('neck_yaw'))  = 0.30*sin(0.5*w*t);
qref = timeseries(Q,t); assignin('base','qref',qref);

%% Her eklem icin PD dongusu
x0 = 0; dy = 60;
for i=1:n
    jb = rj{i}; nm = names{i}; y = 100 + (i-1)*dy;
    set_param(jb,'TorqueActuationMode','InputTorque','MotionActuationMode','ComputedMotion', ...
        'SensePosition','on','SenseVelocity','on');
    pos = @(dx) [3000+dx y 3030+dx y+20];
    ps  = get_param(jb,'PortHandles');
    % sensorler -> Simulink
    cq = add_block('nesl_utility/PS-Simulink Converter',[mdl '/cq_' nm],'Position',pos(0),  'Unit','rad');
    cw = add_block('nesl_utility/PS-Simulink Converter',[mdl '/cw_' nm],'Position',pos(60), 'Unit','rad/s');
    ct = add_block('nesl_utility/Simulink-PS Converter',[mdl '/ct_' nm],'Position',pos(520),'Unit','N*m');
    gk = add_block('simulink/Math Operations/Gain',[mdl '/K_' nm],'Position',pos(240),'Gain',sprintf('Kj(%d)',i));
    gb = add_block('simulink/Math Operations/Gain',[mdl '/B_' nm],'Position',pos(300),'Gain',sprintf('Bj(%d)',i));
    sm = add_block('simulink/Math Operations/Sum',[mdl '/S_' nm],'Position',pos(180),'Inputs','+-');
    sm2= add_block('simulink/Math Operations/Sum',[mdl '/T_' nm],'Position',pos(420),'Inputs','+-');
    fr = add_block('simulink/Signal Routing/From',[mdl '/r_' nm],'Position',pos(120),'GotoTag',['qr_' nm]);
    pc = get_param(jb,'PortHandles');
    % ek ports (torque giris=LConn(2), q=RConn(2), w=RConn(3))
    add_line(mdl, pc.RConn(2), get_param(cq,'PortHandles').LConn(1));
    add_line(mdl, pc.RConn(3), get_param(cw,'PortHandles').LConn(1));
    add_line(mdl, get_param(ct,'PortHandles').RConn(1), pc.LConn(2));
    add_line(mdl, get_param(fr,'PortHandles').Outport(1), get_param(sm,'PortHandles').Inport(1));
    add_line(mdl, get_param(cq,'PortHandles').Outport(1), get_param(sm,'PortHandles').Inport(2));
    add_line(mdl, get_param(sm,'PortHandles').Outport(1), get_param(gk,'PortHandles').Inport(1));
    add_line(mdl, get_param(cw,'PortHandles').Outport(1), get_param(gb,'PortHandles').Inport(1));
    add_line(mdl, get_param(gk,'PortHandles').Outport(1), get_param(sm2,'PortHandles').Inport(1));
    add_line(mdl, get_param(gb,'PortHandles').Outport(1), get_param(sm2,'PortHandles').Inport(2));
    add_line(mdl, get_param(sm2,'PortHandles').Outport(1), get_param(ct,'PortHandles').Inport(1));
end

%% Referans dagitimi + kayit
wp = add_block('simulink/Sources/From Workspace',[mdl '/qref'],'Position',[2000 100 2100 140], ...
     'VariableName','qref','Interpolate','on','OutputAfterFinalValue','Holding final value');
dm = add_block('simulink/Signal Routing/Demux',[mdl '/qref_demux'],'Position',[2200 100 2205 100+n*dy], ...
     'Outputs',num2str(n));
add_line(mdl, get_param(wp,'PortHandles').Outport(1), get_param(dm,'PortHandles').Inport(1));
dp = get_param(dm,'PortHandles');
for i=1:n
    g = add_block('simulink/Signal Routing/Goto',[mdl '/g_' names{i}],'Position',[2300 100+(i-1)*dy 2360 120+(i-1)*dy], ...
        'GotoTag',['qr_' names{i}]);
    add_line(mdl, dp.Outport(i), get_param(g,'PortHandles').Inport(1));
end

%% Pelvisi sabitle: dunya baglantisini kontrol et (smimport World'e kaynak koyar)
set_param(mdl,'Solver','ode15s','StopTime',num2str(tStop),'RelTol','1e-3','AbsTol','1e-5');
% Loglama: tum eklem acilari sim cikisina (Simscape Logging)
set_param(mdl,'SimscapeLogType','all');
save_system(mdl, fullfile(root,[mdl '.slx']));
fprintf('Kaydedildi: %s\n', fullfile(root,[mdl '.slx']));
end
