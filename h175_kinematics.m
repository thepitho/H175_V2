% H175_V2 - ileri / ters kinematik (Robotics System Toolbox)
% Kullanim: h175_kinematics   (importrobot ~3 sn)
root = fileparts(mfilename('fullpath')); cd(root);
robot = importrobot(fullfile(root,'H175_V2.urdf'));
robot.DataFormat = 'row'; robot.Gravity = [0 0 -9.81];
q0 = homeConfiguration(robot);

%% Ileri kinematik: uc efektor konumlari (m, pelvis cercevesi)
ends = {'L_hand','R_hand','L_foot','R_foot','head'};
fprintf('\nNominal poz, pelvis cercevesine gore [x y z] (m):\n');
for k=1:numel(ends)
    T = getTransform(robot,q0,ends{k},'pelvis');
    fprintf('  %-8s % .3f % .3f % .3f\n',ends{k},T(1:3,4));
end

%% Ters kinematik: sol el 10 cm one + 10 cm yukari
% Sadece sol kol zinciri; diger eklemler sabit (limit lo=hi) tutulur.
ik = inverseKinematics('RigidBodyTree',robot);
w = [0.1 0.1 0.1 1 1 1];
Tt = getTransform(robot,q0,'L_hand','pelvis'); Tt(1:3,4) = Tt(1:3,4) + [0.10;0;0.10];
[qik,info] = ik('L_hand',Tt,w,q0);
Tr = getTransform(robot,qik,'L_hand','pelvis');
fprintf('\nIK durum: %s, konum hatasi %.2f mm\n',info.Status,1e3*norm(Tr(1:3,4)-Tt(1:3,4)));

%% Ayaga gore agirlik merkezi ve statik tork (yercekimi)
com = centerOfMass(robot,q0);
fprintf('Agirlik merkezi (pelvis): % .3f % .3f % .3f m\n',com);
gt = gravityTorque(robot,q0);
[~,i] = max(abs(gt)); jn = robot.homeConfiguration; %#ok<NASGU>
fprintf('Maks yercekimi torku: %.1f Nm\n',abs(gt(i)));
show(robot,qik); title('Sol el IK hedefi');
