function [TFF, LAM] = h175_gravity_ff(G, names, urdf)
% Yurume yorungesi boyunca yercekimi + zemin tepkisi ileri-besleme torku (kuasi-statik).
% Pelvis tabanda sabit modelle: zemin tepki kuvveti (toplam M*g) ZMP'ye gore ayaklara dagitilir ve
% inverseDynamics ile statik tutma torku bulunur. Cikti: TFF (nt x numel(names)), Nm.
robot = importrobot(urdf); robot.DataFormat='row'; robot.Gravity=[0 0 -9.81];
jn = cell(1,robot.NumBodies); for i=1:robot.NumBodies, jn{i}=robot.Bodies{i}.Joint.Name; end
map = zeros(1,numel(names)); for i=1:numel(names), map(i)=find(strcmp(jn,names{i})); end
M = 0; for i=1:robot.NumBodies, M = M + robot.Bodies{i}.Mass; end
fn = fieldnames(G.q); q0 = homeConfiguration(robot);
nt = numel(G.t); dtg = G.t(2)-G.t(1); tdd = @(X) gradient(X.',dtg).';   % zaman ekseninde turev (satir = zaman)
acom = tdd(tdd(G.com(:,1:2)));   % planlanan CoM ivmesi (LIPM)
% referans eklem hiz/ivmeleri (sonlu fark)
nb = robot.NumBodies; QQ = zeros(nt,nb); for ff_=1:numel(fn), c_ = find(strcmp(jn,fn{ff_})); if ~isempty(c_), QQ(:,c_) = G.q.(fn{ff_}); end, end
QD = movmean(tdd(QQ),5,1); QDD = movmean(tdd(QD),9,1);
step = 5;                    % 25 ms'de bir hesapla, sonra enterpole et
idx = unique([1:step:nt nt]); TF = zeros(numel(idx),numel(names)); LM = zeros(numel(idx),2); hipY = 0.105;
for a=1:numel(idx)
    k = idx(a); q = q0;
    for f=1:numel(fn), c = find(strcmp(jn,fn{f})); if ~isempty(c), q(c) = G.q.(fn{f})(k); end, end
    robot.Gravity = [-acom(k,1) -acom(k,2) -9.81];   % taban ivmesi (d'Alembert): g_eff = g - a_base
    swL = G.foot.L(k,3) > 1e-3; swR = G.foot.R(k,3) > 1e-3;
    if swL, lamL = 0; elseif swR, lamL = 1; else, lamL = min(max((G.zmp(k,2)+hipY)/(2*hipY),0),1); end
    lam = [lamL, 1-lamL]; LM(a,:) = lam; fext = zeros(robot.NumBodies,6);
    for si=1:2
        sd = 'LR'; sd = sd(si); 
        if lam(si)==0, continue; end
        T = getTransform(robot,q,[sd '_ankle_roll'],'pelvis'); po = T(1:3,4)';
        pc = [G.zmp(k,1)-G.pelvis(k,1), po(2), -G.pelvis(k,3)];     % CoP (pelvis cercevesi)
        F = [lam(si)*M*acom(k,1) lam(si)*M*acom(k,2) lam(si)*M*9.81]; Mo = cross(pc-po, F);   % yatay kuvvet: M*a_com
        row = find(strcmp(cellfun(@(b) b.Name, robot.Bodies,'uni',0),[sd '_ankle_roll']));
        fext(row,:) = [Mo F];
    end
    tau = inverseDynamics(robot,q,QD(k,:),QDD(k,:),fext);
    TF(a,:) = tau(map);
end
TFF = interp1(G.t(idx),TF,G.t,'linear','extrap'); LAM = interp1(G.t(idx),LM,G.t,'linear','extrap');
end
