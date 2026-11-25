function G = h175_gait(opt)
% ZMP/LIPM (Kajita on-bakisli kontrol) ile yuruyus referansi + analitik bacak IK.
% opt alanlari (hepsi opsiyonel):
%   vx (m/s)=0.3, Ts (s)=0.7, dsr=0.2, zh (kalca yuksekligi m)=0.80, nSteps=8, stepH=0.05,
%   T0=1.5 (baslangic bekleme), T1=1.5 (bitis bekleme), dt=0.005, urdf
% Cikti G: .t, .q (struct: eklem adi -> sutun vektor), .pelvis [x y z], .com, .zmp, .foot{L,R}, .info
if nargin<1, opt=struct; end
refine = ~isfield(opt,'comOffset') || isempty(opt.comOffset);
def = struct('vx',0.3,'Ts',0.7,'dsr',0.2,'zh',0.80,'nSteps',8,'stepH',0.05,'T0',1.5,'T1',1.5,'dt',0.005, ...
             'urdf',fullfile(fileparts(mfilename('fullpath')),'H175_V2.urdf'),'comOffset',[]);
fn = fieldnames(def); for i=1:numel(fn), if ~isfield(opt,fn{i}), opt.(fn{i})=def.(fn{i}); end, end
g = 9.81; dt = opt.dt; Ts = opt.Ts; L = opt.vx*opt.Ts;
hipY = 0.105; zHipOff = 0.045;     % pelvis orijininden kalca merkezi: 45 mm asagi (kuresel kalca)
l1 = 0.415; l2 = 0.385; zAnk = 0.080;

%% CoM ofseti (pelvis cercevesinde): URDF'den, duz bacak
robot = importrobot(opt.urdf); robot.DataFormat='row';
if isempty(opt.comOffset)
    comP = centerOfMass(robot, homeConfiguration(robot));
    opt.comOffset = comP(:)';           % pelvis cercevesi, m
end
off = opt.comOffset;
zp = opt.zh + zHipOff;                  % pelvis yuksekligi
zc = zp + off(3);                       % LIPM CoM yuksekligi

%% Adim plani: sag destek, sol sallanir ile baslar
N = opt.nSteps;
landX = ((1:N)-0.5)*L; landX(end) = landX(end-1);   % son adim: ayaklar yan yana
% adim i: i tek -> destek R, sallanan L ; i cift -> destek L, sallanan R
supY = @(i) -hipY*(mod(i,2)==1) + hipY*(mod(i,2)==0);
footL0 = 0; footR0 = 0;
% Destek ayak x konumu (adim i): onceki iniste olan ayak
suppX = zeros(1,N); sx_L = 0; sx_R = 0;
for i=1:N
    if mod(i,2)==1, suppX(i)=sx_R; sx_L = landX(i); else, suppX(i)=sx_L; sx_R = landX(i); end
end
% zaman cizelgesi
T0 = opt.T0; T1 = opt.T1; Ttot = T0 + N*Ts + T1; n = round(Ttot/dt)+1; t = (0:n-1)'*dt;
zmpx = zeros(n,1); zmpy = zeros(n,1);
for k=1:n
    tk = t(k);
    if tk < T0
        zmpx(k)=0; zmpy(k)=0;
    elseif tk < T0+N*Ts
        i = floor((tk-T0)/Ts)+1; ph = (tk-T0-(i-1)*Ts)/Ts;
        cur = [suppX(i) supY(i)];
        if i==1, pv=[0 0]; else, pv=[suppX(i-1) supY(i-1)]; end
        if ph < opt.dsr, a = ph/opt.dsr; a = a*a*(3-2*a); z = (1-a)*pv + a*cur; else, z = cur; end
        zmpx(k)=z(1); zmpy(k)=z(2);
    else
        cur = [suppX(N) supY(N)];
        a = min((tk-(T0+N*Ts))/0.6,1); a=a*a*(3-2*a); fin=[landX(end) 0]; z=(1-a)*cur+a*fin; zmpx(k)=z(1); zmpy(k)=z(2);
    end
end
%% Kajita on-bakisli kontrol
Np = round(1.6/dt);
A=[1 dt dt^2/2;0 1 dt;0 0 1]; B=[dt^3/6;dt^2/2;dt]; C=[1 0 -zc/g];
Ab=[1 C*A;zeros(3,1) A]; Bb=[C*B;B]; Ib=[1;0;0;0]; Fb=[C*A;A];
Qb=blkdiag(1,zeros(3)); Rr=1e-6; P=idare(Ab,Bb,Qb,Rr);
Kb=(Rr+Bb'*P*Bb)\(Bb'*P*Ab); Ke=Kb(1); Kx=Kb(2:4); Ac=Ab-Bb*Kb;
Gd=zeros(Np,1); Gd(1)=-Ke; X=-Ac'*P*Ib;
for j=2:Np, Gd(j)=(Rr+Bb'*P*Bb)\(Bb'*X); X=Ac'*X; end
comxy = zeros(n,2);
for ax=1:2
    ref = [zmpx zmpy]; r = ref(:,ax); r = [r; repmat(r(end),Np,1)];
    x = [0;0;0]; sumE=0; out=zeros(n,1); zo=zeros(n,1);
    x(1)= 0; if ax==2, x(1)=0; end
    for k=1:n
        y = C*x; e = y - r(k); sumE = sumE + e;
        u = -Ke*sumE - Kx*x - Gd'*r(k+1:k+Np);
        x = A*x + B*u; out(k)=x(1); zo(k)=y;
    end
    comxy(:,ax)=out;
end
%% Ayak yorungeleri
sx_L=0; sx_R=0; posL=zeros(n,1); posR=zeros(n,1);
for i=1:N
    ts = T0+(i-1)*Ts+opt.dsr*Ts; te = T0+i*Ts;
    if mod(i,2)==1, sw='L'; x0=sx_L; else, sw='R'; x0=sx_R; end
    x1=landX(i);
    idx = t>=ts & t<=te; s=(t(idx)-ts)/(te-ts); sm=s.*s.*s.*(10-15*s+6*s.*s);
    if sw=='L', posL(t<ts & t>=(T0+(i-1)*Ts-0)) = x0; posL(idx)=x0+(x1-x0)*sm; posL(t>te)=x1; sx_L=x1;
    else,       posR(t<ts & t>=(T0+(i-1)*Ts-0)) = x0; posR(idx)=x0+(x1-x0)*sm; posR(t>te)=x1; sx_R=x1; end
end
% (ilk blok: t<T0 icin 0)  posL/posR zaten sifir
fxL = posL; fxR = posR;
% sallanma sirasinda yukseklik
fzL = zeros(n,1); fzR = zeros(n,1);
for i=1:N
    ts = T0+(i-1)*Ts+opt.dsr*Ts; te = T0+i*Ts; idx = t>=ts & t<=te; s=(t(idx)-ts)/(te-ts);
    h = opt.stepH*0.5*(1-cos(2*pi*s)); if mod(i,2)==1, fzL(idx)=h; else, fzR(idx)=h; end
end
%% Pelvis konumu ve IK
pel = [comxy(:,1)-off(1), comxy(:,2)-off(2), zp*ones(n,1)];
names = {'L','R'}; q = struct();
Q = struct();
for si=1:2
    sd = names{si}; k = 1-2*(si==2);       % L: +1, R: -1
    ft = [ (si==1)*fxL + (si==2)*fxR, k*hipY*ones(n,1), zAnk + (si==1)*fzL + (si==2)*fzR ];
    hip = pel + [zeros(n,1), k*hipY*ones(n,1), -zHipOff*ones(n,1)];
    r = ft - hip;                    % ayak bilegi - kalca (pelvis yonelimi = dunya)
    D = min(vecnorm(r,2,2), l1+l2-1e-4);
    kap = acos((D.^2-l1^2-l2^2)/(2*l1*l2));              % diz fleksiyonu >= 0
    ax_ = -l2*sin(kap); az_ = -(l1+l2*cos(kap));          % ayak bilegi, uyluk cercevesinde
    phi = asin(r(:,2)./abs(az_));                         % kalca roll (R = Ry(th) Rx(phi))
    th = atan2(r(:,1),r(:,3)) - atan2(ax_, cos(phi).*az_);  % kalca pitch
    th = atan2(sin(th),cos(th));
    beta = atan2(-sin(th), cos(phi).*cos(th));            % duz ayak: ayak normali dikey
    rho = asin(-sin(phi).*cos(th));
    Q.([sd '_hip_pitch']) = th; Q.([sd '_hip_roll']) = phi; Q.([sd '_knee']) = kap;
    Q.([sd '_ankle_pitch']) = beta - kap; Q.([sd '_ankle_roll']) = rho;
end
G.t = t; G.q = Q; G.pelvis = pel; G.com = [comxy, zc*ones(n,1)]; G.zmp = [zmpx zmpy];
G.foot.L = [fxL, hipY*ones(n,1), fzL]; G.foot.R = [fxR, -hipY*ones(n,1), fzR];
% CoM ofsetini yuruyus pozunda (bukulu diz) yeniden hesapla ve gerekirse bir kez tekrarla
if refine
    rb = importrobot(opt.urdf); rb.DataFormat='row'; qh = homeConfiguration(rb);
    for ii=1:rb.NumBodies, nmj = rb.Bodies{ii}.Joint.Name; if isfield(Q,nmj), qh(ii)=Q.(nmj)(1); end, end
    c0 = centerOfMass(rb,qh); c0 = c0(:)';
    if norm(c0-off) > 0.003
        opt.comOffset = c0; G = h175_gait(opt); return
    end
end
G.info = struct('L',L,'vx',opt.vx,'Ts',Ts,'zh',opt.zh,'zp',zp,'zc',zc,'comOffset',off,'T0',T0,'N',N,'tEnd',t(end), ...
    'maxHipFlex',max(abs(Q.L_hip_pitch)),'maxKnee',max(Q.L_knee));
end
