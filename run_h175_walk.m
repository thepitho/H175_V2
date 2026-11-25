% H175 V3 yuruyus demosu: modeli kurar, benzetir, ozet ve grafik uretir.
% Kullanim: run_h175_walk            (varsayilan: 0.12 m/s, 8 adim)
%           run_h175_walk('stay')    (yerinde adim - kararli, 10 s)
% Bilinen sinir: ileri yurumede ~5-6 adim (~0.5 m) sonra one devrilir; yerinde adim 10 s kararlidir.
function out = run_h175_walk(mode)
if nargin<1, mode='walk'; end
root = fileparts(mfilename('fullpath')); cd(root);
base = struct('nSteps',8,'Ts',1.0,'dsr',0.4,'vx',0.12,'stepH',0.04,'zh',0.84);
if strcmp(mode,'stay'), base.vx = 0; base.Ts = 0.9; base.dsr = 0.35; end
G = build_h175_walk(base,false); m = 'H175_V3_walk';
tic; out = sim(m); fprintf('benzetim %.0f s (gercek zaman %.1f s)\n',toc,G.t(end));
fb = out.simlog.FloatingBase;
[t,i] = unique(fb.Pz.p.series.time); px = fb.Px.p.series.values('m'); py = fb.Py.p.series.values('m'); pz = fb.Pz.p.series.values('m');
px = px(i); py = py(i); pz = pz(i); fell = find(pz<0.7,1);
if isempty(fell), fprintf('Robot dusmedi.\n'); else, fprintf('Robot t=%.2f s civarinda dustu.\n',t(fell)); end
tau = out.tau_log; Tpk = evalin('base','Tpk'); nm = evalin('base','jointNames');
tend = G.t(end); if ~isempty(fell), tend = t(fell); end
k = out.tout<=tend; pk = max(abs(tau(k,:)))'./Tpk;
T = table(nm, Tpk, pk, 'VariableNames',{'eklem','Tpk_Nm','tepe_oran'}); writetable(T,fullfile(root,'motor_yuk_ozeti.csv'),'Delimiter',';');
[~,o] = sort(pk,'descend'); fprintf('En yuklu motorlar (tepe/Tpk, dusme oncesi):\n'); for q=1:6, fprintf('  %-16s %.2f\n',nm{o(q)},pk(o(q))); end
f = figure('Visible','off','Position',[100 100 900 700]);
subplot(3,1,1); plot(t,px,G.t,G.pelvis(:,1),'--'); ylabel('pelvis x [m]'); legend('benzetim','plan'); grid on
subplot(3,1,2); plot(t,py,G.t,G.pelvis(:,2),'--'); ylabel('pelvis y [m]'); grid on
subplot(3,1,3); plot(t,pz); ylabel('pelvis z [m]'); xlabel('t [s]'); grid on
saveas(f,fullfile(root,'yuruyus_sonuc.png'));
end
