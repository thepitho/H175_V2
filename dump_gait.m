% Yuruyus referansini MuJoCo icin CSV'ye yazar. Kullanim: dump_gait(vx,Ts,dsr,zh,stepH,nSteps)
function dump_gait(vx,Ts,dsr,zh,stepH,nSteps,fname)
if nargin<1, vx=0.12; Ts=1.0; dsr=0.4; zh=0.84; stepH=0.04; nSteps=8; end
if nargin<7, fname='gait_ref.csv'; end
root=fileparts(mfilename('fullpath')); cd(root);
G=h175_gait(struct('vx',vx,'Ts',Ts,'dsr',dsr,'zh',zh,'stepH',stepH,'nSteps',nSteps));
nm={'hip_pitch','hip_roll','knee','ankle_pitch','ankle_roll'}; cols={}; hdr={'t','px','py','pz','zmpx','zmpy','fLz','fRz','fLx','fRx'};
T=[G.t G.pelvis G.zmp G.foot.L(:,3) G.foot.R(:,3) G.foot.L(:,1) G.foot.R(:,1)];
for s='LR', for k=1:5, f=[s '_' nm{k}]; T=[T G.q.(f)]; hdr{end+1}=f; end, end
writetable(array2table(T,'VariableNames',hdr),fullfile(root,'mujoco',fname));
fprintf('yazildi: %d satir, tEnd=%.1f\n',size(T,1),G.t(end));
end
