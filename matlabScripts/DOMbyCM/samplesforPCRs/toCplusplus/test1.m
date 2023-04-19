SL = [100 10 20 15 0.2 5 0.6 0 600]';
q1 = [0 60 1 1 1 1]';
q2 = [0 80 1 1 1 1]';
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP1 = MBP1.refreshLso(q1(2));
MBP1 = MBP1.setLdo(0.0127);
MBP1 = MBP1.resetLg( -0.0182);
MBP2 = MBP2.refreshLso(q2(2));
MBP2 = MBP2.setLdo(0.0390);
MBP2 = MBP2.resetLg(-0.0087);
MBPF = [MBP1 MBP2];

qa = [q1' q2']'/1000;
qa([1 7])=qa([1 7])*1000;
qa0 = qa;
te = 28353; tb = MBP1.Kb1(1,1);
k1 = 0.0016;
k2 = 0.0016;
k3 = 0.0012;
K1 = 5;
K2 = 0.6;
ksi = [te k1 k2 k3...
    te k1 k2 k3...
    K1 K2 K1 K2]';
KSI = ksi;
Fe = zeros(3,1);Me = zeros(3,1);fe = zeros(3,1);le = zeros(3,1);
Guess = zeros(20,1);
[guess,t0,t1,t2,t3,y0,y1,y2,y3]=shootingOpt_keith(Guess,qa,ksi);
T = eye(4); 
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
% % % % % FMfm = zeros(12,1);
% % % % % %%
% % % % % 
% % % % % [KSI,guesss] =optimizeStiffness(guess,T,qa,FMfm,MBPF,ksi*10);
% % % % % ksi_ = KSI;
% % % % % %%
% % % % % vec = [0 0 100 -0.188866 0.62642 0.421401 -0.627972]';
% % % % % Tend = fromQuat2T(vec);
% % % % % 
% % % % % [QA,guessa] =optimizeActuation(guess,Tend,qa,FMfm,MBPF,ksi_);
