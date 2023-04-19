SL = [100 10 20 15 0.2 5 0.6 0 600]';
q1 = [0 60 1 1 1 1]';
q2 = [0 80 1 1 1 1]';
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP1 = MBP1.refreshLso(q1(2));
% MBP1 = MBP1.setLdo(0.0127);
% MBP1 = MBP1.resetLg( -0.0182);
MBP2 = MBP2.refreshLso(q2(2));
% MBP2 = MBP2.setLdo(0.0390);
% MBP2 = MBP2.resetLg(-0.0087);
MBPF = [MBP1 MBP2];
% % % figure;
% % % FM = [0 0 0 0 0 0 0 0 0 0 0 0]';
% % % psi1 = Actuation2Psi_keith(q1,SL,MBP1);
% % % q11 = Psi2Actuation_keith(psi1',SL,MBP1);
% % % psi = Actuation2Psi_keith(q11,SL,MBP1);
% % % [T,S] = FKcc_2segs_bending_keith(psi1,SL);
% % % PS_2segs_keith(S,SL,T);
% % % [T,S] = FKco_2segs_bending_keith(q1,MBP1);
% % % PS_2segs_keith(S,SL,T);

qa = [q1' q2']'/1000;
qa([1 7])=qa([1 7])*1000;
qa0 = qa;
te = MBP1.Ke1(3,3); tb = MBP1.Kb1(1,1);
ksi = [te MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    te MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    MBP1.K1 MBP1.K2 MBP2.K1 MBP2.K2]';
KSI = ksi;
Fe = zeros(3,1);Me = zeros(3,1);fe = zeros(3,1);le = zeros(3,1);
Guess = zeros(20,1);

[guess,t0,t1,t2,t3,y0,y1,y2,y3]=shootingOpt_keith(Guess,qa,Fe,Me,fe,le,MBPF,ksi);
[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;hold on;
plotParallelContinuumRobot(s1,s2);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
% plotCoord_keith(T);
FMfm = zeros(12,1);
%%
% FMfm = zeros(12,1);
% vec=[-15.4479  -2.52908  98.6631  -0.251989  0.558965  0.341466  -0.712362]';
% Ts = fromQuat2T(vec);

% qas=([    0.0433
%    60.0574
%    -1.2647
%     0.3106
%    -1.6245
%    -1.1647
%     0.0258
%    83.8541
%    -2.2001
%     0.6740
%     2.3864
%     2.4707]/1000-qa)/5+qa;
% ksi_=[28348.9873441025 0.00100000000000000 0.168080587726189 0.00100000000000000 28417.3478198196 0.00557844365882616 0.00273885961706730 2.32385144417923 0.00100000000000000 0.00100000000000000 0.00100400415820080 0.0380633128218268];

[KSI,guesss] =optimizeStiffness(guess,T,qa,FMfm,MBPF,ksi*10);
ksi_ = KSI;
%%
vec = [0 0 100 -0.188866 0.62642 0.421401 -0.627972]';
% qa1 =[    0.0017    0.0422   -0.0022   -0.0032    0.0010    0.0048...
%     0.0012    0.0832    0.0035    0.0013    0.0019    0.0018]';
Tend = fromQuat2T(vec);

% qa=[   -0.2364
%    66.4257
%    -0.0495
%    -3.3085
%    -0.8612
%     0.1464
%     0.1903
%    80.6172
%    -4.2833
%     6.3251
%    -0.8710
%     2.6405]/1000;
guess=[  -28.6053
  150.9996
 -127.2452
    4.3394
   -0.6444
    7.8764
    0.0561
    0.0210
    0.0532
   -0.0474
   30.7475
 -149.9719
  127.9832
    6.8965
   -7.3234
   -1.1670
    0.0550
   -0.1051
   -0.0154
    0.0326]/100;
ksi_=[28344.7346135954 0.00100000000000000 0.546564766142401 0.740264111124088 22547.1089128026 0.629543507761286 0.00100000000000000 17.4086744798648 0.00100000000000000 0.00100000000000000 0.00138535368525800 4.49949723421871]';

[QA,guessa] =optimizeActuation(guess,Tend,qa,FMfm,MBPF,ksi_);
[guess,t0,t1,t2,t3,y0,y1,y2,y3]=shootingOpt_keith(guessa,QA,Fe,Me,fe,le,MBPF,ksi_);
dQa = (qa-qa0)/5;
qac = qa0+dQa;

qac([2:6 8:12])=qac([2:6 8:12])*1000;
% MBP1 = MBP1.resetK12(ksi_(9:10));
% MBP2 = MBP2.resetK12(ksi_(11:12));
psi1 = Actuation2Psi_keith(qac(1:6),SL,MBP1);
psi2 = Actuation2Psi_keith(qac(7:12),SL,MBP2);
psi1([1 3:6])=psi1([1 3:6])*180/pi;
psi2([1 3:6])=psi2([1 3:6])*180/pi;
psi = [psi1;psi2],