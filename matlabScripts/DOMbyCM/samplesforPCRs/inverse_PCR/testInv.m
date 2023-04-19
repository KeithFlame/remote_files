SL = [100 10 20 15 0.2 5 0.6 0 600]';

MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBPF = [MBP1 MBP2];

te = MBP1.Ke1(3,3); tb = MBP1.Kb1(1,1);
ksi = [te MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    te MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    MBP1.K1 MBP1.K2 MBP2.K1 MBP2.K2]';

T=[    0.1068    0.2439    0.9639   -0.0178
   -0.8082   -0.5434    0.2271   -0.0020
    0.5792   -0.8033    0.1391    0.0933
         0         0         0    1.0000];
Guess = zeros(20,1);
qa = [0 60 0 0 0 0  0 90 0 0 0 0]'/1000;
% [guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guess,qa,ksi,MBPF);
[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;hold on;
plotParallelContinuumRobot(s1,s2);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
% % % % qa1=rand(6,1);qa1(2)=qa1(2)*100;
% % % % psi1=Actuation2Psi_keith(qa1,SL,MBP1);
% % % % qa2=Psi2Actuation_keith(psi1,SL,MBP1);
% % % % psi2=Actuation2Psi_keith(qa2,SL,MBP1);
% % % % qa3=Psi2Actuation_keith(psi2,SL,MBP1);
% % % % psi3=Actuation2Psi_keith(qa3,SL,MBP1);
% % % % qa4=Psi2Actuation_keith(psi3,SL,MBP1);
% % % % psi4=Actuation2Psi_keith(qa4,SL,MBP1);
% % % % qa5=Psi2Actuation_keith(psi4,SL,MBP1);
% % % % psi5=Actuation2Psi_keith(qa5,SL,MBP1);