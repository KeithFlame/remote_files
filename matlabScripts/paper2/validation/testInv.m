SL = [99.2 10 19.4 25 0.1 5 0.6 0 500 0]';
qa = [0 60 0.1 0.1 0.1 0.1 0 60 0.1 0.1 0.1 0.1]/1000;
Guess_fk=zeros(20,1);
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.resetLg(25e-3);
MBP1 = MBP1.setLdo(23.9871e-3);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP2 = MBP2.resetLg(25e-3);
MBP2 = MBP2.setLdo(17.9698e-3);
MBPF = [MBP1 MBP2];

ksi = [MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 2 ...
    MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 2 ...
    MBP1.K1 MBP2.K1]';


% [guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guess_fk,qa,ksi,MBPF);
% [s1,s2] = setPlotData(y0,y1,y2,y3);
% figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
% plotParallelContinuumRobot(s1,s2);
% T = eye(4);
% T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
% T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
R=eul2rotm([pi/6 0 0 ]);P=[20 0 100]'*1e-3;
target = [R P;[0 0 0 1]];
Guess_inv=zeros(24,1);
target=[
-0.0284734    0.94809  -0.316726   0.593291e-3
 -0.986927 -0.0769478  -0.141612    -25.965e-3
 -0.158632   0.308554   0.937886    112.447e-3
         0          0          0          1];
% Guess_inv(1:6)=guess_fk(1:6);
% Guess_inv(7:8)=qa(1:2)-[0 0.06]';
Guess_inv(8)=0.078;Guess_inv(20)=0.072;
% i=8;
% Guess_inv(i)=1e-1;Guess_inv(12+i)=1e-1;
% Guess_inv(9:18)=guess_fk(7:16);
% Guess_inv(19:20)=qa(7:8)-[0 0.06]';
% Guess_inv(21:24)=guess_fk(17:20);
[Guess,MBP,t0,t1,t2,t3,y0,y1,y2,y3,QA,Ee]=shootingInvOpt_keith(Guess_inv,target,ksi,MBPF);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';

[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
plotParallelContinuumRobot(s1,s2);

QA([2:6 8:12])=QA([2:6 8:12])*1000;
psi1 = frompsi_rad2psi_deg(Actuation2Psi_keith(QA(1:6),SL,MBP1));
psi2 = frompsi_rad2psi_deg(Actuation2Psi_keith(QA(7:12),SL,MBP1));
psi=[psi1 ;psi2]';