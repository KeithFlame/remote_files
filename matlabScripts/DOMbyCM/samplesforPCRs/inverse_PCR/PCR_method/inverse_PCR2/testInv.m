SL = [100 10 20 15 0.1 5 0.6 0 600]';
qa = load('qa.log');
Guess_fk=load('guess.log');
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.resetLg(15e-3);
MBP1 = MBP1.setLdo(45e-3);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP2 = MBP2.resetLg(15e-3);
MBP2 = MBP2.setLdo(45e-3);
MBPF = [MBP1 MBP2];

ksi = [MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 2 ...
    MBP1.Ke1(3,3) MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3) 2 ...
    MBP1.K1 MBP2.K1]';


[guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guess_fk,qa,ksi,MBPF);
[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
plotParallelContinuumRobot(s1,s2);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';
target = T;
Guess_inv=zeros(24,1);
Guess_inv(1:6)=guess_fk(1:6);
Guess_inv(7:8)=qa(1:2)-[0 0.06]';
Guess_inv(9:18)=guess_fk(7:16);
Guess_inv(19:20)=qa(7:8)-[0 0.06]';
Guess_inv(21:24)=guess_fk(17:20);
[guess_inv,MBPF,~,~,~,~,y0,y1,y2,y3,QA]=shootingInvOpt2_keith(Guess_inv,target,ksi,MBPF);
% T = eye(4);
% T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
% T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';

[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
plotParallelContinuumRobot(s1,s2);
% view([0 -90])
% Z = 0.1;
% rr=norm([16 9]);
% h=Z/rr*9;
% w=Z/rr*16;
% xlim([-w w])
% ylim([-h h])
% zlim([0.5*Z 1.5*Z])
