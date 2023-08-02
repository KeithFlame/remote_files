SL = [100 10 20 15 0.1 5 0.6 0 600]';
qa=[ 0 60 0 0 0 0 0 90 0 0 0 0]'/1000;
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
MBP1 = MBP1.resetLg(21.5657e-3);
MBP1 = MBP1.setLdo(22.5716e-3);
MBP1 = MBP1.refreshLso(qa(2)*1000);
MBP2 = MultiBackboneParameter_keith;
MBP2 = MBP2.resetCalibrationPara(SL);
MBP2 = MBP2.refreshLso(qa(8)*1000);
MBP2 = MBP2.resetLg(27.4933e-3);
MBP2 = MBP2.setLdo(25.1837e-3);
MBPF = [MBP1 MBP2];

te = MBP1.Ke1(3,3); tb = MBP1.Kb1(1,1);
ksi = [te MBP1.Kb2(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    te MBP1.Kb1(1,1) MBP1.Kb1(2,2) MBP1.Kb1(3,3)...
    MBP1.K1 MBP1.K2 MBP2.K1 MBP2.K2]';

Guesss=zeros(20,1);
[guess_fk,MBPF,~,~,~,~,y0,y1,y2,y3]=shootingFkOpt_keith(Guesss,qa,ksi,MBPF);
T = eye(4);
T(1:3,1)=y3(end,4:6)';T(1:3,2)=y3(end,7:9)';
T(1:3,3)=y3(end,10:12)';T(1:3,4)=y3(end,1:3)';

[s1,s2] = setPlotData(y0,y1,y2,y3);
figure;axis equal;grid on;xlabel('X');ylabel('Y');zlabel('Z');
plotParallelContinuumRobot(s1,s2);
view([0 -90])
Z = 0.1;
rr=norm([16 9]);
h=Z/rr*9;
w=Z/rr*16;
xlim([-w w])
ylim([-h h])
zlim([0.5*Z 1.5*Z])
