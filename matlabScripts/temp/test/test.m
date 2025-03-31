SL = [100 10 20 15 0.1 5 0.6 0 600 0];
MBP = MultiBackboneParameter_keith;
MBP = MBP.resetCalibrationPara(SL);
FM=[0 0 0 ...分布力
    0 0 0 ...分布力矩
    0 0 0 ...末端力
    0 0 0]'; % 末端力矩
psi = [80 0 0 0 0 0]';

psi(2:end)=psi(2:end)/180*pi;
t = psi(1);
psi(1) =psi(2);
psi(2)=t;
Qa = Psi2Actuation_keith(psi,SL,MBP);

T = FKco_2segs_bending_keith(Qa,MBP,FM)

