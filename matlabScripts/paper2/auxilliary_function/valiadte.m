psi1 = [100/180*pi 30 20*pi/180 -35*pi/180 20*pi/180 145*pi/180];
SL=[99.2 10 19.4 25 0.1];
[T1,S] = FKcc_2segs_nobending_keith(psi1,SL);
figure; hold on;axis equal;
PS_2segs_keith(S,SL,T1);

T20=[0.605443     0.79579   0.0125326       0.091
  0.0152283  0.00416079   -0.999875      96.565
  -0.795743    0.605559 -0.00959942      81.745
          0           0           0           1];
psi2 = [100/180*pi 30 20*pi/180 35*pi/180 20*pi/180 -145*pi/180];
[T2,S] = FKcc_2segs_bending_keith(psi2,SL);
PS_2segs_keith(S,SL,T2);

SL = [99.2 10 19.4 25 0.1 5 0.6 0 500 0]';
MBP1 = MultiBackboneParameter_keith;
MBP1 = MBP1.resetCalibrationPara(SL);
qa = Psi2Actuation_keith(psi1,SL,MBP1);


%% if(1)

if(1)
    psi1 = [0/180*pi 30 0*pi/180 -35*pi/180 0*pi/180 145*pi/180];
    SL=[99.2 10 19.4 12 0.1];
    [T1,~] = FKcc_2segs_nobending_keith(psi1,SL);
    % figure; hold on;axis equal;
    % PS_2segs_keith(S,SL,T1);
    psi2 = [0/180*pi 30 0*pi/180 35*pi/180 0*pi/180 -145*pi/180];
    [T2,~] = FKcc_2segs_bending_keith(psi2,SL);
    % PS_2segs_keith(S,SL,T2);
    T10=[
   -0.9983   -0.0587    0.0042   -8.3722
   -0.0071    0.0484   -0.9988   93.5008
    0.0585   -0.9971   -0.0487   94.6479
         0         0         0    1.0000];
    T20=[   -0.9956    0.0924   -0.0182    2.2524
    0.0246    0.0687   -0.9973   94.1411
   -0.0909   -0.9934   -0.0706   95.5365
         0         0         0    1.0000];
    T1s=T10*T1;
    T2s=T20*T2;
end