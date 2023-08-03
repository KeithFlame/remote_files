% it is a demo to test the derived kinetostatic model.
%
% Author Keith W.
% Ver. 1.0
% Date 08.02.2023

%% base parameter
% name:  L1 Lr L2 Lg zeta K1  K2   gamma1 Lstem gamma3 (rad, mm, or dimensionless)
SL = [100 10 20 15 0.2]';
% psi:  phi L theta1 delta1 theta2 delta2 (rad, mm)
psi1 = [1 150 2 2 -2 -2]';
% psi:  phi L q11 q12 q21 q22 (rad, mm)
q2 = [1 150 4 4 -4 4]';
MBP = MultiBackboneParameter_keith;

%% execution
MBP = MBP.refreshLso(psi1(2));
[Tcc1,Scc1]=FKcc_2segs_bending_keith(psi1,SL);
q1 = Psi2Actuation_keith(psi1,SL,MBP);
[Tco1, Sco1]=FKco_2segs_bending_keith(q1,MBP);

MBP = MBP.refreshLso(q2(2));
[Tco2, Sco2]=FKco_2segs_bending_keith(q2,MBP);
psi2 = Actuation2Psi_keith(q2,SL,MBP);
[Tcc2,Scc2]=FKcc_2segs_bending_keith(psi2,SL);
%% plot
figure(1);hold on;axis equal;grid on;
PS_2segs_keith(Scc1,SL,Tcc1);
PS_2segs_keith(Sco1,SL,Tco1);
PS_2segs_keith(Scc2,SL,Tcc2);
PS_2segs_keith(Sco2,SL,Tco2);