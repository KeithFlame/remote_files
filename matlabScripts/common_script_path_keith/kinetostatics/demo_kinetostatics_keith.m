% it is a demo to test the derived kinetostatic model.
%
% Author Keith W.
% Ver. 1.0
% Date 08.02.2023

SL = [100 10 20 15 0 0 0 0.2]';
psi1 = [1 150 2 2 2 -2]';
MBP = MultiBackboneParameter_keith;
MBP = MBP.refreshLso(psi1(2));
figure;hold on;axis equal;grid on;

[Tcc,Scc]=FKcc_2segs_bending_keith(psi1,SL);
PS_2segs_keith(Scc,SL,Tcc);
q1 = Psi2Actuation_keith(psi1,SL,MBP);
[Tco, Sco]=FKco_2segs_bending_keith(q1,MBP);
PS_2segs_keith(Sco,SL,Tco);
