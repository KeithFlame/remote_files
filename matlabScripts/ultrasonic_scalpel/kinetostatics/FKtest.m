clear
clc

MBP = MultiBackboneParameter_keith;
SL = [MBP.L1;MBP.Lr;MBP.L2;MBP.Lg]*1e3;
psi = [0;180;pi/4;0;-pi/3;0];
qa = Psi2Actuation_keith(psi,SL,MBP);

MBP = MBP.refreshLso(qa(2));
% [Tend, S] = FKco_2segs_bending_keith(qa, MBP);
[Tend, S, T_tip] = FKco_IAUS_2segs_bending(qa, MBP);

PS_2segs_keith(S, SL, Tend, MBP, T_tip);

P_g = Tend(1:3,4);
R_g = Tend(1:3,1:3);

P_tip = T_tip(1:3,4);
R_tip = T_tip(1:3,1:3);
errorR = R_g * MBP.g_R_tip - R_tip;

errorP = norm(P_tip - P_g);
errorR_ = (P_tip - MBP.b_P_port*1e3)/norm(P_tip - MBP.b_P_port*1e3) - R_tip(:,3);