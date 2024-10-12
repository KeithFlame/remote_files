clear
clc

MBP = MultiBackboneParameter_keith;
SL = [MBP.L1;MBP.Lr;MBP.L2;MBP.Lg]*1e3;
psi = [0;80;pi/8;0;-pi/6;0];
qa = Psi2Actuation_keith(psi,SL,MBP);

MBP = MBP.refreshLso(qa(2));
% [Tend, S] = FKco_2segs_bending_keith(qa, MBP);
% 正解+绘图
[Tend, S, T_tip] = FKco_IAUS_2segs_bending(qa, MBP);
PS_2segs_keith(S, SL, Tend, MBP, T_tip);
% 逆解+绘图
T_tip = [T_tip(1:3,1:3)*RotmAxisZ(0/180*pi) T_tip(1:3,4); 0 0 0 1];
[qa,S, Tend] = IKco_IAUS_2segs_bending(T_tip, MBP);
PS_2segs_keith(S, SL, Tend, MBP, T_tip, MBP.discrete_element*1e3, eye(4),1);
