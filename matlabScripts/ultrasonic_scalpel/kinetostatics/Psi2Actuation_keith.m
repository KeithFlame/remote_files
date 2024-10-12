function qa=Psi2Actuation_keith(psi,SL,MBP)
% this is a function to get actuation form a known configuration variable
% and structural variables.
%
% input1: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2) (mm, rad)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg) (mm)
%                           or (10 X 1 vector additional zeta K1 K2 gamma1
%                           Lstem gamma3) (mm, rad, or dimensionless)
% input3: MBP multi-backbone manipulator parameter
%
% output: qa (6 X 1 vector, for 6 DoFs) (rad, mm)
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

u = Psi2Curvature_keith(psi,SL);
MBP = MBP.refreshLso(psi(2));
QA = Curvature2Actuation_keith(u,MBP);
qa = [psi(1);psi(2);QA*1000];
end