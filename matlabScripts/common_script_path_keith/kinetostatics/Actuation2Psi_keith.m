function psi = Actuation2Psi_keith(qa,SL,MBP)
% this is a function to get configuration variable form a known actuation
% and structural variables.
%
% input1: qa (6 X 1 vector, for 6 DoFs) (rad, mm)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg) (mm)
%                           or (10 X 1 vector additional zeta K1 K2 gamma1
%                           Lstem gamma3) (mm, rad, or dimensionless)
% input3: MBP multi-backbone manipulator parameter
%
% output: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2) (mm, rad)
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

    
    qa(2:end) = qa(2:end) / 1000;
    l = qa(2)*1000;
    MBP = MBP.refreshLso(l);
    u = Actuation2Curvature_keith(qa,MBP);
    psi = Curvature2Psi_keith(u,SL,l);
    psi(1) = qa(1);
    psi(2) = qa(2)*1000;
% %     psi([4 6])= psi([4 6]) + pi;

end

