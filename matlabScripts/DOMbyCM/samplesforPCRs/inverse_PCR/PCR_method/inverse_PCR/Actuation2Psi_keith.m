function psi = Actuation2Psi_keith(qa,SL,MBP)
% this is a function to get configuration variable form a known actuation
% and structural variables.
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

    qa(2:end) = qa(2:end) / 1000;
    l = qa(2);
    MBP = MBP.refreshLso(l*1000);
    u = Actuation2Curvature_keith(qa,MBP);
    psi = Curvature2Psi_keith(u,SL,l);
    psi(1) = qa(1);
    psi(2) = qa(2)*1000;
% %     psi([4 6])= psi([4 6]) + pi;

end

