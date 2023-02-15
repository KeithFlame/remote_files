function qa=Psi2Actuation_keith(psi,SL,MBP)
% this is a function to get actuation form a known configuration variable
% and structural variables.
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

u = Psi2Curvature_keith(psi,SL);
l = psi(2);
QA = Curvature2Actuation_keith(u,l,MBP);
qa = [psi(1:2);QA];
end