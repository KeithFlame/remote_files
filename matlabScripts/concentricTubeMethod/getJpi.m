function Jpi=getJpi(ui,pi,Ri,dsi)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: curvature
% input2: posistion
% output1: this seg jacobian matrix
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%% get Dik
Dik=getDik(Ri,ui,dsi);
%% get Eik
Eik=getEik(Ri,ui,dsi);
Rpi=getSkewMatrix(pi);
Jpi=Rpi*Dik+Eik;
end

