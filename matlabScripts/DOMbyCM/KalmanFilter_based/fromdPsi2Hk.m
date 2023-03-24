function Hk = fromdPsi2Hk(dpsi)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: dPsi (little actuation);
%
% output: Hk (the measurement matrix);
%
% Author Keith W.
% Ver. 1.0
% Date 03.21.2023

    Hk = zeros(6,36);
    for i = 1:6
        Hk(i,((i*6-5):(i*6)))=dpsi';
    end

end

