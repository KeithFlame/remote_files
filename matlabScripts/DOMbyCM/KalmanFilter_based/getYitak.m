function yita = getYitak(dPsi,dZ,x)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: dPsi (the actuation increment);
% input 2: dZ (the mesurement increment);
% input 3: x (the state variable, Jacobian)
%
% output: Qk (the covariance matrix of process noise);
%
% Author Keith W.
% Ver. 1.0
% Date 03.22.2023

    alpha = 0.1;
    J_ = reshape(x,[6 6])';

    t= alpha * (dZ - J_ * dPsi) / (dPsi'*dPsi)*dPsi';
    J = t;
    yita = reshape(J',[36,1]);
end

