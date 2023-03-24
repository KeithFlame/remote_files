function R = setR(error)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: error (6X1 error vector);
%
% output: R (the error covariance matrix);
%
% Author Keith W.
% Ver. 1.0
% Date 03.22.2023

    if(nargin == 0)
        error = [0.34 0.34 0.34 0.47 0.47 0.47]';
    end
    R = diag(error);
end

