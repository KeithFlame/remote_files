function P = getErrorCovarianceMatrix(K, P_, H)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: K (Kalman Gain);
% input 2: P_ (the predicted error covariance matrix);
% input 3: H (the measurement matrix);
%
% output: P (the error covariance matrix);
%
% Author Keith W.
% Ver. 1.0
% Date 03.17.2023

tem = K*H;
[m,n] = size(tem);
I = eye(m,n);

P = (I-tem)*P_;

end

