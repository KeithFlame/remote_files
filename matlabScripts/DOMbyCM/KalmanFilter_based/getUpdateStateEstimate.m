function x = getUpdateStateEstimate(y,K, H, x_)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: y (output of this system)
% input 1: K (Kalman Gain);
% input 3: H (the measurement matrix);
% input 4: x_ ( the predicted state estimate);
%
% output: x (the error covariance matrix);
%
% Author Keith W.
% Ver. 1.0
% Date 03.17.2023

x = x_ + K * (y - H * x_);

end

