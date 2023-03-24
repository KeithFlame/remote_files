function K = getKalmanGain(P_, H)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: P_ (the predicted error covariance matrix);
% input 2: H (the measurement matrix);
%
% output: K (Kalman Gain);
%
% Author Keith W.
% Ver. 1.0
% Date 03.17.2023

    R = setR;
    K = P_*H'/(H*P_*H' + R);

end

