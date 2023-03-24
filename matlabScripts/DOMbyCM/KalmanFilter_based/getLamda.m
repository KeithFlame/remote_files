function Lamda = getLamda(H, Q, P, v)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: H (the measurement matrix);
% input 2: Q (the covariance matrix of process noise);
% input 3: P (the error covariance matrix);
% input 4: v (the difference of predicted and mesurement);
%
% output: Lamda (a coefficient);
%
% Author Keith W.
% Ver. 1.0
% Date 03.17.2023
    beta = 1;

    R = setR;
    Vk = getVk(v);
    Nk = Vk - beta * R -H * Q * H';
    Mk = H * P * H';

    ck = trace(Nk)/trace(Mk);
    
    if(ck>1)
        Lamda = ck;
    else
        Lamda = 1;
    end

end

function Vk = getVk(v)
    persistent vk;
    if(isempty(vk))
        vk = v*v';
        Vk = vk;
        return;
    end
    Vk = (0.95*vk +v*v')/1.95;
    vk = Vk;
end
