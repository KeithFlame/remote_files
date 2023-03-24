function Qk = getQk(Kk,vk)
% this function is partial of the Kalman Filter algorithm to estimate the
% Jacobian matrix of a comlex system
% 
% input 1: K (the Kalman Gain);
% input 2: v (the difference of predicted and mesurement);
%
% output: Qk (the covariance matrix of process noise);
%
% Author Keith W.
% Ver. 1.0
% Date 03.22.2023
    persistent v_num;
    block_size = 10;
    [m,~] = size(vk);
    if(isempty(v_num))
        v_num = zeros(m,block_size);
    end
    for i = block_size : -1 : 2
        v_num(:,i) = v_num(:,i-1);
    end
    v_num(:,1) = vk;
    tt = mean(abs(v_num));
    count = block_size - length(find(tt==0));
    cc = zeros(m,m);
    for i = 1:count
        cc = cc + v_num(:,i)*v_num(:,i)';
    end
    cc = cc/count;
    Qk = Kk * cc * Kk';
        

end

