function [P_tb_tbte,continuum] = continuum_position(xita,L,delta)
%   求解单段连续体的
    xita_temp=0:xita/10:xita;
    if xita==0
        P_tb_tbte=[0;0;L];
        continuum=[0;0;L];
    else
        P_tb_tbte=L/xita*[cos(delta)*(1-cos(xita));sin(delta)*(cos(xita)-1);sin(xita)];
        continuum=L/xita*[cos(delta)*(1-cos(xita_temp));sin(delta)*(cos(xita_temp)-1);sin(xita_temp)];
    end
        continuum=[zeros(3,1),continuum];
end

