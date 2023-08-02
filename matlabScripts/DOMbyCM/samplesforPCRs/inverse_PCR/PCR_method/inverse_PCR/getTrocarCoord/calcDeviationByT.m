function dX = calcDeviationByT(T1, T2)
% this is a pose conversion function to calculate the deviation from the
% pose T1 (current) to the pose T2 (target).
%
% input 1: X1 (4X4 vector)
% input 2: X2 (4X4 vector)
%
% output 1: dX (6X1 vector)
%
% Author Keith W.
% Ver. 1.0
% Date 03.21.2023

    dX = zeros(6,1);
    dX(1:3) = T2(1:3,4) - T1(1:3,4);       
    axang = rotm2axang(T1(1:3,1:3)'*T2(1:3,1:3));    
    dX(4:6) = axang(1:3)'*axang(4)*180/pi;
end

