function X = fromT2X(T)
% this is a pose conversion function to convert a 4X4 matrix to a 6X1 
% vector, where first 3 of 6 variables mean position and left variables 
% denote orientation by the axis and angle (deg).
%
% input 1: T (4X4 matrix)
%
% output 1: X (6X1 vector)
%
% Author Keith W.
% Ver. 1.0
% Date 03.21.2023

X = zeros(6,1);
[m,n] = size(T);
assert(m==4&&n==4);
X(1:3) = T(1:3,4);
axang = rotm2axang(T(1:3,1:3));
X(4:6) = axang(1:3)'*axang(4)*180/pi;

end

