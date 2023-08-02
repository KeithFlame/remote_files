function T = fromX2T(X)
% this is a pose conversion function to convert a 6X1 vector to a 4X4 
% matrix, where first 3 of 6 variables mean position and left variables 
% denote orientation by the axis and angle (deg).
%
% input 1: X (6X1 vector)
%
% output 1: T (4X4 matrix)
%
% Author Keith W.
% Ver. 1.0
% Date 03.21.2023

    [m,~] = size(X);
    assert(m==6);
    angle = norm(X(4:6));
    if(angle == 0)
        axis = [0 0 1];
    else
        axis = X(4:6)'/angle;
    end
    angle = angle * pi / 180;
    T = [axang2rotm([axis angle]) X(1:3);[0 0 0 1]];

end

