function dX = calcDeviationByX(X1, X2)
% this is a pose conversion function to calculate the deviation from the
% pose X1 (current) to the pose X2 (target).
%
% input 1: X1 (6X1 vector)
% input 2: X2 (6X1 vector)
%
% output 1: dX (6X1 vector)
%
% Author Keith W.
% Ver. 1.0
% Date 03.21.2023

    T1 = fromX2T(X1);
    T2 = fromX2T(X2);
    dX = calcDeviationByT(T1, T2);

end

