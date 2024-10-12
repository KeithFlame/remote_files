function T = getTransformationFromVec7(vec)
% as function name
% 
% Author: Keith W.
% Ver.: 1.0
% Date: 18.02.2024

    p = vec(1:3)';
    R = quat2rotm(vec(4:7));
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = p;
end

