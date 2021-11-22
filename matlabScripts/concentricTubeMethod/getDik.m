function Dik=getDik(R,u,ds)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: rotation matrix
% input2: curvature
% input3: arc length
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
Dik=R*dexp(u*ds)*ds;
end