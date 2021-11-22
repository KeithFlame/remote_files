function P=getProjection(R)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: R means the rotation matrix on position s of tube. 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
ez=[0 0 1]';
P=eye(3)-R*ez*((R*ez)');
end
