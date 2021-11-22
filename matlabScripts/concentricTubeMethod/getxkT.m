function xkT=getxkT(R0,P0)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: preoptimal rotation matrix in relative position
% input2: preoptimal position in relative position
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
Proj=getProjection(R0);
SJ=selectJp(P0);
xkT=Proj*SJ;
end