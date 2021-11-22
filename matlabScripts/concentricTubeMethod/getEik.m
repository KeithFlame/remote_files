function Eik = getEik(R,u,ds)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: rotation matrix
% input2: curvature
% input3: posistion
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
ez=[0 0 1]';
Rez=getSkewMatrix(ez);
Eik=R*(C(u*ds) + 0.5 *Rez)*ds;
end