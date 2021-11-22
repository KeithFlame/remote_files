function Xk=getXk(R,Pi)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: a point rotation attched on outer tube 
% input2: a point position attched on inner tube
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
persistent Xk_per;
if(~isempty(Xk_per)&&nargin==0)
    Xk=Xk_per;
    return;
end

Proj=getProjection(R);
SJ=selectJp(Pi);
Xk=Proj*SJ;
Xk_per=Xk;
end
