function [u_o,P_o,R_o,ds_o] = getPreoptimalVariables(u,P,R,ds)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: a point position attched on outer tube 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
persistent u_per;
persistent P_per;
persistent R_per;
persistent ds_per;

cter=~(isempty(u_per)||isempty(P_per)||isempty(R_per)||isempty(ds_per));

if(nargin==0&&cter)
    u_o=u_per;
    P_o=P_per;
    R_o=R_per;
    ds_o=ds_per;
    return;
end
u_per=u;
P_per=P;
R_per=R;
ds_per=ds;

u_o=u_per;
P_o=P_per;
R_o=R_per;
ds_o=ds_per;

end
