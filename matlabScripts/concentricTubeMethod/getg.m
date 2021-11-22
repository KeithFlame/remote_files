function g=getg(U,U0)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.22.2021
% Ver. 1.0
% input1: U0 guess 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
persistent g_per;
if(~isempty(g_per)&&nargin==0)
    g=g_per;
    return;
end
    K=getK;
   
    g=K(U-U0);
    g_per=g;
end