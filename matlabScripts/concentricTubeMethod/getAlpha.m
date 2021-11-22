function alpha=getAlpha(u)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: curvature
% output1: alpha
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%%
nu=norm(u);
alpha=2*sin(nu/2)*sin(nu/2)/nu;
end