function beta=getBeta(u)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: curvature
% output: beta
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

%%
nu=norm(u);
beta=4*sin(nu/2)*sin(nu/2)/nu/nu;
end