function dexpu=dexp(u)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: curvature
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% 
nu=norm(u);
Ru=getSkewMatrix(u);
beta=getBeta(u);
alpha=getAlpha(u);
dexpu=eye(3) + 0.5 * beta * Ru + (1-alpha)/nu/nu * Ru * Ru;

end