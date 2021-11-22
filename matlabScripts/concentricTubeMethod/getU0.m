function u0=getU0
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% output1: pre-curvature
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% outer tube: trocar

u1=[0 0 0]';
%% inner tube: arm
u2_Ls=[0 0 0]';
u2_L1=[0 0 0]';
u2_Lr=[0 0 0]';
u2_L2=[0 0 0]';
u2_Lg=[0 0 0]';
u2=[u2_Ls;u2_L1;u2_Lr;u2_L2;u2_Lg];
u0=[u1;u2];
end