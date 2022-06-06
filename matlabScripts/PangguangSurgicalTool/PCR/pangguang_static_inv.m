% for pangguang parallel continuum robot
%
% author Keith W.
% Ver. 1.0
% Date 05.15.2022

R_target = eul2rotm([0 0 0]);
P_target = [0 0 40] * 1e-3;

target = [R_target P_target; [0 0 0 1]];
