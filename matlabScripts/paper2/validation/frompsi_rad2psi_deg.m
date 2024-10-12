function [psi_deg] = frompsi_rad2psi_deg(psi_rad)
%FROMPSI_RAD2PSI_DEG 此处显示有关此函数的摘要
%   此处显示详细说明
psi_deg = psi_rad*180/pi;
psi_deg(2) = psi_deg(2)*pi/180; 
end

