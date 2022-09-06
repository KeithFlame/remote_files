% show one arm.
%
% Author: Keith W.
% Ver. 1.0
% Date 05.05.2022

clear all;
SP = getStructurePara_keith;
init_pose = SP.init_pose;
figure;
view([90 0]);axis equal;grid on; hold on;
% title("initial pose");
trocar = SP.trocar;
plotTrocar(trocar);

arm1 = getArmPara(112,1);
% psi1=[ pi/2 150 pi/2 pi/2 pi/5 -pi/2 0];
psi1=[3.0559   55.8531    0.4387    2.8648    1.1801   -0.8069    0.9698];
psi1= [40 0 0 0 0 0 1];
[Tend1, ~] = forwardKinematicsandPlotSnake_keith(psi1,arm1,1);

xlabel("X (mm)");
ylabel("Y (mm)");
zlabel("Z (mm)");
set(gca,'FontName','Times New Roman','FontSize',12,'FontWeight','bold');