% show one arm.
%
% Author: Keith W.
% Ver. 1.0
% Date 08.02.2022

clear all;
SP = getStructurePara_keith;
init_pose = SP.init_pose;
figure;
view([0 0]);axis equal;grid on; hold on;
trocar = SP.trocar;
plotTrocar(trocar);

arm1 = getArmPara(112,4);
arm2 = getArmPara(1112,2);

%% 输入参数
Fe=[0 -0. -0.]';
Me=[0 0 0]';
fe=[0 0 0]';
le=[0 0 0]';
external_1 = [Fe;Me;fe;le];
psi_1 = [60 45 10 180 100 0 1];
psi_1(2:6) = psi_1(2:6)*pi/180;

Fe=[0 -0. -0.]';
Me=[0 0 0]';
fe=[0 0 0]';
le=[0 0 0]';
external_2 = [Fe;Me;fe;le];
psi_2 = [80 45 0 0 0 0 1];
psi_2(2:6) = psi_2(2:6)*pi/180;
%%
[t1,u1,res1]=forwardStatic_END_andPlotSnake_keith(psi_1,arm1,external_1,1);
[t2,u2,res2]=forwardStaticandPlotSnake_keith(psi_2,arm2,external_2,1);
% forwardKinematicsandPlotSnake_keith(psi,arm2,1);
xlabel("X (mm)");
ylabel("Y (mm)");
zlabel("Z (mm)");
set(gca,fontsize = 14);
