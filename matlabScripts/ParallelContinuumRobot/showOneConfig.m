% show one config.
%
% Author: Keith W.
% Ver. 1.0
% Date 04.29.2022

clear all;
SP = getStructurePara_keith;
init_pose = SP.init_pose;
figure;
view([90 0]);axis equal;grid on; hold on;
% title("initial pose");
trocar = SP.trocar;
plotTrocar(trocar);

arm1 = getArmPara(112,1);
arm2 = getArmPara(1112,2);
arm3 = getArmPara(111,3);
arm4 = getArmPara(2112,4);

T_needle = eye(4);T_needle(1:3,1:3)=eul2rotm([0 10/180*pi 0]);T_needle(1:3,4) = [30 -20 120];
R1=eul2rotm([10/180*pi 0 0]);
P1=R1*[-SP.needle.r;0; 0];R1=R1*eul2rotm([90/180*pi 0 0]);
R2=eul2rotm([70/180*pi 0 0]);
P2=R2*[-SP.needle.r;0; 0];R2=R2*eul2rotm([90/180*pi 0 0])*eul2rotm([0 10/180*pi 0]);
T1_0=T_needle*[R1 P1;[0 0 0 1]];
T2_0 = T_needle*[R2 P2;[0 0 0 1]];
T1=init_pose(:,:,arm1.port)\T1_0;
T2=init_pose(:,:,arm2.port)\T2_0;
psi1=invKine_keith(T1, arm1);
psi2=invKine_keith(T2, arm2);
psi1=[psi1 pi/45];
psi2=[psi2 pi/45];
psi3=[ pi/2 20 pi/12 pi/2 pi/5 -pi/2 ];

plotNeedle(T_needle);
[Tend1, ~] = forwardKinematicsandPlotSnake_keith(psi1,arm1,1);
forwardKinematicsandPlotSnake_keith(psi2,arm2,1);
forwardKinematicsandPlotSnake_keith(psi3,arm3,1);
T4_0 = Tend1(:,:,2);T4_0(1:3,1:3)=T4_0(1:3,1:3)*eul2rotm([0 0 pi/2])*eul2rotm([pi/2 0 0]);
T4=init_pose(:,:,arm4.port)\T4_0;
psi4=invKine_keith(T4, arm4);
psi4=[psi4 pi/5];
[Tend4, ~] = forwardKinematicsandPlotSnake_keith(psi4,arm4,1);

xlabel("X (mm)");
ylabel("Y (mm)");
zlabel("Z (mm)");
set(gca,'FontName','Times New Roman','FontSize',12,'FontWeight','bold');