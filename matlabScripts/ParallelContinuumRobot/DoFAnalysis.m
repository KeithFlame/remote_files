% test and validate the Degrees of Freedom.
%
% Author: Keith W.
% Ver. 1.0
% Date 25.03.2022

%% First Construction
SP = getStructurePara_keith;
init_pose = SP.init_pose;
T_relative = [eul2rotm([0 0 pi]), [0 5 0]';[0 0 0 1]];
% initial pose
T10=[eul2rotm([0 0 -pi/2]), [0 0 90]';[0 0 0 1]];
T1=T10;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
% figure;
view([90 0]);
title("initial pose");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);

%% along X
T1=T10;T1(1,4)=20;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
figure;view([45 0]);
title("along X");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);

%% along Y
T1=T10;T1(2,4)=5;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
figure;view([90 0]);
title("along Y");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);

%% along Z
T1=T10;T1(3,4)=110;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
figure;view([90 0]);
title("along Z");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);

%% about X
T1=T10;
T_r = [eul2rotm([0 0 pi/45]), [0 0 0]';[0 0 0 1]];
T1=T1*T_r;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
figure;
view([90 0]);
title("about X");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);

%% about Y
T1=T10;
T_r = [eul2rotm([0 pi/12 0]), [0 0 0]';[0 0 0 1]];
T1=T1*T_r;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
figure;view([90 0]);
title("about Y");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);

%% about Z
T1=T10;
T_r = [eul2rotm([pi/2 0 0]), [0 0 0]';[0 0 0 1]];
T1=T1*T_r;
T2=init_pose\T1*T_relative;
psi1=invKine_keith(T1);
psi2=invKine_keith(T2);
figure;view([90 0]);
title("about Z");
forwardKinematicsandPlotSnake_keith(psi1,1,1);
forwardKinematicsandPlotSnake_keith(psi2,2,1);