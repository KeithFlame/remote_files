clc;clear
trolleyInfo;

O_RCM21=[0 -232.5 100]';%RCM点在世界坐标系下的坐标
O_RCM22=[0 -232.5+50 100]';%RCM点在世界坐标系下的坐标
O_RCM23=[0 -232.5+100 100]';%RCM点在世界坐标系下的坐标

O_RCM=O_RCM23;


Fy=0/180*pi;Pz=0/180*pi;Jj=-0.001;
Pj_XY1=Jj*cos(Fy);
RCM_TT=[-Pj_XY1*cos(Pz) -Pj_XY1*sin(Pz) Jj*sin(Fy)]';  

PO1=[58 -572.5+50 -85]';%方案1
PO2=PO1+arm1.d1*[0 0 1]';

%求机械臂末端姿态
Dirction_RT=RCM_TT/norm(RCM_TT);%工具的轴线方向
L=arm1.d8+Tool.H1+Tool.H2+Tool.H3;
RCM_w=RCM_TT-L*Dirction_RT;%w点(腕部)在RCM坐标系下的坐标(坐标系与世界坐标系对齐)
PO7=RCM_w+O_RCM;
Z_W=[0 0 1]';
O7R=O_RCM-PO7;
Z_T1=Dirction_RT;%工具Z轴方向(忽略工具Z方向转动)
n_TRZ=cross(Z_W,Z_T1);
Y_T1=n_TRZ/norm(n_TRZ);%工具X轴方向(忽略工具Z方向转动)
X_T1=cross(Y_T1,Z_T1);%工具Y轴方向(忽略工具Z方向转动)
ROT=[X_T1 Y_T1 Z_T1];%旋转矩阵



Rd7=50/180*pi;
Cof=[-1 1];
[Q,status1]=Clc_7DOF_Offset_Type2(arm1,O_RCM,RCM_TT,PO2,Rd7,Tool,Cof);
Tbase=[Expm([pi 0 0]') PO1;0 0 0 1];
figure;cla;grid on;hold on;axis equal
view(105,35)
PlotAxis(100,Tbase);

[Arm_7DOF_Offset,Tbase7]=Plot_Arm7_DOF_Offset(arm1,Tool,Tbase,Q);
%grid on
PlotAxis(100,[Expm([0 0 0]') [0 0 0]';0 0 0 1]);
PlotAxis(100,[Expm([0 0 0]') O_RCM21;0 0 0 1]);
PlotAxis(100,[Expm([0 0 0]') O_RCM22;0 0 0 1]);
PlotAxis(100,[Expm([0 0 0]') O_RCM23;0 0 0 1]);
PlotAxis(200,Tbase7);


[~,~,~,~,~,~,~,~,Tbase1,Tbase2,Tbase3,Tbase4,Tbase5,Tbase6,Tbase7,Tbase8]=FK_7DOF_Offset(arm1,Tbase,Q);

Tbase7


