function [Q,status]=Clc_7DOF_Offset_Type2(arm,O_RCM,RCM_TT,PO2,Rd7,Tool,Cof)

%O_RCM:RCM点在世界坐标系下的坐标
%RCM_TT:%工具末端在RCM坐标系下的坐标(坐标系与世界坐标系对齐)
%PO2:关节2在世界坐标系下的坐标
%L:工具末端到O7点(关节7)的距离
%Cof:多解判断
%status:为0说明有解，为1说明无解
status=0;
Q=[0 0 0 0 0 0 0]';
Dirction_RT=RCM_TT/norm(RCM_TT);%工具的轴线方向
L=arm.d8+Tool.H1+Tool.H2+Tool.H3;

% if RCM_TT(3)>0
% RCM_w=RCM_TT+L*Dirction_RT;%w点(腕部)在RCM坐标系下的坐标(坐标系与世界坐标系对齐)
% else
RCM_w=RCM_TT-L*Dirction_RT;%w点(腕部)在RCM坐标系下的坐标(坐标系与世界坐标系对齐)
% end
PO7=RCM_w+O_RCM;
q7=Rd7;
Z_W=[0 0 1]';
O7R=O_RCM-PO7;

Z_T1=O7R/norm(O7R);%工具Z轴方向(忽略工具Z方向转动)
n_TRZ=cross(Z_W,Z_T1);
Y_T1=n_TRZ/norm(n_TRZ);%工具X轴方向(忽略工具Z方向转动)
X_T1=cross(Y_T1,Z_T1);%工具Y轴方向(忽略工具Z方向转动)
ROT=[X_T1 Y_T1 Z_T1];%旋转矩阵

O7_PO6=Expm([0 0 q7]')*[-arm.a7 0 -norm(PO7-O_RCM)]';

PO6=ROT*O7_PO6+O_RCM;%未标注坐标系的都是在全局坐标系下的坐标

angle_O2O4O3=atan(arm.d3/arm.a4);
angle_HO4O6=atan(arm.d5/abs(arm.a5));
L_O2O4=sqrt(arm.d3^2+arm.a4^2);
L_O4O6=sqrt(arm.d5^2+arm.a5^2);
L_O2O6=norm(PO2-PO6);
angle_O2O4O6=acos((L_O2O4^2+L_O4O6^2-L_O2O6^2)/(2*L_O2O4*L_O4O6));
q4=(angle_O2O4O3+angle_HO4O6+angle_O2O4O6-2*pi);
O7_X6=[cos(q7) sin(q7) 0]';
X6=ROT*O7_X6;
Y6=-Z_T1;
Z6=cross(X6,Y6);

angle_HO6O4=pi/2-angle_HO4O6;

fig1=(L_O2O6^2+L_O4O6^2-L_O2O4^2)/(2*L_O2O6*L_O4O6);
if fig1>1
status=1.001;
return
else
angle_O2O6O4=acos((L_O2O6^2+L_O4O6^2-L_O2O4^2)/(2*L_O2O6*L_O4O6));
end

angle_O2O6H=angle_O2O6O4+angle_HO6O4;

R6=[X6 Y6 Z6];
O6_O2O6=R6'*(PO6-PO2);
fai6=atan2(O6_O2O6(2),O6_O2O6(1));


fig2=(norm(PO6-PO2)*cos(angle_O2O6H))/(sqrt(O6_O2O6(1)^2+O6_O2O6(2)^2));
if fig2>1
status=1.002;
return
else
psi6=asin((norm(PO6-PO2)*cos(angle_O2O6H))/(sqrt(O6_O2O6(1)^2+O6_O2O6(2)^2)));
end


if Cof(1)==1
q6=(psi6-fai6);
else
q6=pi-psi6-fai6; 
end
 
if q6>2*pi
  q6=q6-2*pi;
end


angle_O3O2O4=pi/2-angle_O2O4O3;
angle_O4O2O6=pi-angle_O2O6O4-angle_O2O4O6;
angle_PO2O6=angle_O3O2O4+angle_O4O2O6;
angle_O2PO6=(angle_O2O6O4+angle_O2O4O6+angle_O2O4O3-angle_O2O6H-pi/2);

O6_Z5=[sin(q6) cos(q6) 0]';

V_O2O6=PO6-PO2;
L_PO6=norm(V_O2O6)*sin(angle_PO2O6)/sin(angle_O2PO6);
RR=Expm([pi 0 0 ]')*Expm([pi 0 0 ]');%和原版有改动*Expm([pi 0 0 ]')
O2V_O2P=RR*(V_O2O6-L_PO6*R6*O6_Z5);%注意坐标系1和坐标系0之间有个180°的差距

if ~isreal(O2V_O2P(1))||~isreal(O2V_O2P(2))
status=1.003;
return   
end
if Cof(2)==1
%case1
q1=atan2(O2V_O2P(2),O2V_O2P(1));
q2=acos(O2V_O2P(3)/norm(O2V_O2P));
else
%case2
q1=atan2(-O2V_O2P(2),-O2V_O2P(1));
q2=(-acos(O2V_O2P(3)/norm(O2V_O2P)));
end
V_O2P=(V_O2O6-L_PO6*R6*O6_Z5);
Y3=cross(V_O2P,V_O2O6)/norm(cross(V_O2P,V_O2O6));
Z3=V_O2P/norm(V_O2P);
X3=cross(Y3,Z3);

RR2=[Expm([pi 0 0]') [0 0 0]';0 0 0 1];
T2=RR2*MDH(arm.Alpha1,arm.a1,q1,arm.d1)*MDH(arm.Alpha2,arm.a2,q2,arm.d2);%注意跟原始版本的区别，有个绕Y的转动
R2=T2(1:3,1:3);
O2_X3=R2'*X3;
q3=atan2(O2_X3(3),O2_X3(1));

Z5=R6*O6_Z5;


V_pH=(L_PO6-arm.d5)*Z5;

V_pO4=-V_O2P+arm.d3*Z3+arm.a4*X3;

V_HO4=V_pO4-V_pH;


T6=MDH(arm.Alpha6,arm.a6,q6,arm.d6);
O5_V_O5S=T6(1:3,1:3)*R6'*V_HO4;
q5=-atan2(O5_V_O5S(2),O5_V_O5S(1));


rage_q2=[-140 140]*pi/180;
rage_q2_2=[-5 5]*pi/180;
rage_q4=[-170 0]*pi/180;
%rage_q6=[0 180]*pi/180;
rage_q6=[0 200]*pi/180;



%防止工具与P02干涉
rage_D1=120;
xx1=O_RCM;
xx2=PO6;
pt=PO2;
D1=norm(xx2-pt);


%防止连杆PO4 PO6与P02干涉



%防止肘关节在肩关节之下
PO4=-arm.d5*Z5+PO6;
D3=PO4(3)-PO2(3);

% D4=PO4(2);
% 
% elow_Y=-307.5;
%5防止PO4与RCM PO7
rage_D5=200;
D5=DistanceBetweenPointAndLine(PO4,PO7,O_RCM);

%
rage_D6=0;
D6=TwoLineDistance(PO4,PO2,PO7,O_RCM);



if q2<rage_q2(1)||q2>rage_q2(2)||q2>rage_q2_2(1)&&q2<rage_q2_2(2)
status=1.0042;
return
end

if q4<rage_q4(1)||q4>rage_q4(2)
status=1.0044;
return
end

if q6<rage_q6(1)||q6>rage_q6(2)
status=1.0046;
return
end


if D1<rage_D1||D3<0
status=1.0051;
return
end

if D5<rage_D5&&q6<pi/2
status=1.0052;
return
end

if D6<rage_D6
status=1.0053;
return
end

Q=[q1 q2 q3 q4 q5 q6 q7]';
end




