clear
clc

%平面Cosserat rod积分测试

%界面参数
global Kse Kbt;
E = 62e9;        %杨氏模量  
G = 23.3082e9;   %截切模量 
r = 0.0012/2;     %横截面半径

%合成变量
A = pi*r^2;   %横截面面积
I = pi*r^4/4; %二次面积矩
J = 2*I;      %极惯性矩

Kse = 5*diag([G*A, E*A]);    %剪切刚度矩阵
Kbt = 5*E*I;                 %拉压刚度矩阵

%连续体支链尺寸参数
L = 0.100;
D = 0.300;

%导轨向量
vector_1 = [0.342020143325669;0;0.939692620785908];
vector_2 = [-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = [-0.171010071662835;-0.296198132726024;0.939692620785908];

%动平台参数(mm)
r = 20;
phi_1 = 0;
phi_2 = 2/3 * pi;
phi_3 = 4/3 * pi;

%初始驱动量(m)
q1 = 300/1000;
q2 = 300/1000;
q3 = 300/1000;

%目标动平台位置(m)
x_end = 150;
y_end = 0;
z_end = -225;
target = 1/1000*[x_end;y_end;z_end];

%支链1初值
P1_0 = [0;0];
R1_0 = -eye(2);
n1_0 = [0;0];
m1_0 = 0; 
n1_1 = [0;0];
m1_1 = 0; 


%支链2初值
P2_0 = [0;0];
R2_0 = -eye(2);
n2_0 = [0;0];
m2_0 = 0; 
n2_1 = [0;0];
m2_1 = 0; 


%支链3初值
P3_0 = [0;0];
R3_0 = -eye(2);
n3_0 = [0;0];
m3_0 = 0; 
n3_1 = [0;0];
m3_1 = 0; 

%初始化猜测向量
guess = [q1;q2;q3;n1_0;m1_0;n1_1;m1_1;n2_0;m2_0;n2_1;m2_1;n3_0;m3_0;n3_1;m3_1];

%积分全部支链
[ys1,ys2,ys3] = int_all_translator(guess);

%计算各支链的弯曲平面矩阵
global R1_1 R2_1 R3_1;
R1_1 = get_R1(q1,target,1);
R2_1 = get_R1(q2,target,2);
R3_1 = get_R1(q3,target,3);

%各支链坐标变换到世界坐标系下
block_size=2*(L*100+1);
points_1 = 1000 * R1_1 * [ys1(1:2,:);zeros(1,14)];
points_2 = 1000 * R2_1 * [ys2(1:2,:);zeros(1,14)];
points_3 = 1000 * R3_1 * [ys3(1:2,:);zeros(1,14)];

%绘制 delta robot
plot_delta_robot(1000*target,1000*q1,1000*q2,1000*q3,points_1,points_2,points_3);

%计算残差向量
[res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,target,ys1,ys2,ys3);
%迭代计数
k = 1;
%阻尼系数
damp = 0.00005;

% guess=fsolve(@solve_test,guess);
while error_P >= 1e-4 || error_A >= 1e-4 || error_n >= 1e-4 || error_m >= 1e-4
    tic
    %计算雅可比矩阵
    jacobian = get_jacobian_inverse(res,guess,target,ys1,ys2,ys3);
    %更新guess
    guess = guess - (jacobian'*jacobian+damp*eye(size(jacobian)))\jacobian' * res;
    q1 = guess(1);
    q2 = guess(2);
    q3 = guess(3);
    %积分全部支链
    [ys1,ys2,ys3] = int_all_translator(guess);
    %计算各支链的弯曲平面矩阵
    R1_1 = get_R1(q1,target,1);
    R2_1 = get_R1(q2,target,2);
    R3_1 = get_R1(q3,target,3);
    
    %各支链坐标变换到世界坐标系下
    points_1 = 1000 * R1_1 * [ys1(1:2,:);zeros(1,14)];
    points_2 = 1000 * R2_1 * [ys2(1:2,:);zeros(1,14)];
    points_3 = 1000 * R3_1 * [ys3(1:2,:);zeros(1,14)];
    
    %绘制 delta robot
    plot_delta_robot(1000*target,1000*q1,1000*q2,1000*q3,points_1,points_2,points_3);
    
    %计算残差向量
    [res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,target,ys1,ys2,ys3);
    %迭代计数
    k = k + 1;
    toc
end
disp([q1 q2 q3]);
point=points_3;
angle=90+atand((point(3,block_size/2+1)-point(3,block_size/2))/abs(point(1,block_size/2+1)-point(1,block_size/2)))

function [ys1,ys2,ys3] = int_all_translator(guess)
%	积分全部支链

%解包 guess

n1_0 = guess(4:5);
m1_0 = guess(6);
n1_1 = guess(7:8);
m1_1 = guess(9);

n2_0 = guess(10:11);
m2_0 = guess(12);
n2_1 = guess(13:14);
m2_1 = guess(15);

n3_0 = guess(16:17);
m3_0 = guess(18);
n3_1 = guess(19:20);
m3_1 = guess(21);

%积分
input1 = [zeros(2,1);reshape(-eye(2),4,1);n1_0;m1_0;n1_1;m1_1];
ys1 = int_one_translator(input1);

input2 = [zeros(2,1);reshape(-eye(2),4,1);n2_0;m2_0;n2_1;m2_1];
ys2 = int_one_translator(input2);

input3 = [zeros(2,1);reshape(-eye(2),4,1);n3_0;m3_0;n3_1;m3_1];
ys3 = int_one_translator(input3);

end

function ys = int_one_translator(input)
%	积分一条支链

%连续体支链尺寸参数
% global L D;
L=0.060;
D=0.380;
%积分步长
h = 0.010;

%解包输入
P0 = input(1:2,1);
R0 = reshape(input(3:6,1),2,2);
n0 = input(7:8,1);
m0 = input(9,1);
n1 = input(10:11,1);
m1 = input(12,1);
%第一段积分
y0 = [P0;reshape(R0,4,1);n0;m0];
ys0 = RK4(@cos_rod,y0,h,0,L);
%第二段积分
R1 = reshape(ys0(3:6,end),2,2);    %姿态不变
P1 = ys0(1:2,end) + R1*[D;0];    %加上多腔管的长度
y1 = [P1;reshape(R1,4,1);n1;m1];
ys1 = RK4(@cos_rod,y1,h,D+L,D+2*L);
%合并
ys = [ys0 ys1];


end

function [y] = RK4(f,y0,h,s0,s1)
%RK4 四阶龙格库塔积分
% f:cosserat Rod微分方程组
% y0:状态初值
% h:积分步长

% y:输出的状态矩阵，每一列对应一个s
% s0-s1:积分区间
s=s0:h:s1;
y=zeros(length(y0),length(s));
y(:,1)=y0;
for n = 1:length(s)-1
    %积分过程
    k1 = feval(f,s(n),y(:,n));
    k2 = feval(f,s(n)+h/2,y(:,n)+k1*h/2);
    k3 = feval(f,s(n)+h/2,y(:,n)+k2*h/2);
    k4 = feval(f,s(n)+h,y(:,n)+k3*h);
    y(:,n+1) = y(:,n) + (k1+2*k2+2*k3+k4)/6*h;
end
end

function ys = cos_rod(s,y)
    %Cosserat_Rod微分方程组
    global Kse Kbt;

    %解包
    P = y(1:2);
    R = reshape(y(3:6),2,2);
    n = y(7:8);
    m = y(9);
    %计算
    dP = R*(Kse^-1*R'*n + [1;0]);
    dm = -cross([dP;0],[n;0]);
    dm = dm(3);
    u = Kbt^-1*m + 0;
    dR = R*[0 -u;u 0];
    dn = [0;0];
    %打包
    ys = [dP;reshape(dR,4,1);dn;dm];
end

function plot_delta_robot(target,q1,q2,q3,points_1,points_2,points_3)
%	绘制delta机器人

% 导轨长度
points_can_not_reach = [0 190;0 0;0 0];
points_can_reach = [190 490;0 0;0 0];

%导轨角度参数
alpha = -70/180*pi;
beta = 120/180*pi;

%连续体长度参数
L = 100;        %连续体段
D = 300;        %多腔管

%% 计算导轨顶点

%计算支链1导轨顶点
points_can_not_reach_1 = [cos(alpha) 0 sin(alpha);0 1 0;-sin(alpha) 0 cos(alpha)] * points_can_not_reach;
points_can_reach_1 = [cos(alpha) 0 sin(alpha);0 1 0;-sin(alpha) 0 cos(alpha)] * points_can_reach;

vector_1 = points_can_reach_1(:,2);
vector_1 = vector_1/norm(vector_1);

x_n_1 = points_can_not_reach_1(1,:);
y_n_1 = points_can_not_reach_1(2,:);
z_n_1 = points_can_not_reach_1(3,:);

x_1 = points_can_reach_1(1,:);
y_1 = points_can_reach_1(2,:);
z_1 = points_can_reach_1(3,:);
%计算支链2导轨顶点
points_can_not_reach_2 = [cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1]*points_can_not_reach_1;
points_can_reach_2 = [cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1]*points_can_reach_1;

vector_2 = points_can_reach_2(:,2);
vector_2 = vector_2/norm(vector_2);

x_n_2 = points_can_not_reach_2(1,:);
y_n_2 = points_can_not_reach_2(2,:);
z_n_2 = points_can_not_reach_2(3,:);

x_2 = points_can_reach_2(1,:);
y_2 = points_can_reach_2(2,:);
z_2 = points_can_reach_2(3,:);
%计算支链3导轨顶点
points_can_not_reach_3 = [cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1]*points_can_not_reach_2;
points_can_reach_3 = [cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1]*points_can_reach_2;

vector_3 = points_can_reach_3(:,2);
vector_3 = vector_3/norm(vector_3);

x_n_3 = points_can_not_reach_3(1,:);
y_n_3 = points_can_not_reach_3(2,:);
z_n_3 = points_can_not_reach_3(3,:);

x_3 = points_can_reach_3(1,:);
y_3 = points_can_reach_3(2,:);
z_3 = points_can_reach_3(3,:);
%% 计算连续体段的顶点

base1 = q1 * vector_1;
base2 = q2 * vector_2;
base3 = q3 * vector_3;

points_1 = points_1 + repmat(base1,1,size(points_1,2));
points_2 = points_2 + repmat(base2,1,size(points_2,2));
points_3 = points_3 + repmat(base3,1,size(points_3,2));

proximal_1 = points_1(:,1:7);
distal_1 = points_1(:,8:14);

proximal_2 = points_2(:,1:7);
distal_2 = points_2(:,8:14);

proximal_3 = points_3(:,1:7);
distal_3 = points_3(:,8:14);

%% 绘制

%绘制三根导轨
plot_orientation([0;0;0],eye(3),100);
plot3(x_n_1,y_n_1,z_n_1,'r-.','linewidth',3);
plot3(x_1,y_1,z_1,'r','linewidth',3);

plot3(x_n_2,y_n_2,z_n_2,'g-.','linewidth',3);
plot3(x_2,y_2,z_2,'g','linewidth',3);

plot3(x_n_3,y_n_3,z_n_3,'b-.','linewidth',3);
plot3(x_3,y_3,z_3,'b','linewidth',3);

%绘制连续体支链1
plot3(proximal_1(1,:),proximal_1(2,:),proximal_1(3,:),'r','linewidth',1);
plot3(distal_1(1,:),distal_1(2,:),distal_1(3,:),'r','linewidth',1);
plot3([proximal_1(1,end);distal_1(1,1)],[proximal_1(2,end);distal_1(2,1)],[proximal_1(3,end);distal_1(3,1)],'k','linewidth',1);

%绘制连续体支链2
plot3(proximal_2(1,:),proximal_2(2,:),proximal_2(3,:),'g','linewidth',1);
plot3(distal_2(1,:),distal_2(2,:),distal_2(3,:),'g','linewidth',1);
plot3([proximal_2(1,end);distal_2(1,1)],[proximal_2(2,end);distal_2(2,1)],[proximal_2(3,end);distal_2(3,1)],'k','linewidth',1);

%绘制连续体支链3
plot3(proximal_3(1,:),proximal_3(2,:),proximal_3(3,:),'b','linewidth',1);
plot3(distal_3(1,:),distal_3(2,:),distal_3(3,:),'b','linewidth',1);
plot3([proximal_3(1,end);distal_3(1,1)],[proximal_3(2,end);distal_3(2,1)],[proximal_3(3,end);distal_3(3,1)],'k','linewidth',1);

%画目标位置
plot_orientation(target,eye(3),30);
circle(target,eye(3),20);

grid on;
title('delta robot');
axis equal;
axis([-500 500  -500 500  -300 500]);
drawnow;
hold off;

end

function [res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,target,ys1,ys2,ys3)
%	残差向量计算函数

% target = 1/1000 * target;
% 单位变换
% q1 = q1/1000;
% q2 = q2/1000;
% q3 = q3/1000;
%动平台参数(m)
r = 20/1000;
phi_1 = 0;
phi_2 = 2/3 * pi;
phi_3 = 4/3 * pi;

%多腔管重力（N）
G_tran = [0;0;-2.4059];

%动平台重力(N)
G_end = [0;0;-0.7448];

%负载重力（N）
Fe = [-0.0*9.81;0;-0*9.81];

%导轨向量
angle_alpha=70*pi/180;

vector_1 = [cos(angle_alpha);0;sin(angle_alpha)];
vector_2 = eul2rotm([phi_2,0,0])*vector_1;%[-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = eul2rotm([phi_3,0,0])*vector_1;
%偏移量
offset_1 = 1e-3 *[0;-1;0];
offset_2 = 1e-3 *[-cos(pi/6);-sin(pi/6);0];
offset_3 = 1e-3 *[-cos(pi/6);sin(pi/6);0];

%全局变量
global R1_1 R2_1 R3_1;

%状态序号
s1 = 7;    %穿入多腔管
s2 = 8;    %穿出多腔管
s3 = 14;    %末端
%% 解包

% 各支链穿入多腔管位置
P_1_1= [ys1(1:2,s1);0];
P_2_1 = [ys2(1:2,s1);0];
P_3_1 = [ys3(1:2,s1);0];

% 各支链穿出多腔管位置
P_1_2 = [ys1(1:2,s2);0];
P_2_2 = [ys2(1:2,s2);0];
P_3_2 = [ys3(1:2,s2);0];

% 各支链末端位置
P_1_3= [ys1(1:2,s3);0];
P_2_3 = [ys2(1:2,s3);0];
P_3_3 = [ys3(1:2,s3);0];

% 各支链末端姿态
R_1_3 = reshape(ys1(3:6,end),2,2);
R_1_3 = [R_1_3 zeros(2,1);zeros(1,2) 1];

R_2_3 = reshape(ys2(3:6,end),2,2);
R_2_3 = [R_2_3 zeros(2,1);zeros(1,2) 1];

R_3_3 = reshape(ys3(3:6,end),2,2);
R_3_3 = [R_3_3 zeros(2,1);zeros(1,2) 1];

% 各支链穿入多腔管的力
n_1_1 = [ys1(7:8,s1);0];
n_2_1 = [ys2(7:8,s1);0];
n_3_1 = [ys3(7:8,s1);0];

% 各支链穿出多腔管的力
n_1_2 = [ys1(7:8,s2);0];
n_2_2 = [ys2(7:8,s2);0];
n_3_2 = [ys3(7:8,s2);0];

% 各支链穿入末端的力
n_1_3 = [ys1(7:8,s3);0];
n_2_3 = [ys2(7:8,s3);0];
n_3_3 = [ys3(7:8,s3);0];

% 各支链穿入多腔管的力矩
m_1_1 = [0;0;ys1(9,s1)];
m_2_1 = [0;0;ys2(9,s1)];
m_3_1 = [0;0;ys3(9,s1)];

% 各支链穿出多腔管的力矩
m_1_2 = [0;0;ys1(9,s2)];
m_2_2 = [0;0;ys2(9,s2)];
m_3_2 = [0;0;ys3(9,s2)];

% 各支链穿入末端的力矩
m_1_3 = [0;0;ys1(9,end)];
m_2_3 = [0;0;ys2(9,end)];
m_3_3 = [0;0;ys3(9,end)];

% 各支链多腔管的中心位置(弯曲平面坐标系)
P_1_tran = 1/2 *(P_1_1 + P_1_2);
P_2_tran = 1/2 *(P_2_1 + P_2_2);
P_3_tran = 1/2 *(P_3_1 + P_3_2);

% 各支链多腔管的重力（弯曲平面坐标）
G_tran_1 = R1_1'* G_tran;
G_tran_2 = R2_1'* G_tran;
G_tran_3 = R3_1'* G_tran;

%% 计算残差向量
% 各支链的末端位置约束
error_P_1 = R1_1' * (q1 * vector_1 + 0*offset_1 + R1_1 * P_1_3 - target - [r*cos(phi_1);r*sin(phi_1);0]);
error_P_1 = error_P_1(1:2,1);

error_P_2 = R2_1' * (q2 * vector_2 + 0*offset_2 + R2_1 * P_2_3 - target - [r*cos(phi_2);r*sin(phi_2);0]);
error_P_2 = error_P_2(1:2,1);

error_P_3 = R3_1' * (q3 * vector_3 + 0*offset_3 + R3_1 * P_3_3 - target - [r*cos(phi_3);r*sin(phi_3);0]);
error_P_3 = error_P_3(1:2,1);

% 各支链的末端姿态约束
target_orientation = [-1 0 0;0 -1 0;0 0 1];
[~,error_A_1] = error_orientation(R_1_3,target_orientation);
error_A_1 = error_A_1(3);

[~,error_A_2] = error_orientation(R_2_3,target_orientation);
error_A_2 = error_A_2(3);

[~,error_A_3] = error_orientation(R_3_3,target_orientation);
error_A_3 = error_A_3(3);


% 各支链多腔管受力平衡
error_n_1 = -n_1_1 + n_1_2 + R1_1'* G_tran;
error_n_1 = error_n_1(1:2,1);
error_n_2 = -n_2_1 + n_2_2 + R2_1'* G_tran;
error_n_2 = error_n_2(1:2,1);
error_n_3 = -n_3_1 + n_3_2 + R3_1'* G_tran;
error_n_3 = error_n_3(1:2,1);

% 各支链多腔管力矩平衡
error_m_1 = -cross(P_1_1,n_1_1) + cross(P_1_2,n_1_2) - m_1_1 + m_1_2 + cross(P_1_tran,G_tran_1);
error_m_1 = error_m_1(3);
error_m_2 = -cross(P_2_1,n_2_1) + cross(P_2_2,n_2_2) - m_2_1 + m_2_2 + cross(P_2_tran,G_tran_2);
error_m_2 = error_m_2(3);
error_m_3 = -cross(P_3_1,n_3_1) + cross(P_3_2,n_3_2) - m_3_1 + m_3_2 + cross(P_3_tran,G_tran_3);
error_m_3 = error_m_3(3);

% 动平台受力平衡
error_n_end = -R1_1*n_1_3 - R2_1*n_2_3 - R3_1*n_3_3 + Fe + G_end;
% 动平台力矩平衡
r1 = r*[cos(0);sin(0);0];
r2 = r*[cos(2/3*pi);sin(2/3*pi);0];
r3 = r*[cos(4/3*pi);sin(4/3*pi);0];
error_m_end = -R1_1*m_1_3 - R2_1*m_2_3 - R3_1*m_3_3;
error_m_end = error_m_end - cross(r1,R1_1*n_1_3) - cross(r2,R2_1*n_2_3) - cross(r3,R3_1*n_3_3);
%组合

res = [ error_P_1;error_P_2;error_P_3;
        error_A_1;error_A_2;error_A_3;
        error_n_1;error_n_2;error_n_3;
        error_m_1;error_m_2;error_m_3;
        error_n_end];

%% 残差范数计算
error_P = max([norm(error_P_1),norm(error_P_2),norm(error_P_3)]);
error_A = max([norm(error_A_1),norm(error_A_2),norm(error_A_3)]);
error_n = max([norm(error_n_1),norm(error_n_2),norm(error_n_3),norm(error_n_end)]);
error_m = max([norm(error_m_1),norm(error_m_2),norm(error_m_3)]);
end

function [error_angle,error_axis] = error_orientation(orientation_current,orientation_target)
%   根据当前姿态和目标姿态求解姿态偏差（轴-角表示）
    error_R=orientation_target'*orientation_current;
    error_angle=acos((trace(error_R)-1)/2);
    if sin(error_angle)==0
        error_axis = [0;0;0];
    else
        error_axis=1/(2*sin(error_angle))*[error_R(3,2)-error_R(2,3);error_R(1,3)-error_R(3,1);error_R(2,1)-error_R(1,2)];
    end
    error_axis = error_angle*error_axis;
end

function jacobian = get_jacobian_inverse(error,guess,target,ys1,ys2,ys3)
%	计算雅可比矩阵
delta = 0.001;
jacobian = zeros(length(error),length(guess));
q1 = guess(1);
q2 = guess(2);
q3 = guess(3);
for i = 1:length(guess)                             % 逐列计算雅可比矩阵
    guess(i) = guess(i) + delta;
    if i >= 1 && i <= 3        %属于 驱动量
        [error_new,~,~,~,~] = delta_robo_error(guess(1),guess(2),guess(3),target,ys1,ys2,ys3);
    elseif i >= 4 && i <= 9    %属于 支链1	
        input1 = [zeros(2,1);reshape(-eye(2),4,1);guess(4:9,1)];
        ys1_new = int_one_translator(input1);      %仅积分 支链1
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,target,ys1_new,ys2,ys3);
    elseif i >= 10 && i <= 15   %属于 支链2
        input2 = [zeros(2,1);reshape(-eye(2),4,1);guess(10:15,1)];
        ys2_new=int_one_translator(input2);       %仅积分 支链2
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,target,ys1,ys2_new,ys3);
    elseif i >= 16 && i <= 21   %属于 支链3
        input3 = [zeros(2,1);reshape(-eye(2),4,1);guess(16:21,1)];
        ys3_new=int_one_translator(input3);       %仅积分 支链3
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,target,ys1,ys2,ys3_new);
    end
    jacobian(:,i) = (error_new - error)/delta;
    guess(i) = guess(i) - delta;
end
end

function  R1 = get_R1(q,target,number)
%	计算弯曲平面变换矩阵

%动平台参数(m)
r = 20/1000;
phi_1 = 0;
phi_2 = 2/3 * pi;
phi_3 = 4/3 * pi;

%导轨向量
vector_1 = [0.342020143325669;0;0.939692620785908];
vector_2 = [-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = [-0.171010071662835;-0.296198132726024;0.939692620785908];

switch number
    case 1  %支链1
        vector = vector_1;
        phi = phi_1;
        offset = 1e-3 * [0;-1;0];
    case 2  %支链2
        vector = vector_2;
        phi = phi_2;
        offset = 1e-3 * [-cos(pi/6);-sin(pi/6);0];
    case 3  %支链3
        vector = vector_3;
        phi = phi_3;
        offset = 1e-3 * [-cos(pi/6);sin(pi/6);0];
end
% 解包target
x_end = target(1);
y_end = target(2);

%计算变换矩阵
P0 = q*vector+0*offset;
delta = pi - atan2(P0(2)-y_end-r*sin(phi),P0(1)-x_end-r*cos(phi));
R1 = [0 cos(delta) sin(delta);0 -sin(delta) cos(delta);1 0 0]; 
end

function plot_orientation(position,orientation,length)
%   绘制三维箭头以表示
%   position 箭头起点
%   oriention 姿态矩阵

    
    orientation_x=length*orientation(:,1);
    orientation_y=length*orientation(:,2);
    orientation_z=length*orientation(:,3);
    plot3([position(1),position(1)+orientation_x(1)],[position(2),position(2)+orientation_x(2)],[position(3),position(3)+orientation_x(3)],'r','linewidth',1);
    hold on
    plot3([position(1),position(1)+orientation_y(1)],[position(2),position(2)+orientation_y(2)],[position(3),position(3)+orientation_y(3)],'g','linewidth',1);
    hold on
    plot3([position(1),position(1)+orientation_z(1)],[position(2),position(2)+orientation_z(2)],[position(3),position(3)+orientation_z(3)],'b','linewidth',1);
    hold on
end

function  circle(position,orientation,r)
%   在目标位置和姿态处画圆
    x=r*[1.00500000000000,0.950817241700635,0.794140509396394,0.551948158122427,0.250485487140799,-0.0775793454723323,-0.396695424652969,-0.672281571625741,-0.874473751206489,-0.981361303402722,-0.981361303402722,-0.874473751206489,-0.672281571625741,-0.396695424652970,-0.0775793454723327,0.250485487140799,0.551948158122427,0.794140509396393,0.950817241700635,1.00500000000000];
    y=r*[-0.0750000000000000,0.249699469204683,0.539212712689668,0.762166478262529,0.894400265939330,0.921584493006670,0.840773326655058,0.660723910673132,0.400947393037074,0.0895945902807340,-0.239594590280734,-0.550947393037074,-0.810723910673131,-0.990773326655057,-1.07158449300667,-1.04440026593933,-0.912166478262529,-0.689212712689668,-0.399699469204684,-0.0750000000000003];
    z(1,20)=zeros;
    for i=0:19
        Pos = position + orientation*[x(i+1);y(i+1);z(i+1)];
        x(i+1)=Pos(1);
        y(i+1)=Pos(2);
        z(i+1)=Pos(3);
    end 
    plot3(x,y,z,'b','linewidth',2);
end

