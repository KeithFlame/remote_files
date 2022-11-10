clear
clc
flag=1;
%平面Cosserat rod积分测试

%界面参数
SPC = getStructureParaforCDR_keith;

Kse = SPC.Kse;    %剪切刚度矩阵
Kbt = SPC.Kbt;    %拉压刚度矩阵

%连续体支链尺寸参数
L = SPC.size_para(4);
D = SPC.size_para(5);

%导轨向量
alpha = SPC.size_para(1);
beta = SPC.size_para(2);
vector_1 = [cos(alpha);0;sin(alpha)];
vector_2 = eul2rotm([beta 0 0])*vector_1;
vector_3 = eul2rotm([2*beta 0 0])*vector_1;

%动平台参数(mm)
phi = SPC.size_para(3);
r = SPC.size_para(6);
phi_1 = phi * 0;
phi_2 = phi * 1;
phi_3 = phi * 2;

%初始驱动量(m)
q1 = 300/1000;
q2 = 300/1000;
q3 = 301/1000;

%初始动平台位置(m)
x_end = 0;
y_end = 0;
z_end = 0e-3;
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

guess = [target;n1_0;m1_0;n2_0;m2_0;n3_0;m3_0];
% guess=[0.0000    0.0000   -0.3183   -2.6542    0.3862   -0.0625   -2.6542    0.3862   -0.0625   -2.6542    0.3862   -0.0625]';
% load('matlab.mat');
%积分全部支链
[ys1,ys2,ys3] = int_all_translator(guess);

%计算各支链的弯曲平面矩阵
% global R1_1 R2_1 R3_1;
R1_1 = get_R1(q1,guess(1:3,1),1);
R2_1 = get_R1(q2,guess(1:3,1),2);
R3_1 = get_R1(q3,guess(1:3,1),3);

%各支链坐标变换到世界坐标系下
points_1 = 1000 * R1_1 * [ys1(1:2,:);zeros(1,14)];
points_2 = 1000 * R2_1 * [ys2(1:2,:);zeros(1,14)];
points_3 = 1000 * R3_1 * [ys3(1:2,:);zeros(1,14)];

%绘制 delta robot
plot_delta_robot(flag, 1000*guess(1:3,1),1000*q1,1000*q2,1000*q3,points_1,points_2,points_3);

%计算残差向量
[res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1,ys2,ys3,SPC, R1_1, R2_1, R3_1);
%迭代计数

%阻尼系数
damp = 0.000005;
% v = 4;
% guess=fsolve(@solve_test,guess);
guess_set=guess';
res_set=res';
normres_set=0;
Q=getQ_keith;
tic
for mmi =1:10
    k=1;
    qq=Q(:,mmi)'/1000;
    q1=qq(1);
    q2=qq(2);
    q3=qq(3);
    [ys1,ys2,ys3] = int_all_translator(guess);
    R1_1 = get_R1(q1,guess(1:3,1),1);
    R2_1 = get_R1(q2,guess(1:3,1),2);
    R3_1 = get_R1(q3,guess(1:3,1),3);
    [res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1,ys2,ys3,SPC,R1_1, R2_1, R3_1);
    while error_P >= 1e-3 || error_A >= 5e-3 || error_n >= 5e-3 || error_m >= 1e-4
        
        %计算雅可比矩阵
        jacobian = get_jacobian_forward(res,guess,q1,q2,q3,ys1,ys2,ys3,SPC, R1_1, R2_1, R3_1);
        %更新guess
        if(k<10)
            step_keith=1;
        elseif(k<20)
            step_keith=0.3;
        else
            step_keith=0.1;
        end
        t=(jacobian'*jacobian+damp*eye(size(jacobian)))\jacobian' * res;
        guess = guess - step_keith* t(1:size(guess,1));
        guess = guessLimit(guess);
        guess_set=[guess_set;guess'];
        %积分全部支链
        
        [ys1,ys2,ys3] = int_all_translator(guess);
        %计算各支链的弯曲平面矩阵
        R1_1 = get_R1(q1,guess(1:3,1),1);
        R2_1 = get_R1(q2,guess(1:3,1),2);
        R3_1 = get_R1(q3,guess(1:3,1),3);
        %计算残差向量
        [res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1,ys2,ys3,SPC, R1_1, R2_1, R3_1);
        res_set=[res_set;res'];
        %各支链坐标变换到世界坐标系下
        points_1 = 1000 * R1_1 * [ys1(1:2,:);zeros(1,14)];
        points_2 = 1000 * R2_1 * [ys2(1:2,:);zeros(1,14)];
        points_3 = 1000 * R3_1 * [ys3(1:2,:);zeros(1,14)];
        
        %绘制 delta robot
    %     plot_delta_robot(flag, 1000*guess(1:3,1),1000*q1,1000*q2,1000*q3,points_1,points_2,points_3);
        
        %计算残差向量
        
        %迭代计数
        k = k + 1;
    %     disp(norm(res));
        
        normres_set=[normres_set; norm(res)];
    end
end
toc
disp(1000*guess(1:3,1));

function [ys1,ys2,ys3] = int_all_translator(guess)
%	积分全部支链

%解包 guess

n1_0 = guess(4:5);
m1_0 = guess(6);

n2_0 = guess(7:8);
m2_0 = guess(9);

n3_0 = guess(10:11);
m3_0 = guess(12);

%积分
input1 = [zeros(2,1);reshape(-eye(2),4,1);n1_0;m1_0];
ys1 = int_one_translator(input1);

input2 = [zeros(2,1);reshape(-eye(2),4,1);n2_0;m2_0];
ys2 = int_one_translator(input2);

input3 = [zeros(2,1);reshape(-eye(2),4,1);n3_0;m3_0];
ys3 = int_one_translator(input3);

end

function ys = int_one_translator(input)
%	积分一条支链

%连续体支链尺寸参数

%界面参数
SPC = getStructureParaforCDR_keith;

%连续体支链尺寸参数
L = SPC.size_para(4);
D = SPC.size_para(5);

%多腔管重力
gravity_para=SPC.gravity_para;
G_m = [-gravity_para(1); 0];
%积分步长
h = 0.010;

%解包输入
P0 = input(1:2,1);
R0 = reshape(input(3:6,1),2,2);
n0 = input(7:8,1);
m0 = input(9,1);

%第一段积分
y0 = [P0;reshape(R0,4,1);n0;m0];
ys0 = RK4(@cos_rod,y0,h,0,L);

%第二段积分
R1 = reshape(ys0(3:6,end),2,2);    %姿态不变
P1 = ys0(1:2,end) + R1*[D;0];    %加上多腔管的长度

n1=(-G_m + ys0(7:8 ,end));
m1=(-cross_2D_keith((ys0(1:2,end) + P1)/2, G_m) + cross_2D_keith(ys0(1:2,end),ys0(7:8 ,end))...
   -cross_2D_keith(P1,n1) + ys0(end,end));

y1 = [P1;reshape(R1,4,1);n1;m1];
ys1 = RK4(@cos_rod,y1,h,D+L,D+2*L);
%合并
ys = [ys0 ys1];


end

function c= cross_2D_keith(x,y)
    c=x(1)*y(2)-x(2)*y(1);
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

function ys = cos_rod(~,y)
    %Cosserat_Rod微分方程组
    %界面参数
    SPC = getStructureParaforCDR_keith;
    
    Kse = SPC.Kse;    %剪切刚度矩阵
    Kbt = SPC.Kbt;    %拉压刚度矩阵


    %解包
%     P = y(1:2);
    R = reshape(y(3:6),2,2);
    n = y(7:8);
    m = y(9);
    %计算
    dP = R*(Kse^-1*R'*n + [1;0]);
    dm = -cross_2D_keith(dP,n);
%     dm = dm(3);
    u = Kbt^-1*m + 0;
    dR = R*[0 -u;u 0];
    dn = [0;0];
    %打包
    ys = [dP;reshape(dR,4,1);dn;dm];
end

function plot_delta_robot(flag, target,q1,q2,q3,points_1,points_2,points_3)
%	绘制delta机器人
if(flag == 0)
    return;
end
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
circle_keith(target,eye(3),20);

grid on;
title('delta robot');
axis equal;
axis([-500 500  -500 500  -300 500]);
drawnow;
hold off;

end

function [res,error_P,error_A,error_n,error_m] = delta_robo_error(q1,q2,q3,target,ys1,ys2,ys3,SPC, R1_1, R2_1, R3_1)
%	残差向量计算函数

% target = 1/1000 * target;
% 单位变换
% q1 = q1/1000;
% q2 = q2/1000;
% q3 = q3/1000;
%动平台参数(m)
r = SPC.size_para(6);
phi0 = SPC.size_para(3);
phi_1 = phi0 * 0;
phi_2 = phi0 * 1;
phi_3 = phi0 * 2;

%多腔管重力（N）
G_m = [0;0;-SPC.gravity_para(1)];

%动平台重力(N)
G_P = [0;0;-SPC.gravity_para(2)];

%负载重力（N）
Fe = [0;0;-SPC.gravity_para(3)];

%导轨向量
angle_alpha=SPC.size_para(1);
vector_1 = [cos(angle_alpha);0;sin(angle_alpha)];
vector_2 = eul2rotm([phi_2,0,0])*vector_1;%[-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = eul2rotm([phi_3,0,0])*vector_1;
%偏移量
% offset_1 = 1e-3 *[0;-1;0];
% offset_2 = 1e-3 *[-cos(pi/6);-sin(pi/6);0];
% offset_3 = 1e-3 *[-cos(pi/6);sin(pi/6);0];

%全局变量

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
G_tran_1 = R1_1'* G_m;
G_tran_2 = R2_1'* G_m;
G_tran_3 = R3_1'* G_m;

%% 计算残差向量
% 各支链的末端位置约束
error_P_1 = R1_1' * (q1 * vector_1 + R1_1 * P_1_3 - target - [r*cos(phi_1);r*sin(phi_1);0]);
error_P_1 = error_P_1(1:2,1);

error_P_2 = R2_1' * (q2 * vector_2 + R2_1 * P_2_3 - target - [r*cos(phi_2);r*sin(phi_2);0]);
error_P_2 = error_P_2(1:2,1);

error_P_3 = R3_1' * (q3 * vector_3 + R3_1 * P_3_3 - target - [r*cos(phi_3);r*sin(phi_3);0]);
error_P_3 = error_P_3(1:2,1);

% 各支链的末端姿态约束
target_orientation = [-1 0 0;0 -1 0;0 0 1];
% [~,error_A_1] = error_orientation(R_1_3,target_orientation);
% error_A_1 = error_A_1(3);
% 
% [~,error_A_2] = error_orientation(R_2_3,target_orientation);
% error_A_2 = error_A_2(3);
% 
% [~,error_A_3] = error_orientation(R_3_3,target_orientation);
% error_A_3 = error_A_3(3);
[error_A_1,~] = error_orientation(R_1_3,target_orientation);
[error_A_2,~] = error_orientation(R_2_3,target_orientation);
[error_A_3,~] = error_orientation(R_3_3,target_orientation);

% 各支链多腔管受力平衡
error_n_1 = -n_1_1 + n_1_2 + R1_1'* G_m;
error_n_1 = error_n_1(1:2,1);
error_n_2 = -n_2_1 + n_2_2 + R2_1'* G_m;
error_n_2 = error_n_2(1:2,1);
error_n_3 = -n_3_1 + n_3_2 + R3_1'* G_m;
error_n_3 = error_n_3(1:2,1);

% 各支链多腔管力矩平衡
error_m_1 = -cross(P_1_1,n_1_1) + cross(P_1_2,n_1_2) - m_1_1 + m_1_2 + cross(P_1_tran,G_tran_1);
error_m_1 = error_m_1(3);
error_m_2 = -cross(P_2_1,n_2_1) + cross(P_2_2,n_2_2) - m_2_1 + m_2_2 + cross(P_2_tran,G_tran_2);
error_m_2 = error_m_2(3);
error_m_3 = -cross(P_3_1,n_3_1) + cross(P_3_2,n_3_2) - m_3_1 + m_3_2 + cross(P_3_tran,G_tran_3);
error_m_3 = error_m_3(3);

% 动平台受力平衡
error_n_end = -R1_1*n_1_3 - R2_1*n_2_3 - R3_1*n_3_3 + Fe + G_P;
% 动平台力矩平衡
r1 = r*[cos(0);sin(0);0];
r2 = r*[cos(2/3*pi);sin(2/3*pi);0];
r3 = r*[cos(4/3*pi);sin(4/3*pi);0];
error_m_end = -R1_1*m_1_3 - R2_1*m_2_3 - R3_1*m_3_3;
error_m_end = error_m_end - cross(r1,R1_1*n_1_3) - cross(r2,R2_1*n_2_3) - cross(r3,R3_1*n_3_3);
%组合

res = [ error_P_1;error_P_2;error_P_3;
        error_A_1;error_A_2;error_A_3;

        error_n_end;
%         error_m_end
        ];

%% 残差范数计算
error_P = max([norm(error_P_1),norm(error_P_2),norm(error_P_3)]);
error_A = max([norm(error_A_1),norm(error_A_2),norm(error_A_3)]);
error_n = norm(error_n_end);
error_m = max([norm(error_m_1),norm(error_m_2),norm(error_m_3)]);%,norm(error_m_end)
end

function jacobian = get_jacobian_forward(error,guess,q1,q2,q3,ys1,ys2,ys3,SPC, R1_1, R2_1, R3_1)
%	计算雅可比矩阵
delta = 0.0001;
jacobian = zeros(length(error),length(error));
for i = 1:length(guess)                             % 逐列计算雅可比矩阵
    guess(i) = guess(i) + delta;
    if i >= 1 && i <= 3        %属于 动平台
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1,ys2,ys3,SPC, R1_1, R2_1, R3_1);
    elseif i >= 4 && i <= 6    %属于 支链1	
        input1 = [zeros(2,1);reshape(-eye(2),4,1);guess(4:6,1)];
        ys1_new = int_one_translator(input1);      %仅积分 支链1
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1_new,ys2,ys3,SPC, R1_1, R2_1, R3_1);
    elseif i >= 7 && i <= 9   %属于 支链2
        input2 = [zeros(2,1);reshape(-eye(2),4,1);guess(7:9,1)];
        ys2_new=int_one_translator(input2);       %仅积分 支链2
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1,ys2_new,ys3,SPC, R1_1, R2_1, R3_1);
    elseif i >= 10 && i <= 12   %属于 支链3
        input3 = [zeros(2,1);reshape(-eye(2),4,1);guess(10:12,1)];
        ys3_new=int_one_translator(input3);       %仅积分 支链3
        [error_new,~,~,~,~] = delta_robo_error(q1,q2,q3,guess(1:3,1),ys1,ys2,ys3_new,SPC, R1_1, R2_1, R3_1);
    end
    jacobian(:,i) = (error_new - error)/delta;
    guess(i) = guess(i) - delta;
end
end

function  R1 = get_R1(q,target,number)
%	计算弯曲平面变换矩阵

%动平台参数(m)
SPC = getStructureParaforCDR_keith;
phi = SPC.size_para(3);
r = SPC.size_para(6);
phi_1 = phi * 0;
phi_2 = phi * 1;
phi_3 = phi * 2;

%导轨向量
alpha = SPC.size_para(1);
beta = SPC.size_para(2);
vector_1 = [cos(alpha);0;sin(alpha)];
vector_2 = eul2rotm([beta 0 0])*vector_1;
vector_3 = eul2rotm([2*beta 0 0])*vector_1;

switch number
    case 1  %支链1
        vector = vector_1;
        phi = phi_1;
%         offset = 1e-3 * [0;-1;0];
    case 2  %支链2
        vector = vector_2;
        phi = phi_2;
%         offset = 1e-3 * [-cos(pi/6);-sin(pi/6);0];
    case 3  %支链3
        vector = vector_3;
        phi = phi_3;
%         offset = 1e-3 * [-cos(pi/6);sin(pi/6);0];
end
% 解包target
x_end = target(1);
y_end = target(2);

%计算变换矩阵
P0 = q*vector;
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

function  circle_keith(position,orientation,r)
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

function [error_angle,error_axis] = error_orientation(orientation_current,orientation_target)
%   根据当前姿态和目标姿态求解姿态偏差（轴-角表示）
    error_R=orientation_target'*orientation_current;
    error_angle=acos((trace(error_R)-1)/2);
    if sin(error_angle)==0
        error_axis = [0;0;0];
    else
        error_axis=1/(2*sin(error_angle))*[error_R(3,2)-error_R(2,3);error_R(1,3)-error_R(3,1);error_R(2,1)-error_R(1,2)];
    end
%     error_axis = error_angle*error_axis;
    error_angle=error_angle/10;
end

function guess = guessLimit(guess0)
    guess_max=[0.406 0.406 0.126 6 16 1 6 16 1 6 16 1]';
    guess_min=[-0.406 -0.406 -0.400 -6 -16 -1 -6 -16 -1 -6 -16 -1]';
    x=find(guess0./guess_max>1);
    y=find(guess0./guess_min>1);
    guess=guess0;
    guess(x)=guess_max(x);
    guess(y)=guess_min(y);
end