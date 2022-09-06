clear
clc
% FABRIKC算法用于求解delta机器人正运动学

%% 结构参数
global alpha L_continuum L_rigid L_moving_platform;
alpha = 60/180*pi;      %导轨倾斜角
L_continuum = 60;       %连续体段长度
L_rigid = 200;          %刚性段长度
L_moving_platform = 50; %动平台长度

% 导轨单位向量
linear_guide_1 = [-cos(alpha);sin(alpha)];  
linear_guide_2 = [cos(alpha);sin(alpha)];
% 支链连接处姿态
Z_base = [0;1];

%% 构型初值
xita_1 = 0;    %支链1连续体弯曲角
xita_2 = 0;    %支链2连续体弯曲角 

%% 导轨驱动量
q_1 = 250;              %支链1导轨驱动量
q_2 = 150;              %支链2导轨驱动量

%% 计算运动链的root和target
% 以支链1的近端为root 支链2的近端为target
root = q_1 * linear_guide_1;
target = q_2 * linear_guide_2;

%% 根据构型初值计算当前位置
P_1_proximal_base = root;
P_1_proximal_joint = P_1_proximal_base + [0;-L_continuum/2];
P_1_distal_end = P_1_proximal_base + [0;-2*L_continuum-L_rigid];
P_1_distal_joint = P_1_distal_end + [0;L_continuum/2];
P_2_distal_end = P_1_distal_end + [L_moving_platform;0];
P_2_distal_joint = P_2_distal_end + [0;L_continuum/2];
P_2_proximal_base = P_2_distal_end + [0;2*L_continuum+L_rigid];
P_2_proximal_joint = P_2_proximal_base + [0;-L_continuum/2];

%% 迭代
threshold = 1e-3;
iteration_index = 0;
tic
error_s = zeros(500,1);

while norm(P_2_proximal_base - target) > threshold
    iteration_index = iteration_index + 1;
    % forward reaching phase
    P_2_proximal_base = target;
    if xita_2 == 0
        L_2 = L_continuum / 2;
    else
        L_2 = L_continuum / xita_2 * tan(xita_2/2);
    end
    P_2_proximal_joint = P_2_proximal_base - L_2 * [0;1];   
    Z_2_internal = (P_2_proximal_joint - P_2_distal_joint)/norm(P_2_proximal_joint - P_2_distal_joint);
    xita_2 = acos(Z_2_internal'*Z_base);
    if xita_2 == 0
        L_2 = L_continuum / 2;
    else
        L_2 = L_continuum / xita_2 * tan(xita_2/2);
    end
    P_2_proximal_joint = P_2_proximal_base - L_2 * [0;1];   
    P_2_proximal_end = P_2_proximal_joint - L_2 * Z_2_internal;
    P_2_distal_base = P_2_proximal_end - L_rigid * Z_2_internal;
    P_2_distal_joint = P_2_distal_base - L_2 * Z_2_internal;
    P_2_distal_end = P_2_distal_joint - L_2 * Z_base;
    
    P_1_distal_end = P_2_distal_end - [L_moving_platform;0];
    
    if xita_1 == 0
        L_1 = L_continuum / 2;
    else
        L_1 = L_continuum / xita_1 * tan(xita_1/2);
    end
    
    P_1_distal_joint = P_1_distal_end + L_1 * [0;1];
    Z_1_internal = (P_1_proximal_joint - P_1_distal_joint) / norm(P_1_proximal_joint - P_1_distal_joint);
    xita_1 = acos(Z_1_internal'*Z_base);
    
    if xita_1 == 0
        L_1 = L_continuum / 2;
    else
        L_1 = L_continuum / xita_1 * tan(xita_1/2);
    end
    P_1_distal_joint = P_1_distal_end + L_1 * Z_base;
    P_1_distal_base = P_1_distal_joint + L_1 * Z_1_internal;
    P_1_proximal_end = P_1_distal_base + L_rigid * Z_1_internal;
    P_1_proximal_joint = P_1_proximal_end + L_1 * Z_1_internal;
    P_1_proximal_base = P_1_proximal_joint + L_1 * Z_base;
    
    % backward reaching phase
    P_1_proximal_base = root;
    P_1_proximal_joint =  P_1_proximal_base - L_1 * Z_base;
    
    Z_1_internal = (P_1_proximal_joint - P_1_distal_joint) / norm(P_1_proximal_joint - P_1_distal_joint);
    xita_1 = acos(Z_1_internal'*Z_base);
    
    if xita_1 == 0
        L_1 = L_continuum / 2;
    else
        L_1 = L_continuum / xita_1 * tan(xita_1/2);
    end
    P_1_proximal_joint = P_1_proximal_base - L_1 * Z_base;
    P_1_proximal_end = P_1_proximal_joint - L_1 * Z_1_internal;
    P_1_distal_base = P_1_proximal_end - L_rigid * Z_1_internal;
    P_1_distal_joint = P_1_distal_base - L_1 * Z_1_internal;
    P_1_distal_end = P_1_distal_joint - L_1 * Z_base;
    
    P_2_distal_end = P_1_distal_end + [L_moving_platform;0];
    
    P_2_distal_joint = P_2_distal_end + L_2 * Z_base;
    Z_2_internal = (P_2_proximal_joint - P_2_distal_joint)/norm(P_2_proximal_joint - P_2_distal_joint);
    xita_2 = acos(Z_2_internal'*Z_base);
    if xita_2 == 0
        L_2 = L_continuum / 2;
    else
        L_2 = L_continuum / xita_2 * tan(xita_2/2);
    end
    P_2_distal_joint = P_2_distal_end + L_2 * Z_base;
    P_2_distal_base = P_2_distal_joint + L_2 * Z_2_internal;
    P_2_proximal_end = P_2_distal_base + L_rigid * Z_2_internal;
    P_2_proximal_joint = P_2_proximal_end + L_2 * Z_2_internal;
    P_2_proximal_base = P_2_proximal_joint + L_2 * Z_base;  
    
    error_s(iteration_index) = norm(P_2_proximal_base - target);
end
toc
%% 后续处理
% 判断delta (弯曲方向)
if Z_1_internal(1)<0
    delta_1 = 1;
else
    delta_1 = -1;
end

if Z_2_internal(1)<0
    delta_2 = 1;
else
    delta_2 = -1;
end

%% 画图
figure(1);
plot_delta_planar(q_1,xita_1,delta_1,q_2,xita_2,delta_2);
figure(2);
origin_point = [0;0];
line_1 = [origin_point 300*linear_guide_1];
line_2 = [origin_point 300*linear_guide_2];
plot(line_1(1,:),line_1(2,:),'.-','linewidth',3,'color','black');
hold on;
plot(line_2(1,:),line_1(2,:),'.-','linewidth',3,'color','black');
continuum_1_proximal = [P_1_proximal_base P_1_proximal_joint P_1_proximal_end];
rigid_1 = [P_1_proximal_end P_1_distal_base];
continuum_1_distal = [P_1_distal_base P_1_distal_joint P_1_distal_end];
continuum_2_proximal = [P_2_proximal_base P_2_proximal_joint P_2_proximal_end];
rigid_2 = [P_2_proximal_end P_2_distal_base];
continuum_2_distal = [P_2_distal_base P_2_distal_joint P_2_distal_end];

chain_1 = [continuum_1_proximal rigid_1 continuum_1_distal];
chain_2 = [continuum_2_proximal rigid_2 continuum_2_distal];
moving_platform = [chain_1(:,end) chain_2(:,end)];

plot(continuum_1_proximal(1,:),continuum_1_proximal(2,:),'linewidth',3,'color','blue');
plot(continuum_1_distal(1,:),continuum_1_distal(2,:),'linewidth',3,'color','blue')
plot(continuum_2_proximal(1,:),continuum_2_proximal(2,:),'linewidth',3,'color','blue');
plot(continuum_2_distal(1,:),continuum_2_distal(2,:),'linewidth',3,'color','blue')
plot(rigid_1(1,:),rigid_1(2,:),'linewidth',3,'color','red');
plot(rigid_2(1,:),rigid_2(2,:),'linewidth',3,'color','red');
plot(moving_platform(1,:),moving_platform(2,:),'linewidth',3,'color','black');
grid on;
box on;
axis equal;
hold off;

figure(3);
plot(error_s(1:iteration_index));






