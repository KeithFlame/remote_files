function [running_time,iteration_index] = FABRIKc_Delta_Num(q,number)
%FABRIKc_Delta 用于求解 delta机器人正运动学
%% 结构参数
L = 60;         % 连续体段长度
D = 500 - 2*L;  % 刚性段长度

R = 20;         % 动平台半径

vector_1 = [0.342020143325669;0;0.939692620785908];
vector_2 = [-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = [-0.171010071662835;-0.296198132726024;0.939692620785908];

moving_platform_2 = [-30;10*sqrt(3);0];
moving_platform_3 = [-30;-10*sqrt(3);0];

Z_base = [0;0;1];

%% 输入的导轨驱动量
q_1 = q(1);
q_2 = q(2);
q_3 = q(3);

%% 以支链1为root 支链2和支链3为target
% root = q_1 * vector_1;      %支链1末端
% target_2 = q_2 * vector_2;  %支链2末端
% target_3 = q_3 * vector_3;  %支链3末端
switch number
    case 1
        root = q_1 * vector_1;      %支链1末端
        target_2 = q_2 * vector_2;  %支链2末端
        target_3 = q_3 * vector_3;  %支链3末端
        moving_platform_2 = [-30;10*sqrt(3);0];
        moving_platform_3 = [-30;-10*sqrt(3);0];
    case 2
        root = q_2 * vector_2;      %支链2末端
        target_2 = q_1 * vector_1;  %支链1末端
        target_3 = q_3 * vector_3;  %支链3末端
        moving_platform_2 = [30;-10*sqrt(3);0];
        moving_platform_3 = [0;-20*sqrt(3);0];
    case 3
        root = q_3 * vector_3;      %支链3末端
        target_2 = q_1 * vector_1;  %支链1末端
        target_3 = q_2 * vector_2;  %支链2末端
        moving_platform_2 = [30;10*sqrt(3);0];
        moving_platform_3 = [0;20*sqrt(3);0];
end

%% 根据构型初值计算当前位置
% 支链1
xita_1 = 0;
P_1_proximal_base = root;
P_1_proximal_joint = P_1_proximal_base + [0;0;-L/2];
P_1_proximal_end = P_1_proximal_joint + [0;0;-L/2];
P_1_distal_base = P_1_proximal_end + [0;0;-D];
P_1_distal_joint = P_1_distal_base + [0;0;-L/2];
P_1_distal_end = P_1_distal_joint + [0;0;-L/2];

% 支链2
xita_2 = 0;
P_2_distal_end = P_1_distal_end + moving_platform_2;
P_2_distal_joint = P_2_distal_end + [0;0;L/2];
P_2_distal_base = P_2_distal_joint + [0;0;L/2];
P_2_proximal_end = P_2_distal_base + [0;0;D];
P_2_proximal_joint = P_2_proximal_end + [0;0;L/2];
P_2_proximal_base = P_2_proximal_joint + [0;0;L/2];

% 支链3
xita_3 = 0;
P_3_distal_end = P_1_distal_end + moving_platform_3;
P_3_distal_joint = P_3_distal_end + [0;0;L/2];
P_3_distal_base = P_3_distal_joint + [0;0;L/2];
P_3_proximal_end = P_3_distal_base + [0;0;D];
P_3_proximal_joint = P_3_proximal_end + [0;0;L/2];
P_3_proximal_base = P_3_proximal_joint + [0;0;L/2];

% 计算位置误差
pos_error = norm(P_2_proximal_base-target_2) + norm(P_3_proximal_base-target_3);
error_s = zeros(1000,1);
%% FABRIKc 迭代
threshold = 2*1e-3;
iteration_index = 0;
tic
while pos_error > threshold
    iteration_index = iteration_index + 1;
    %% forward reaching phase
    % 支链2
    P_2_proximal_base = target_2;
    if xita_2 == 0
        L_2 = L / 2;
    else
        L_2 = L / xita_2 * tan(xita_2/2);
    end
    P_2_proximal_joint = P_2_proximal_base - L_2 * [0;0;1];
    Z_2_internal = (P_2_proximal_joint - P_2_distal_joint)/norm(P_2_proximal_joint - P_2_distal_joint);
    xita_2 = acos(Z_2_internal'*Z_base);
    if xita_2 == 0
        L_2 = L / 2;
    else
        L_2 = L / xita_2 * tan(xita_2/2);
    end
    P_2_proximal_joint = P_2_proximal_base - L_2 * [0;0;1];   
    P_2_proximal_end = P_2_proximal_joint - L_2 * Z_2_internal;
    P_2_distal_base = P_2_proximal_end - D * Z_2_internal;
    P_2_distal_joint = P_2_distal_base - L_2 * Z_2_internal;
    P_2_distal_end = P_2_distal_joint - L_2 * Z_base;
    
    P_1_distal_end_temp_2 = P_2_distal_end - moving_platform_2;
    
    % 支链3
    P_3_proximal_base = target_3;
    if xita_3 == 0
        L_3 = L / 2;
    else
        L_3 = L / xita_3 * tan(xita_3/2);
    end
    P_3_proximal_joint = P_3_proximal_base - L_3 * [0;0;1];
    Z_3_internal = (P_3_proximal_joint - P_3_distal_joint)/norm(P_3_proximal_joint - P_3_distal_joint);
    xita_3 = acos(Z_3_internal'*Z_base);
    if xita_3 == 0
        L_3 = L / 2;
    else
        L_3 = L / xita_3 * tan(xita_3/2);
    end
    P_3_proximal_joint = P_3_proximal_base - L_3 * [0;0;1];   
    P_3_proximal_end = P_3_proximal_joint - L_3 * Z_3_internal;
    P_3_distal_base = P_3_proximal_end - D * Z_3_internal;
    P_3_distal_joint = P_3_distal_base - L_3 * Z_3_internal;
    P_3_distal_end = P_3_distal_joint - L_3 * Z_base;
    
    P_1_distal_end_temp_3 = P_3_distal_end - moving_platform_3;
    
    %支链1
    P_1_distal_end = (P_1_distal_end_temp_2 + P_1_distal_end_temp_3)/2;
    if xita_1 == 0
        L_1 = L / 2;
    else
        L_1 = L / xita_1 * tan(xita_1/2);
    end
    P_1_distal_joint = P_1_distal_end + L_1 * [0;0;1];
    Z_1_internal = (P_1_proximal_joint - P_1_distal_joint) / norm(P_1_proximal_joint - P_1_distal_joint);
    xita_1 = acos(Z_1_internal'*Z_base);
    if xita_1 == 0
        L_1 = L / 2;
    else
        L_1 = L / xita_1 * tan(xita_1/2);
    end
    P_1_distal_joint = P_1_distal_end + L_1 * Z_base;
    P_1_distal_base = P_1_distal_joint + L_1 * Z_1_internal;
    P_1_proximal_end = P_1_distal_base + D * Z_1_internal;
    P_1_proximal_joint = P_1_proximal_end + L_1 * Z_1_internal;
    P_1_proximal_base = P_1_proximal_joint + L_1 * Z_base;
       
    %% backward reaching phase
    %支链1
    P_1_proximal_base = root;
    P_1_proximal_joint =  P_1_proximal_base - L_1 * Z_base;
    
    Z_1_internal = (P_1_proximal_joint - P_1_distal_joint) / norm(P_1_proximal_joint - P_1_distal_joint);
    xita_1 = acos(Z_1_internal'*Z_base);
    
    if xita_1 == 0
        L_1 = L / 2;
    else
        L_1 = L / xita_1 * tan(xita_1/2);
    end
    P_1_proximal_joint = P_1_proximal_base - L_1 * Z_base;
    P_1_proximal_end = P_1_proximal_joint - L_1 * Z_1_internal;
    P_1_distal_base = P_1_proximal_end - D * Z_1_internal;
    P_1_distal_joint = P_1_distal_base - L_1 * Z_1_internal;
    P_1_distal_end = P_1_distal_joint - L_1 * Z_base;
 
    %支链2
    P_2_distal_end = P_1_distal_end + moving_platform_2;
    P_2_distal_joint = P_2_distal_end + L_2 * Z_base;
    Z_2_internal = (P_2_proximal_joint - P_2_distal_joint)/norm(P_2_proximal_joint - P_2_distal_joint);
    xita_2 = acos(Z_2_internal'*Z_base);
    if xita_2 == 0
        L_2 = L / 2;
    else
        L_2 = L / xita_2 * tan(xita_2/2);
    end
    P_2_distal_joint = P_2_distal_end + L_2 * Z_base;
    P_2_distal_base = P_2_distal_joint + L_2 * Z_2_internal;
    P_2_proximal_end = P_2_distal_base + D * Z_2_internal;
    P_2_proximal_joint = P_2_proximal_end + L_2 * Z_2_internal;
    P_2_proximal_base = P_2_proximal_joint + L_2 * Z_base;  
    
    % 支链3
    P_3_distal_end = P_1_distal_end + moving_platform_3;
    P_3_distal_joint = P_3_distal_end + L_3 * Z_base;
    Z_3_internal = (P_3_proximal_joint - P_3_distal_joint)/norm(P_3_proximal_joint - P_3_distal_joint);
    xita_3 = acos(Z_3_internal'*Z_base);
    if xita_3 == 0
        L_3 = L / 2;
    else
        L_3 = L / xita_3 * tan(xita_3/2);
    end
    P_3_distal_joint = P_3_distal_end + L_3 * Z_base;
    P_3_distal_base = P_3_distal_joint + L_3 * Z_3_internal;
    P_3_proximal_end = P_3_distal_base + D * Z_3_internal;
    P_3_proximal_joint = P_3_proximal_end + L_3 * Z_3_internal;
    P_3_proximal_base = P_3_proximal_joint + L_3 * Z_base;
    
    %% 计算误差
    pos_error = norm(P_2_proximal_base-target_2) + norm(P_3_proximal_base-target_3);
    error_s(iteration_index) = pos_error;
end
running_time=toc;



end

