clear
clc
% FABRIKc用于求解delta机器人正运动学 3自由度版

%% 结构参数
global L D;
L = 60;         % 连续体段长度
D = 500 - 2*L;  % 刚性段长度

R = 20;         % 动平台半径

vector_1 = [0.342020143325669;0;0.939692620785908];
vector_2 = [-0.171010071662834;0.296198132726024;0.939692620785908];
vector_3 = [-0.171010071662835;-0.296198132726024;0.939692620785908];

% moving_platform_2 = [-30;10*sqrt(3);0];
% moving_platform_3 = [-30;-10*sqrt(3);0];

Z_base = [0;0;1];

%% 输入的导轨驱动量
global q_1 q_2 q_3;
q_1 = 490;
q_2 = 190;
q_3 = 490;
% q_1 = 190;
% q_2 = 390;
% q_3 = 190.407;
%% 以支链1为root 支链2和支链3为target
number = 2;
global root target_2 target_3;
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
% 支链1(Delta-Chain的root)
global xita_1 xita_2 xita_3;

global P_1_proximal_base P_1_proximal_joint P_1_proximal_end P_1_distal_base P_1_distal_joint P_1_distal_end;
xita_1 = 0;
P_1_proximal_base = root;
P_1_proximal_joint = P_1_proximal_base + [0;0;-L/2];
P_1_proximal_end = P_1_proximal_joint + [0;0;-L/2];
P_1_distal_base = P_1_proximal_end + [0;0;-D];
P_1_distal_joint = P_1_distal_base + [0;0;-L/2];
P_1_distal_end = P_1_distal_joint + [0;0;-L/2];


% 支链2(Delta_Chain的End-effector2)
global P_2_proximal_base P_2_proximal_joint P_2_proximal_end P_2_distal_base P_2_distal_joint P_2_distal_end;
xita_2 = 0;
P_2_distal_end = P_1_distal_end + moving_platform_2;
P_2_distal_joint = P_2_distal_end + [0;0;L/2];
P_2_distal_base = P_2_distal_joint + [0;0;L/2];
P_2_proximal_end = P_2_distal_base + [0;0;D];
P_2_proximal_joint = P_2_proximal_end + [0;0;L/2];
P_2_proximal_base = P_2_proximal_joint + [0;0;L/2];

% 支链3(Delta_Chain的End-effector3)
xita_3 = 0;
global P_3_proximal_base P_3_proximal_joint P_3_proximal_end P_3_distal_base P_3_distal_joint P_3_distal_end;
P_3_distal_end = P_1_distal_end + moving_platform_3;
P_3_distal_joint = P_3_distal_end + [0;0;L/2];
P_3_distal_base = P_3_distal_joint + [0;0;L/2];
P_3_proximal_end = P_3_distal_base + [0;0;D];
P_3_proximal_joint = P_3_proximal_end + [0;0;L/2];
P_3_proximal_base = P_3_proximal_joint + [0;0;L/2];

% 计算位置误差
pos_error = norm(P_2_proximal_base-target_2) + norm(P_3_proximal_base-target_3);
error_s = zeros(1000,1);
xita_1_s = zeros(1000,1);
xita_2_s = zeros(1000,1);
xita_3_s = zeros(1000,1);

delta_1_s = zeros(1000,1);
delta_2_s = zeros(1000,1);
delta_3_s = zeros(1000,1);
%% FABRIKc 迭代
threshold = 1e-3;
iteration_index = 0;
broyden_number = 1;
J=[eye(6) eye(6) eye(6)]';
last_joint_position = [P_1_proximal_joint; P_1_distal_joint; ...
            P_2_proximal_joint; P_2_distal_joint; ...
            P_3_proximal_joint; P_3_distal_joint;];
last_end_position = [P_2_proximal_base; P_3_proximal_base];
tic
% FABRIKc_process_draw();
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
    xita_1_s(iteration_index) = xita_2/pi*180;
    xita_2_s(iteration_index) = xita_1/pi*180;
    xita_3_s(iteration_index) = xita_3/pi*180;
    
    delta_1 = -atan2(P_2_proximal_base(2)-P_2_distal_base(2),P_2_proximal_base(1)-P_2_distal_base(1));
    delta_2 = -atan2(P_1_proximal_base(2)-P_1_distal_base(2),P_1_proximal_base(1)-P_1_distal_base(1));
    delta_3 = -atan2(P_3_proximal_base(2)-P_3_distal_base(2),P_3_proximal_base(1)-P_3_distal_base(1));

    delta_1_s(iteration_index) = delta_1/pi*180;
    delta_2_s(iteration_index) = delta_2/pi*180;
    delta_3_s(iteration_index) = delta_3/pi*180;

    if(iteration_index>1000)
        current_joint_position = [P_1_proximal_joint; P_1_distal_joint; ...
            P_2_proximal_joint; P_2_distal_joint; ...
            P_3_proximal_joint; P_3_distal_joint;];
        current_end_position = [P_2_proximal_base; P_3_proximal_base];
        dp = current_joint_position-last_joint_position;
        dx = current_end_position-last_end_position;
        J=BadBroydenJacobian(dp,dx,J);
        if(iteration_index>15)
            dx_ = -[P_2_proximal_base - target_2; ...
                P_3_proximal_base - target_3];
            dp = J*dx_;
            P_1_proximal_joint = P_1_proximal_joint+dp(1:3);
            P_1_distal_joint = P_1_distal_joint+dp(4:6);
            P_2_proximal_joint = P_2_proximal_joint+dp(7:9);
            P_2_distal_joint = P_2_distal_joint+dp(10:12);
            P_3_proximal_joint = P_3_proximal_joint+dp(13:15);
            P_3_distal_joint = P_3_distal_joint+dp(16:18);
            broyden_number = broyden_number + 1;
        end
        last_joint_position = current_joint_position;
        last_end_position = current_end_position;
    end

end
FABRIKc_process_draw();
toc
%%  后续处理
pos_moving_platform = 1/3*(P_1_distal_end+P_2_distal_end+P_3_distal_end);
delta_1 = -atan2(P_2_proximal_base(2)-P_2_distal_base(2),P_2_proximal_base(1)-P_2_distal_base(1));
delta_2 = -atan2(P_1_proximal_base(2)-P_1_distal_base(2),P_1_proximal_base(1)-P_1_distal_base(1));
delta_3 = -atan2(P_3_proximal_base(2)-P_3_distal_base(2),P_3_proximal_base(1)-P_3_distal_base(1));
switch number
    case 1
        psi = [q_1;xita_1;delta_1;q_2;xita_2;delta_2;q_3;xita_3;delta_3];
    case 2
        psi = [q_1;xita_2;delta_2;q_2;xita_1;delta_1;q_3;xita_3;delta_3];
    case 3
        psi = [q_1;xita_2;delta_2;q_2;xita_3;delta_3;q_3;xita_1;delta_1];
end


% figure(1);
% plot_delta_robot(psi,pos_moving_platform);
% figure(2);
% plot(error_s(1:iteration_index));
% figure(3);view(2)
% hold on;
% yyaxis left;
% xlabel('Iterarion index');
% ylabel('{\itθ}(°)');
% plot(xita_1_s(1:iteration_index),'-','color',[0 0.45 0.74],'linewidth',2);
% 
% plot(xita_2_s(1:iteration_index),'--','color',[0 0.45 0.74],'linewidth',2);
% 
% yyaxis right;
% ylabel('{\itδ}(°)');
% plot(delta_1_s(1:iteration_index),'-','color','r','linewidth',2);
% plot(delta_2_s(1:iteration_index),'--','color','r','linewidth',2);
% plot(delta_3_s(1:iteration_index),'-.','color','r','linewidth',2);
% legend('{\itθ}_1 and {\itθ}_3','{\itθ}_2','{\itδ}_1','{\itδ}_2','{\itδ}_3');
% hold off;



%%

% 生成原始数据
x = 1:1:length(error_s(1:iteration_index)); % 原始数据点的 x 值
y = error_s(1:iteration_index); % 原始数据点的 y 值

% 扩展后的数据点的 x 值
x1 = 1:(length(error_s(1:iteration_index)));

% 使用 interp1 函数进行插值
y_extended = interp1(x, y, x1, 'spline'); % 使用样条插值

font_size = 30;
% x1=1:length(error);
tt=[256.575860235455
206.635455040622
161.958362994665
124.628939409522
94.7980686810313
71.6042790721487
53.8623357975585
40.4190302356612
30.2885835304701
22.6786909952332
16.9726349210021
12.6986442446638
9.49929358731600
7.10525737697124
5.31422308726672
3.97448876665500
0.185440376188128
0.182467446479423
0.179459630821637
0.0401469423387211
0.000124544233523784];
x2=1:length(tt);
% figure;
% loglog(1:100,100:-1:1)
semilogy(1:100,100:-1:1)
cla;

hold on;
grid on;
% loglog(x1, error,'LineWidth', 2);
% loglog(x2, tt,'LineWidth', 2);
semilogy(x1, y_extended,'LineWidth', 2)
semilogy(x2, tt,'LineWidth', 2)
xlabel("iteration",'FontName', 'Times New Roman', 'FontSize', font_size)
ylabel("error (mm)",'FontName', 'Times New Roman', 'FontSize', font_size)
legend('FABRIKc Delta','FM-FABRIKc');
set(gca, 'FontSize', font_size)