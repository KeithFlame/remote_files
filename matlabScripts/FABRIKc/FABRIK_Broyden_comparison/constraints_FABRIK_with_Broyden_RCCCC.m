function [flag, time,iteration,error_PR,THETA]=constraints_FABRIK_with_Broyden_RCCCC(target,LS,initial_pose,err_epsilon,kmax,limit)

%% Broyden
JBro_forward1 = eye(3);JBro_forward2 = eye(3);
cur_tar_forward = zeros(3,1);
cur_pos_forward = zeros(6,1);
last_tar_forward = cur_tar_forward;
last_pos_forward = cur_pos_forward;

%% 部分构型参数
L1=LS(1);
L2=LS(2);
L3=LS(3);
L4=LS(4);
theta_limit = limit;

%% 输入目标位姿
orientation_target = target(1:3,1:3);
position_target = target(1:3,4);

%% 迭代参数
error_P_desired=err_epsilon;
k_max=kmax;

%% 输入当前位姿并初始化虚拟关节位置和虚拟杆长
% position_current = initial_pose(1:3,4);
% l4 = initial_pose(4,4);
% l3 = initial_pose(4,3);
% l2 = initial_pose(4,2);
% P_1j = initial_pose(1:3,1);
% P_2j = initial_pose(1:3,2);
% P_3j = initial_pose(1:3,3);
position_current = [0 0 400]';
l4 = 50;
l3 = 50;
l2 = 50;
P_1j = [0 0 50]';
P_2j = [0 0 150]';
P_3j = [0 0 250]';
deadlock_cter=[1 10 100];

%计算位置误差
error_current=norm(position_current-position_target);
%

%% execution
k=0;
tic;
while k < k_max && error_current > error_P_desired
    if(k>5)
        dp_forward = cur_pos_forward-last_pos_forward;
        dx_forward = cur_tar_forward-last_tar_forward;
        [JBro_forward1,JBro_forward2] = BadBroydenJacobian(dp_forward,dx_forward,JBro_forward1,JBro_forward2);
        if(mod(k,5)==0)
            
            % dx_norm = norm(dx_forward);
            c1 = [theta1 theta2 theta3 theta4]'-theta_limit;
            c2 = sum(c1>0);
            
            if(c2>1)%theta1==theta_limit  || theta2==theta_limit || theta3==theta_limit || theta4==theta_limit
                dp_forward2 = dp_forward/norm(dp_forward); 
                dp2 = rand(6,1);
                dp2=dp2/norm(dp2)*0.1-dp_forward2*0.1;
                dp_forward=dp2;
            else
                dx_tar = position_target-cur_tar_forward;
                dp_forward(1:3) = JBro_forward1*(dx_tar);
                dp_forward(4:6) = JBro_forward2*(dx_tar);
            end
            P_2j = P_2j + dp_forward(1:3);
            P_3j = P_3j + dp_forward(4:6);
        end
    end
    k = k + 1;

    % forward reaching phase
    %--------------------------------------------------------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 4%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_4e = orientation_target(:,3);
    P_4e = position_target;  
    P_4j = P_4e - l4 * z_4e;
    z_4b = (P_4j - P_3j)/norm(P_4j - P_3j);
    
    dot4f = z_4b'*z_4e;
    if(dot4f>1)
        dot4f = 1;
    end
    theta4 = acos(dot4f);
    % if theta4 > theta_limit
    %     theta4 = theta_limit;
    %     e_1 = z_4e;
    %     e_2 = z_4b - z_4e * dot4f;
    %     e_2 = e_2/norm(e_2);
    %     z_4b = cos(theta4) * e_1 + sin(theta4) * e_2;
    % end

    if theta4 == 0
        l4 = L4/2;
    else
        l4 = L4/theta4 * tan(theta4/2);
    end
    
    P_4j = P_4e - l4 * z_4e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 3%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_3e = z_4b;
    P_3e = P_4j - l4 * z_4b;
    P_3j = P_3e - l3 * z_4b;
    z_3b = (P_3j - P_2j)/norm(P_3j - P_2j);
    
    dot3f = z_3b'*z_3e;
    if(dot3f>1)
        dot3f = 1;
    end
    theta3 = acos(dot3f);
    % if theta3 > theta_limit
    %     theta3 = theta_limit;
    %     e_1 = z_3e;
    %     e_2 = z_3b - z_3e * dot3f;
    %     e_2 = e_2/norm(e_2);
    %     z_3b = cos(theta3) * e_1 + sin(theta3) * e_2;
    % end

    if theta3 == 0
        l3 = L3/2;
    else
        l3 = L3/theta3 * tan(theta3/2);
    end
    
    P_3j = P_3e - l3 * z_3e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_2e = z_3b;
    P_2e = P_3j - l3 * z_3b;
    P_2j = P_2e - l2 * z_3b;
    z_2b = (P_2j - P_1j)/norm(P_2j - P_1j);
    
    dot2f = z_2b'*z_2e;
    if(dot2f>1)
        dot2f = 1;
    end
    theta2 = acos(dot2f);
    % if theta2 > theta_limit
    %     theta2 = theta_limit;
    %     e_1 = z_2e;
    %     e_2 = z_2b - z_2e * dot2f;
    %     e_2 = e_2/norm(e_2);
    %     z_2b = cos(theta2) * e_1 + sin(theta2) * e_2;
    % end

    if theta2 == 0
        l2 = L2/2;
    else
        l2 = L2/theta2 * tan(theta2/2);
    end
    
    P_2j = P_2e - l2 * z_2e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_1e = z_2b;
    % P_1e = P_2j - l2 * z_2b;
    % P_1j = P_1e - l2 * z_3b;
    z_1b = [0 0 1]';
    
    dot1f = z_1b'*z_1e;
    if(dot1f>1)
        dot1f = 1;
    end
    theta1 = acos(dot1f);
    % if theta1 > theta_limit
    %     theta1 = theta_limit;
    %     % e_1 = z_1e;
    %     % e_2 = z_1b - z_1e * dot1f;
    %     % e_2 = e_2/norm(e_2);
    %     % z_1b = cos(theta1) * e_1 + sin(theta1) * e_2;
    % end

    if theta1 == 0
        l1 = L1/2;
    else
        l1 = L1/theta1 * tan(theta1/2);
    end
    
    % P_1j = P_1e - l1 * z_1e;
    
    % backward reaching phase
    %--------------------------------------------------------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_1b = [0;0;1];
    P_1b = [0 0 0]';
    
    P_1j = P_1b + l1 * z_1b;
    
    z_1e = (P_2j - P_1j)/norm(P_2j - P_1j);
    
    dot1f = z_1b'*z_1e;
    if(dot1f>1)
        dot1f = 1;
    end
    theta1 = acos(dot1f);
    if theta1 > theta_limit
        theta1 = theta_limit;
        e_1 = z_1b;
        e_2 = z_1e - z_1b * dot1f;
        e_2 = e_2/norm(e_2);
        z_1e = cos(theta1) * e_1 + sin(theta1) * e_2;
    end
    
    if theta1 == 0
        l1 = L1/2;
    else
        l1 = L1/theta1 * tan(theta1/2);
    end
    
    P_1j = P_1b + l1 * z_1b;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 2%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_2b = z_1e;
    P_2b = P_1j + l1 * z_1e;
    
    P_2j = P_2b + l2 * z_2b;
    
    z_2e = (P_3j - P_2j)/norm(P_3j - P_2j);
    
    dot2f = z_2b'*z_2e;
    if(dot2f>1)
        dot2f = 1;
    end
    theta2 = acos(dot2f);
    if theta2 > theta_limit
        theta2 = theta_limit;
        e_1 = z_2b;
        e_2 = z_2e - z_2b * dot1f;
        e_2 = e_2/norm(e_2);
        z_2e = cos(theta2) * e_1 + sin(theta2) * e_2;
    end
    
    if theta2 == 0
        l2 = L2/2;
    else
        l2 = L2/theta2 * tan(theta2/2);
    end
    
    P_2j = P_2b + l2 * z_2b;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 3%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_3b = z_2e;
    P_3b = P_2j + l2 * z_2e;
    P_3j = P_3b + l3 * z_3b;
    z_3e = (P_4j - P_3j)/norm(P_4j - P_3j);
    
    dot3f = z_3b'*z_3e;
    if(dot3f>1)
        dot3f = 1;
    end
    theta3 = acos(dot3f);
    if theta3 > theta_limit
        theta3 = theta_limit;
        e_1 = z_3b;
        e_2 = z_3e - z_3b * dot1f;
        e_2 = e_2/norm(e_2);
        z_3e = cos(theta3) * e_1 + sin(theta3) * e_2;
    end

    if theta3 == 0
        l3 = L3/2;
    else
        l3 = L3/theta3 * tan(theta3/2);
    end
       
    P_3j = P_3b + l3 * z_3b;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 4%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_4b = z_3e;
    P_4b = P_3j + l3 * z_3e; 
    z_4e = orientation_target(:,3);
    dot4f = z_4b'*z_4e;
    if(dot4f>1)
        dot4f = 1;
    end
    theta4 = acos(dot4f);
    if theta4 > theta_limit
        theta4 = theta_limit;
        e_1 = z_4b;
        e_2 = z_4e - z_4b * dot4f;
        e_2 = e_2/norm(e_2);
        z_4e = cos(theta4) * e_1 + sin(theta4) * e_2;
    end
    
    if theta4 == 0
        l4 = L4/2;
    else
        l4 = L4/theta4 * tan(theta4/2);
    end
    
    P_4j = P_4b + l4 * z_4b;
    position_current = P_4j + l4 * z_4e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    last_tar_forward = cur_tar_forward;
    cur_tar_forward = position_current;
    
    last_pos_forward = cur_pos_forward;
    cur_pos_forward = [P_2j;P_3j];

    %计算位置误差
    error_current=norm(position_current-position_target);  
    if(k>3)
        deadlock_cter(1)=deadlock_cter(2);
        deadlock_cter(2)=deadlock_cter(3);
        deadlock_cter(3)=error_current;
        if(abs(deadlock_cter(1)-deadlock_cter(2)) == abs(deadlock_cter(2)-deadlock_cter(3)))
            % flag = -2;
            break;
        end
    end
end
time=toc;

if(error_current<error_P_desired)
    flag = 1;
    iteration = k;
elseif(k == k_max)
    flag = -1;
    iteration = k_max;
else
    flag = -2;
    iteration = k;
end
errorP = norm(position_current-position_target);

dotf = z_4e'*orientation_target(:,3);
if(dotf > 1)
    dotf = 1;
end
errorR = acos(dotf);
error_PR = [errorP errorR];
THETA = [theta1 theta2 theta3 theta4];
end

function [J_cur1,J_cur2] = BadBroydenJacobian(dp,dx,J_last1,J_last2)

alpha = 1;
dx_norm = (dx'*dx);
J_cur1 = J_last1 + alpha*(dp(1:3)-J_last1*dx)/dx_norm*dx';
J_cur2 = J_last2 + alpha*(dp(4:6)-J_last2*dx)/dx_norm*dx';
end