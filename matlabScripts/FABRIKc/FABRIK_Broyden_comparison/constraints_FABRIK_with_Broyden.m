function [flag, time,iteration,error_PR,THETA]=constraints_FABRIK_with_Broyden(target,LS,initial_pose,err_epsilon,kmax,limit)

%% Broyden
JBro_forward = eye(3);
cur_tar_forward = zeros(3,1);
cur_pos_forward = zeros(3,1);
last_tar_forward = cur_tar_forward;
last_pos_forward = cur_pos_forward;

% JBro_backward = [eye(3);[1 1 1]];
% cur_tar_backward = zeros(3,1);
% cur_pos_backward = zeros(4,1);
% last_tar_backward = cur_tar_backward;
% last_pos_backward = cur_pos_backward;

%% initialization
%部分构型参数
Lg=LS(4);
L10=LS(1);
Lr0=LS(2);
L20=LS(3);
theta1_limit = limit(1)*1.1;
theta2_limit = limit(2)*1.1;

%输入目标位姿
orientation_target = target(1:3,1:3);
position_target = target(1:3,4) - Lg*orientation_target(1:3,3);

%迭代参数
error_P_desired=err_epsilon;
k_max=kmax;

%输入当前位姿并初始化虚拟关节位置和虚拟杆长
position_current = initial_pose(1:3,4);
% position_current = initial_pose(1:3,4);
% l1 = LS(1) / 2;
% l2 = LS(3) / 2;
% P_2j = position_target - l2;
% P_1j = P_2j - l1 - l2;
% l1 = initial_pose(4,1);
l2 = initial_pose(4,2);
P_1j = initial_pose(1:3,1);
% l1 = LS(1) / 2;
% l2 = LS(3) / 2;
% P_2j = position_target - l2;
% P_1j = P_2j - l1 - l2;
% z_1b = [0 0 1]';
theta1 = 0;
theta2 = 0;
deadlock_cter=[1 10 100];

%计算位置误差
error_current=norm(position_current-position_target);
% error_block = zeros(k_max,1);
%

%迭代次数初始化
k=0;

%% execution
tic;
while k < k_max && error_current > error_P_desired
    k = k + 1;
    if(k>3)
        dp_forward = cur_pos_forward-last_pos_forward;
        dx_forward = cur_tar_forward-last_tar_forward;
        JBro_forward=BadBroydenJacobian(dp_forward,dx_forward,JBro_forward);
        if(k>7)
            
            % dx_norm = norm(dx_forward);
            if(theta1==theta1_limit  || theta2==theta2_limit)
                % dp_forward2 = [dp_forward(2) dp_forward(3) dp_forward(1)]'/norm(dp_forward);
                dp_forward2 = dp_forward/norm(dp_forward);
                % dx_forward2 = dx_forward/dx_norm;
                % tem = cross(dp_forward2,dx_forward2);
                % tem_norm = norm(tem);
                % if(tem_norm == 0)
                %     tem2 = z_1b;
                % else
                %     tem2 = tem/tem_norm;
                % end
                % dp2=-dp_forward2-tem2;%norm(position_target-cur_tar)*dp/norm(dp);   
                dp2 = rand(3,1);
                dp2=dp2/norm(dp2)*0.1-dp_forward2*0.1;
                dp_forward=dp2;
            else
                dp_forward = JBro_forward*(position_target-cur_tar_forward);
            end
            P_1j = P_1j + dp_forward;
        end
    end

    
    % error(k) = error_P;
    %forward reaching phase
    %%%%%%%CONTINUUM2%%%%%%%%%%%%%%
    z_2e = orientation_target(:,3);
    z_2e = z_2e/norm(z_2e);
    P_2e = position_target;  
    P_2j = P_2e - l2 * z_2e;
    z_2b = (P_2j - P_1j)/norm(P_2j - P_1j);
    
    dot2f = z_2b'*z_2e;
    if(dot2f>1)
        dot2f = 1;
    end
    theta2 = acos(dot2f);
    % if theta2 > theta2_limit
    %     theta2 = theta2_limit;
    %     e_1 = z_2e;
    %     e_2 = z_2b - z_2e * dot2f;
    %     e_2 = e_2/norm(e_2);
    %     z_2b = cos(theta2) * e_1 + sin(theta2) * e_2;
    % end

    if theta2 == 0
        l2 = L20/2;
    else
        l2 = L20/theta2 * tan(theta2/2);
    end
    
    P_2j = P_2e - l2 * z_2e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%CONTINUUM1%%%%%%%%%%%%%%
    z_1e = z_2b;
    P_1e = P_2j - l2 * z_2b - Lr0 * z_2b;
     
    z_1b = [0;0;1];
    
    dot1f = z_1b'*z_1e;
    if(dot1f>1)
        dot1f = 1;
    end
    theta1 = acos(dot1f);
    % if theta1 > theta1_limit
    %     theta1 = theta1_limit;
    %     e_1 = z_1e;
    %     e_2 = z_1b - z_1e * dot1f;
    %     e_2 = e_2/norm(e_2);
    %     z_1b = cos(theta1) * e_1 + sin(theta1) * e_2;
    % end

    if theta1 == 0
        l1 = L10/2;
    else
        l1 = L10/theta1 * tan(theta1/2);
    end
    
    P_1j = P_1e - l1 * z_1e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%STEM%%%%%%%%%%%%%%%
    P_STEM = P_1j - l1 * z_1b;
    Ls = P_STEM(3);
    if(Ls<0)
        Ls = 0;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    

    % last_pos_backward = cur_pos_backward;
    % cur_pos_backward = [P_2j; Ls];
    % last_tar_backward = cur_tar_backward;
    % cur_tar_backward = P_STEM;cur_tar_backward(3) = 0;
    % if(k>3)
    %     dp_backward = cur_pos_backward-last_pos_backward;
    %     dx_backward = cur_tar_backward-last_tar_backward;
    %     JBro_backward=BadBroydenJacobian(dp_backward,dx_backward,JBro_backward);
    %     if(k>7)
    %         dp_backward = JBro_backward*([0 0 0]'-cur_tar_backward);
    %         % if(norm(dx)<error_P_desired/100)
    %         %     dp2=norm(position_target-cur_tar)*dp/norm(dp);   
    %         %     dp=dp2;
    %         % end
    %         P_2j = P_2j + dp_backward(1:3);
    %         Ls = Ls + dp_backward(4);
    %     end
    % end
    % if(k>7)
    %     P_2j = P_2j + dp_forward(4:6);
    % end


    %backward reaching phase
    %%%%%%%%%%%%STEM%%%%%%%%%%%%%%%

    P_STEM = [0;0;Ls];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%CONTINUUM1%%%%%%%%%%%%%%
    z_1b = [0;0;1];
    P_1b = P_STEM;
    
    P_1j = P_1b + l1 * z_1b;
    
    z_1e = (P_2j - P_1j)/norm(P_2j - P_1j);
    
    dot1f = z_1b'*z_1e;
    if(dot1f>1)
        dot1f = 1;
    end
    theta1 = acos(dot1f);
    if theta1 > theta1_limit
        theta1 = theta1_limit;
        e_1 = z_1b;
        e_2 = z_1e - z_1b * dot1f;
        e_2 = e_2/norm(e_2);
        z_1b = cos(theta1) * e_1 + sin(theta1) * e_2;
    end
    
    if theta1 == 0
        l1 = L10/2;
    else
        l1 = L10/theta1 * tan(theta1/2);
    end
    
    P_1j = P_1b + l1 * z_1b;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%CONTINUUM2%%%%%%%%%%%%%%
    z_2b = z_1e;
    P_2b = P_1j + l1 * z_1e + Lr0 * z_1e; 
    
    z_2e = orientation_target(:,3);
    
    dot2f = z_2b'*z_2e;
    if(dot2f>1)
        dot2f = 1;
    end
    theta2 = acos(dot2f);
    if theta2 > theta2_limit
        theta2 = theta2_limit;
        e_1 = z_2b;
        e_2 = z_2e - z_2b * dot2f;
        e_2 = e_2/norm(e_2);
        z_2e = cos(theta2) * e_1 + sin(theta2) * e_2;
    end
    
    if theta2 == 0
        l2 = L20/2;
    else
        l2 = L20/theta2 * tan(theta2/2);
    end
    
    P_2j = P_2b + l2 * z_2b;
    position_current = P_2j + l2 * z_2e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    %计算位置误差
    last_tar_forward = cur_tar_forward;
    cur_tar_forward = position_current;
    
    last_pos_forward = cur_pos_forward;
    cur_pos_forward = P_1j;
    error_current=norm(position_current-position_target); 
    % error_block(k) = error_current;
    if(k>3)
        deadlock_cter(1)=deadlock_cter(2);
        deadlock_cter(2)=deadlock_cter(3);
        deadlock_cter(3)=error_current;
        if(abs(deadlock_cter(1)-deadlock_cter(2)) == abs(deadlock_cter(2)-deadlock_cter(3))&& deadlock_cter(1)==deadlock_cter(3))
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

dotf = z_2e'*orientation_target(:,3);
if(dotf > 1)
    dotf = 1;
end
errorR = acos(dotf);
error_PR = [errorP errorR];
THETA = [theta1 theta2];
end

function J_cur=BadBroydenJacobian(dp,dx,J_last)

alpha = 1;
J_cur = J_last + alpha*(dp-J_last*dx)/(dx'*dx)*dx';
end