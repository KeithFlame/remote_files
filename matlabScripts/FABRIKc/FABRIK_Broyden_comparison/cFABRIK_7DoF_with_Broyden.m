function [flag, time,iteration,error_PR,THETA]=cFABRIK_7DoF_with_Broyden(target,LS,initial_pose,err_epsilon,kmax,limit)

%% Broyden
JBro_forward = [eye(3);eye(3);eye(3)];
last_pose = zeros(9,1);
cur_pose = zeros(9,1);
last_end = zeros(3,1);
cur_end = zeros(3,1);
%%
%部分构型参数
L10 = LS(1);
L20 = LS(2);
L30 = LS(3);
L40 = LS(4);
% L4 = L40;
La = 20;
theta1_limit = limit(1);
theta2_limit = limit(2);
theta3_limit = limit(3);
theta4_limit = limit(4);
theta1 = 0;
theta2 = 0;
theta3 = 0;
theta4 = 0;
%输入目标位姿
orientation_target = target(1:3,1:3);
position_target = target(1:3,4);

%迭代参数
error_P_desired=err_epsilon;
k_max=kmax;

%输入当前位姿并初始化虚拟关节位置和虚拟杆长
position_current = [0 0 L10+L20+L30]';
P1 = [0 0 0]';
P2 = [0 0 L10]';
P3 = [0 0 L10+L20]';
deadlock_cter=[1 10 100];

%计算位置误差
error_current=norm(position_current-position_target);
error_block = zeros(k_max,1);
%

%迭代次数初始化
k=0;
m = 0;
tic;
is_plot = 0;
if(is_plot)
    figure;
    hold on;grid on;
    axis equal;
    h1 = plot3([0 0], [0 0], [0 50],'r-');
    h2 = plot3([0 0], [0 0], [50 250],'g-');
    h3 = plot3([0 0], [0 0], [250 350],'b-');
    h4 = plot3([0 0], [0 0], [350 475],'c-');
    h5 = plot3([0 0], [0 0], [475 525],'k-');
    
    % 绘制平面
    [XX1, YY1] = meshgrid(-50:10:50, -50:10:50);
    A = [0 0 0]';P = [0 0 0]'; N = [0 0 1]';P_proj = P;
    ZZ1 = A(3) + (N(1)*(XX1 - A(1)) + N(2)*(YY1 - A(2))) / - N(3);
    hs_for = surf(XX1, YY1, ZZ1, 'FaceAlpha', 0.2, 'EdgeColor', 'none','FaceColor','red');
    hs_back = surf(XX1, YY1, ZZ1, 'FaceAlpha', 0.2, 'EdgeColor', 'none','FaceColor','cyan');

    % 绘制被投影的点 P
    hd_proj1 = plot3(P(1), P(2), P(3), 'ro', 'MarkerSize', 10);
    hd_proj2 = plot3(P(1), P(2), P(3), 'co', 'MarkerSize', 10);

    % 绘制投影点 P_proj
    hd_projed1 = plot3(P_proj(1), P_proj(2), P_proj(3), 'ko', 'MarkerSize', 10);
    hd_projed2 = plot3(P_proj(1), P_proj(2), P_proj(3), 'ko', 'MarkerSize', 10);

    % 绘制点 P 和 P_proj 之间的连线
    hd_line1 = line([P(1), P_proj(1)], [P(2), P_proj(2)], [P(3), P_proj(3)], 'Color', 'k', 'LineStyle', '--');
    hd_line2 = line([P(1), P_proj(1)], [P(2), P_proj(2)], [P(3), P_proj(3)], 'Color', 'k', 'LineStyle', '--');
    
    plotCoord_keith(target,50,1);
    view([-130 31])
end
while k < k_max && error_current > error_P_desired
    k = k + 1;
    
    % Broyden
    if(k>5)
        dp_forward = cur_pose-last_pose;
        dx_forward = cur_end-last_end;
        JBro_forward=BadBroydenJacobian(dp_forward,dx_forward,JBro_forward);
        m = m + 1;
        if(m>5)
            dp_forward2 = JBro_forward*(position_target-cur_end);
            P3 = P3 + dp_forward2(1:3);
            P2 = P2 + dp_forward2(4:6);
            P1 = P1 + dp_forward2(7:9);
            % if(norm(dp_forward2)>10)
            %     m = 1;
            % end
        end
    end

    % forward reaching phase
    %--------------------------------------------------------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_4 = orientation_target(:,3);
    L4 = L40 + La/2/cos(theta4/2);
    P4 = position_target - z_4*L4;
    z_3 = (P4 - P3)/norm(P4 - P3);
    dot4f = z_3'*z_4;
    if(dot4f>1)
        dot4f = 1;
    end
    theta4 = acos(dot4f);
    if theta4 > theta4_limit
        theta4 = theta4_limit;
        e_1 = z_4;
        e_2 = z_3 - z_4 * dot4f;
        e_2 = e_2/norm(e_2);
        z_3 = cos(theta4) * e_1 + sin(theta4) * e_2;
    end
    
    L3 = L30 + La/2/cos(theta4/2) + La/2/cos(theta3/2);
    P3 = P4 - L3 * z_3;
    
    if(is_plot)
        set(h4, 'XData', [P3(1) P4(1)]);set(h4, 'YData', [P3(2) P4(2)]);set(h4, 'ZData', [P3(3) P4(3)]);
        set(h5, 'XData', [P4(1) position_target(1)]);set(h5, 'YData', [P4(2) position_target(2)]);
        set(h5, 'ZData', [P4(3) position_target(3)]);
        pause(0.5);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % z_2 = (P3 - P1)/norm(P3 - P1);
    [P2_dot,N1] = projectPointToPlane(P2,P3,P3-P1,z_3);
    z_2 = (P3 - P2_dot)/norm(P3 - P2_dot);
    dot3f = z_2'*z_3;
    if(dot3f>1)
        dot3f = 1;
    end
    theta3 = acos(dot3f);
    if theta3 > theta3_limit
        theta3 = theta3_limit;
        e_1 = z_3;
        e_2 = z_2 - z_3 * dot3f;
        e_2 = e_2/norm(e_2);
        z_2 = cos(theta3) * e_1 + sin(theta3) * e_2;
    end
    L2 = L20 + La/2/cos(theta2/2) + La/2/cos(theta3/2);
    P2 = P3 - L2 * z_2;
    if(is_plot)
        set(h3, 'XData', [P2(1) P3(1)]);set(h3, 'YData', [P2(2) P3(2)]);set(h3, 'ZData', [P2(3) P3(3)]);
        PP = [P1 P2 P3];
        set(hs_for, 'XData', PP(1,:));set(hs_for, 'YData', PP(2,:));set(hs_for, 'ZData', PP(3,:));
        set(hd_proj1, 'XData', P2(1));set(hd_proj1, 'YData', P2(2));set(hd_proj1, 'ZData', P2(3));
        set(hd_projed1, 'XData', P2_dot(1));set(hd_proj1, 'YData', P2_dot(2));set(hd_proj1, 'ZData', P2_dot(3));
        set(hd_line1, 'XData', [P2(1) P2_dot(1)]);set(hd_line1, 'YData', [P2(2) P2_dot(2)]);set(hd_line1, 'ZData', [P2(3) P2_dot(3)]);
        pause(0.5);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_1 = (P2 - P1)/norm(P2 - P1);  
    dot2f = z_1'*z_2;
    if(dot2f>1)
        dot2f = 1;
    end
    theta2 = acos(dot2f);
    if theta2 > theta2_limit
        theta2 = theta2_limit;
        e_1 = z_2;
        e_2 = z_1 - z_2 * dot2f;
        e_2 = e_2/norm(e_2);
        z_1 = cos(theta2) * e_1 + sin(theta2) * e_2;
    end
   
    if(is_plot)
        L1 = L10 + La/2/cos(theta2/2);
        P1 = P2 - L1 * z_1;
        set(h2, 'XData', [P1(1) P2(1)]);set(h2, 'YData', [P1(2) P2(2)]);set(h2, 'ZData', [P1(3) P2(3)]);
        pause(0.5);
    end
        
    % backward reaching phase
    %--------------------------------------------------------------------%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_0 = [0;0;1];
    P1 = [0 0 50]';
    z_1 = (P2 - P1)/norm(P2 - P1);
    
    dot1f = z_0'*z_1;
    if(dot1f>1)
        dot1f = 1;
    end
    theta1 = acos(dot1f);
    if theta1 > theta1_limit
        theta1 = theta1_limit;
        e_1 = z_0;
        e_2 = z_1 - z_0 * dot1f;
        e_2 = e_2/norm(e_2);
        z_1 = cos(theta1) * e_1 + sin(theta1) * e_2;
    end
    L1 = L10 + La/2/cos(theta2/2);
    P2 = P1 + L1 * z_1;
    if(is_plot)
        set(h2, 'XData', [P1(1) P2(1)]);set(h2, 'YData', [P1(2) P2(2)]);set(h2, 'ZData', [P1(3) P2(3)]);
        pause(0.5);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [P3_dot, N2] = projectPointToPlane(P3,P2,z_1,P2-P4);
    z_2 = (P3_dot - P2)/norm(P3_dot - P2);
    
    dot2f = z_1'*z_2;
    if(dot2f>1)
        dot2f = 1;
    end
    theta2 = acos(dot2f);
    if theta2 > theta2_limit
        theta2 = theta2_limit;
        e_1 = z_1;
        e_2 = z_2 - z_1 * dot2f;
        e_2 = e_2/norm(e_2);
        z_2 = cos(theta2) * e_1 + sin(theta2) * e_2;
    end
    
    L2 = L20 + La/2/cos(theta2/2) + La/2/cos(theta3/2);
    P3 = P2 + L2 * z_2;
    if(is_plot)
        set(h3, 'XData', [P2(1) P3(1)]);set(h3, 'YData', [P2(2) P3(2)]);set(h3, 'ZData', [P2(3) P3(3)]);
        PP = [P2 P3 P4];
        set(hs_back, 'XData', PP(1,:));set(hs_back, 'YData', PP(2,:));set(hs_back, 'ZData', PP(3,:));
        set(hd_proj2, 'XData', P3(1));set(hd_proj2, 'YData', P3(2));set(hd_proj2, 'ZData', P3(3));
        set(hd_projed2, 'XData', P3_dot(1));set(hd_proj2, 'YData', P3_dot(2));set(hd_proj2, 'ZData', P3_dot(3));
        set(hd_line2, 'XData', [P3(1) P3_dot(1)]);set(hd_line2, 'YData', [P3(2) P3_dot(2)]);set(hd_line2, 'ZData', [P3(3) P3_dot(3)]);
        pause(0.5);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_3 = (P4 - P3)/norm(P4 - P3);
    
    dot3f = z_2'*z_3;
    if(dot3f>1)
        dot3f = 1;
    end
    theta3 = acos(dot3f);
    if theta3 > theta3_limit
        theta3 = theta3_limit;
        e_1 = z_2;
        e_2 = z_3 - z_2 * dot3f;
        e_2 = e_2/norm(e_2);
        z_3 = cos(theta3) * e_1 + sin(theta3) * e_2;
    end
    
    L3 = L30 + La/2/cos(theta4/2) + La/2/cos(theta3/2);
    P4 = P3 + L3 * z_3;
    if(is_plot)
        set(h4, 'XData', [P3(1) P4(1)]);set(h4, 'YData', [P3(2) P4(2)]);set(h4, 'ZData', [P3(3) P4(3)]);
        pause(0.5);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Joint 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_4 = (position_target - P4)/norm(position_target - P4);
    dot4f = z_3'*z_4;
    if(dot4f>1)
        dot4f = 1;
    end
    theta4 = acos(dot4f);
    if theta4 > theta4_limit
        theta4 = theta4_limit;
        e_1 = z_3;
        e_2 = z_4 - z_3 * dot4f;
        e_2 = e_2/norm(e_2);
        z_4 = cos(theta4) * e_1 + sin(theta4) * e_2;
    end
    
    L4 = L40 + La/2/cos(theta4/2);
    position_current = P4 + L4 * z_4;
    
    if(is_plot)
        set(h5, 'XData', [position_current(1) P4(1)]);
        set(h5, 'YData', [position_current(2) P4(2)]);
        set(h5, 'ZData', [position_current(3) P4(3)]);
        pause(0.5);
    end

    %位置更新
    last_end = cur_end;
    cur_end = position_current;
    
    last_pose = cur_pose;
    cur_pose = [P3;P2;P1];

    %计算位置误差
    error_current=norm(position_current-position_target);  
    error_block(k) = error_current;
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

dotf = z_4'*orientation_target(:,3);
if(dotf > 1)
    dotf = 1;
end
errorR = acos(dotf);
error_PR = [errorP errorR];
THETA = [theta2 theta2 theta3 theta4];
end

function J_cur=BadBroydenJacobian(dp,dx,J_last)

alpha = 1;
J_cur = J_last + alpha*(dp-J_last*dx)/(dx'*dx)*dx';
end