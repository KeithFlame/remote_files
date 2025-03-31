function [flag, time,iteration,error_PR,THETA]=constraints_FABRIK_without_Broyden_RCCCC(target,LS,initial_pose,err_epsilon,kmax,limit)

%部分构型参数
L1=LS(1);
L2=LS(2);
L3=LS(3);
L4=LS(4);
theta_limit = limit;

%输入目标位姿
orientation_target = target(1:3,1:3);
position_target = target(1:3,4);

%迭代参数
error_P_desired=err_epsilon;
k_max=kmax;

%输入当前位姿并初始化虚拟关节位置和虚拟杆长
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
P_4j = [0 0 350]';
deadlock_cter=[1 10 100];

%计算位置误差
error_current=norm(position_current-position_target);
error_block = zeros(k_max,1);
%

%迭代次数初始化
k=0;
tic;
is_plot = 0;
if(is_plot)
    figure;
    hold on;grid on;
    axis equal;
    h1 = plot3([0 0], [0 0], [0 50],'c--');
    h2 = plot3([0 0], [0 0], [50 100],'c--');
    h3 = plot3([0 0], [0 0], [100 150],'r--');
    h4 = plot3([0 0], [0 0], [150 200],'r--');
    h5 = plot3([0 0], [0 0], [200 250],'g--');
    h6 = plot3([0 0], [0 0], [250 300],'g--');
    h7 = plot3([0 0], [0 0], [300 350],'b--');
    h8 = plot3([0 0], [0 0], [350 400],'b--');
    % h3 = plot3([P_2j(1) P_3j(1)], [P_2j(2) P_3j(2)], [P_2j(3) P_3j(3)],'g--');
    % h4 = plot3([P_3j(1) P_4j(1)], [P_3j(2) P_4j(2)], [P_3j(3) P_4j(3)],'b--');
    % h5 = plot3([P_4j(1) position_current(1)], [P_4j(2) position_current(2)], [P_4j(3) position_current(3)],'k--');
    arcPoints1 = getCircleSegment([0 0 0],[0 0.1 100],[0 0.2 100],0.001);
    arcPoints2 = getCircleSegment([0 0.1 100],[0 0.2 200],[0 0.4 100],0.001);
    arcPoints3 = getCircleSegment([0 0.2 200],[0 0.1 300],[0 0.5 100],0.001);
    arcPoints4 = getCircleSegment([0 0.1 300],[0 0 400],[0 1 100],0.001);
    hr1 = plot3(arcPoints1(1, :), arcPoints1(2, :), arcPoints1(3, :), 'c-', 'LineWidth', 2);
    hr2 = plot3(arcPoints2(1, :), arcPoints2(2, :), arcPoints2(3, :), 'r-', 'LineWidth', 2);
    hr3 = plot3(arcPoints3(1, :), arcPoints3(2, :), arcPoints3(3, :), 'g-', 'LineWidth', 2);
    hr4 = plot3(arcPoints4(1, :), arcPoints4(2, :), arcPoints4(3, :), 'b-', 'LineWidth', 2);
    view([-130 31])
end
while k < k_max && error_current > error_P_desired
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
    if theta4 > theta_limit
        theta4 = theta_limit;
        e_1 = z_4e;
        e_2 = z_4b - z_4e * dot4f;
        e_2 = e_2/norm(e_2);
        z_4b = cos(theta4) * e_1 + sin(theta4) * e_2;
    end

    if theta4 == 0
        l4 = L4/2;
    else
        l4 = L4/theta4 * tan(theta4/2);
    end
    
    P_4j = P_4e - l4 * z_4e;
    if(is_plot)
        P_4b = P_4j - l4 * z_4b;
        arcPoints4 = getCircleSegment(P_4b',P_4e',P_4j',theta4);
        set(hr4, 'XData', arcPoints4(1,:));set(hr4, 'YData', arcPoints4(2,:));set(hr4, 'ZData', arcPoints4(3,:));
        set(h7, 'XData', [P_4b(1) P_4j(1)]);set(h7, 'YData', [P_4b(2) P_4j(2)]);set(h7, 'ZData', [P_4b(3) P_4j(3)]);
        set(h8, 'XData', [P_4j(1) P_4e(1)]);set(h8, 'YData', [P_4j(2) P_4e(2)]);set(h8, 'ZData', [P_4j(3) P_4e(3)]);
        pause(0.1);
    end
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
    if theta3 > theta_limit
        theta3 = theta_limit;
        e_1 = z_3e;
        e_2 = z_3b - z_3e * dot3f;
        e_2 = e_2/norm(e_2);
        z_3b = cos(theta3) * e_1 + sin(theta3) * e_2;
    end

    if theta3 == 0
        l3 = L3/2;
    else
        l3 = L3/theta3 * tan(theta3/2);
    end
    
    P_3j = P_3e - l3 * z_3e;
    if(is_plot)
        P_3b = P_3j - l3 * z_3b;
        arcPoints3 = getCircleSegment(P_3b',P_3e',P_3j',theta3);
        set(hr3, 'XData', arcPoints3(1,:));set(hr3, 'YData', arcPoints3(2,:));set(hr3, 'ZData', arcPoints3(3,:));
        set(h5, 'XData', [P_3b(1) P_3j(1)]);set(h5, 'YData', [P_3b(2) P_3j(2)]);set(h5, 'ZData', [P_3b(3) P_3j(3)]);
        set(h6, 'XData', [P_3j(1) P_3e(1)]);set(h6, 'YData', [P_3j(2) P_3e(2)]);set(h6, 'ZData', [P_3j(3) P_3e(3)]);
        pause(0.1);
    end
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
    if theta2 > theta_limit
        theta2 = theta_limit;
        e_1 = z_2e;
        e_2 = z_2b - z_2e * dot2f;
        e_2 = e_2/norm(e_2);
        z_2b = cos(theta2) * e_1 + sin(theta2) * e_2;
    end

    if theta2 == 0
        l2 = L2/2;
    else
        l2 = L2/theta2 * tan(theta2/2);
    end
    
    P_2j = P_2e - l2 * z_2e;
    if(is_plot)
        P_2b = P_2j - l2 * z_2b;
        arcPoints2 = getCircleSegment(P_2b',P_2e',P_2j',theta2);
        set(hr2, 'XData', arcPoints2(1,:));set(hr2, 'YData', arcPoints2(2,:));set(hr2, 'ZData', arcPoints2(3,:));
        set(h3, 'XData', [P_2b(1) P_2j(1)]);set(h3, 'YData', [P_2b(2) P_2j(2)]);set(h3, 'ZData', [P_2b(3) P_2j(3)]);
        set(h4, 'XData', [P_2j(1) P_2e(1)]);set(h4, 'YData', [P_2j(2) P_2e(2)]);set(h4, 'ZData', [P_2j(3) P_2e(3)]);
        pause(0.1);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%CONTINUUM 1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    z_1e = z_2b;
    % P_1e = P_2j - l2 * z_2b;
    % P_1j = P_1e - l1 * z_3b;
    z_1b = [0 0 1]';
    
    dot1f = z_1b'*z_1e;
    if(dot1f>1)
        dot1f = 1;
    end
    theta1 = acos(dot1f);
    if theta1 > theta_limit
        theta1 = theta_limit;
        % e_1 = z_1e;
        % e_2 = z_1b - z_1e * dot1f;
        % e_2 = e_2/norm(e_2);
        % z_1b = cos(theta1) * e_1 + sin(theta1) * e_2;
    end

    if theta1 == 0
        l1 = L1/2;
    else
        l1 = L1/theta1 * tan(theta1/2);
    end
    
    if(is_plot)
        P_1e = P_2j - l2 * z_2b;
        P_1j = P_1e - l1 * z_3b;
        P_1b = P_1j - l1 * z_1b;
        arcPoints1 = getCircleSegment(P_1b',P_1e',P_1j',theta1);
        set(hr1, 'XData', arcPoints1(1,:));set(hr1, 'YData', arcPoints1(2,:));set(hr1, 'ZData', arcPoints1(3,:));
        set(h1, 'XData', [P_1b(1) P_1j(1)]);set(h1, 'YData', [P_1b(2) P_1j(2)]);set(h1, 'ZData', [P_1b(3) P_1j(3)]);
        set(h2, 'XData', [P_1j(1) P_1e(1)]);set(h2, 'YData', [P_1j(2) P_1e(2)]);set(h2, 'ZData', [P_1j(3) P_1e(3)]);
        pause(0.1);
    end
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
    if(is_plot)
        P_1e = P_1j + l1 * z_2b;
        arcPoints1 = getCircleSegment(P_1b',P_1e',P_1j',theta1);
        set(hr1, 'XData', arcPoints1(1,:));set(hr1, 'YData', arcPoints1(2,:));set(hr1, 'ZData', arcPoints1(3,:));
        set(h1, 'XData', [P_1b(1) P_1j(1)]);set(h1, 'YData', [P_1b(2) P_1j(2)]);set(h1, 'ZData', [P_1b(3) P_1j(3)]);
        set(h2, 'XData', [P_1j(1) P_1e(1)]);set(h2, 'YData', [P_1j(2) P_1e(2)]);set(h2, 'ZData', [P_1j(3) P_1e(3)]);
        pause(0.1);
    end
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
        e_2 = z_2e - z_2b * dot2f;
        e_2 = e_2/norm(e_2);
        z_2e = cos(theta2) * e_1 + sin(theta2) * e_2;
    end
    
    if theta2 == 0
        l2 = L2/2;
    else
        l2 = L2/theta2 * tan(theta2/2);
    end
    
    P_2j = P_2b + l2 * z_2b;
    if(is_plot)
        P_2e = P_2j + l2 * z_2e;
        arcPoints2 = getCircleSegment(P_2b',P_2e',P_2j',theta2);
        set(hr2, 'XData', arcPoints2(1,:));set(hr2, 'YData', arcPoints2(2,:));set(hr2, 'ZData', arcPoints2(3,:));
        set(h3, 'XData', [P_2b(1) P_2j(1)]);set(h3, 'YData', [P_2b(2) P_2j(2)]);set(h3, 'ZData', [P_2b(3) P_2j(3)]);
        set(h4, 'XData', [P_2j(1) P_2e(1)]);set(h4, 'YData', [P_2j(2) P_2e(2)]);set(h4, 'ZData', [P_2j(3) P_2e(3)]);
        pause(0.1);
    end
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
        e_2 = z_3e - z_3b * dot3f;
        e_2 = e_2/norm(e_2);
        z_3e = cos(theta3) * e_1 + sin(theta3) * e_2;
    end

    if theta3 == 0
        l3 = L3/2;
    else
        l3 = L3/theta3 * tan(theta3/2);
    end
       
    P_3j = P_3b + l3 * z_3b;
    if(is_plot)
        P_3e = P_3j + l3 * z_3e;
        arcPoints3 = getCircleSegment(P_3b',P_3e',P_3j',theta3);
        set(hr3, 'XData', arcPoints3(1,:));set(hr3, 'YData', arcPoints3(2,:));set(hr3, 'ZData', arcPoints3(3,:));
        set(h5, 'XData', [P_3b(1) P_3j(1)]);set(h5, 'YData', [P_3b(2) P_3j(2)]);set(h5, 'ZData', [P_3b(3) P_3j(3)]);
        set(h6, 'XData', [P_3j(1) P_3e(1)]);set(h6, 'YData', [P_3j(2) P_3e(2)]);set(h6, 'ZData', [P_3j(3) P_3e(3)]);
        pause(0.1);
    end
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
    
    if(is_plot)
        P_4e = position_current;
        arcPoints4 = getCircleSegment(P_4b',P_4e',P_4j',theta4);
        set(hr4, 'XData', arcPoints4(1,:));set(hr4, 'YData', arcPoints4(2,:));set(hr4, 'ZData', arcPoints4(3,:));
        set(h7, 'XData', [P_4b(1) P_4j(1)]);set(h7, 'YData', [P_4b(2) P_4j(2)]);set(h7, 'ZData', [P_4b(3) P_4j(3)]);
        set(h8, 'XData', [P_4j(1) P_4e(1)]);set(h8, 'YData', [P_4j(2) P_4e(2)]);set(h8, 'ZData', [P_4j(3) P_4e(3)]);
        pause(0.1);
    end

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

dotf = z_4e'*orientation_target(:,3);
if(dotf > 1)
    dotf = 1;
end
errorR = acos(dotf);
error_PR = [errorP errorR];
THETA = [theta1 theta2 theta3 theta4];
end