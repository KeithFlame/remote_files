clear;close;
clc;
%部分构型参数
Lg=20;
L10=40;
Lr0=20;
L20=60;
%

%迭代参数
error_P_desired=0.001;
error_angle_desired=0.001;
k_max=2000;
%

%输入目标位姿
config_target=4;
figure(1);
psi_target = [1.01243460441139;113.716864699113;1.36833815053487;2.20399528956732;1.43578263091649;1.84819037381710];
[position_target,orientation_target] = plot_manipulator(psi_target);
%
close;
figure;
cla;
hold on;

axis equal;
xlabel("x (mm)");ylabel("y (mm)");zlabel("z (mm)");
view([-54 11]);
title("FM-FABRIKc");
set(gca, 'FontSize', 20);
set(gca,'FontName','Times New Roman');
color = ['k','k','k'];
%输入当前位姿并初始化虚拟关节位置和虚拟杆长
config=4;
psi=[0;10;.50;0;1.5;-pi];
[position_current,~,z_1b,z_1e,z_2b,z_2e,l1,l2,P_1j,P_2j] = plot_manipulator(psi);
hold on;
plot_orientation(position_target,orientation_target,40,color);
%
% position_target = position_target + 20 * orientation_target(:,3);
%计算位置误差
[error_P,~]=error_position(position_current,position_target);
%
axis([-50 100 -120 30 0 150])
grid off;
%迭代次数初始化
k=1;

%

xita_one(1)=psi(3);
xita_two(1)=psi(5);

JBro = eye(3);%[eye(3) eye(3) eye(3)]';
cur_tar = zeros(3,1);
last_tar = cur_tar;
cur_pos = zeros(9,1);
last_pos = cur_pos;
is_first_broyden = 0;
is_broyden = 1;
tic;
while k<k_max && error_P>error_P_desired
    k_count(k) = k-1;
    error(k) = error_P;

    %Broyden phase
    %%%%%%%WHOLE%%%%%%%%%%%%%%
    if(is_broyden==1&&k>9)
            dp = cur_pos-last_pos;
            dx = cur_tar-last_tar;
            JBro=BadBroydenJacobian(dp(1:3),dx,JBro);
            if(k>14)
                dp = JBro*(position_target-cur_tar);
                % cur_pos = [P_1j;P_2j;P_1b];
                P_1j = P_1j + dp(1:3);
                % P_2j = P_2j + cur_pos(4:6);
                % P_1b = P_1b + dp(7:9);
            end
    end

    %forward reaching phase
    %%%%%%%CONTINUUM2%%%%%%%%%%%%%%
    z_2e = orientation_target(:,3);
    P_2e = position_target - Lg * z_2e;
    
    P_2j = P_2e - l2 * z_2e;
    
    z_2b = (P_2j - P_1j)/norm(P_2j - P_1j);
    
    xita2 = acos(z_2b'*z_2e);
    
    if xita2 == 0
        l2 = L20/2;
    else
        l2 = L20/xita2 * tan(xita2/2);
    end
    
    P_2j = P_2e - l2 * z_2e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%CONTINUUM1%%%%%%%%%%%%%%
    z_1e = z_2b;
    P_1e = P_2j - l2 * z_2b - Lr0 * z_2b;
     
    z_1b = [0;0;1];
    
    xita1 = acos(z_1b'*z_1e);
    
    if xita1 == 0
        l1 = L10/2;
    else
        l1 = L10/xita1 * tan(xita1/2);
    end
    
    P_1j = P_1e - l1 * z_1e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%STEM%%%%%%%%%%%%%%%
    P_STEM = P_1j - l1 * z_1b;
    Ls = P_STEM(3);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    
    %backward reaching phase
    %%%%%%%%%%%%STEM%%%%%%%%%%%%%%%
    P_STEM = [0;0;Ls];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%CONTINUUM1%%%%%%%%%%%%%%
    z_1b = [0;0;1];
    P_1b = P_STEM;
    
    P_1j = P_1b + l1 * z_1b;
    
    z_1e = (P_2j - P_1j)/norm(P_2j - P_1j);
    
    xita1 = acos(z_1b'*z_1e);
    
    if xita1 == 0
        l1 = L10/2;
    else
        l1 = L10/xita1 * tan(xita1/2);
    end
    
    P_1j = P_1b + l1 * z_1b;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%CONTINUUM2%%%%%%%%%%%%%%
    z_2b = z_1e;
    P_2b = P_1j + l1 * z_1e + Lr0 * z_1e; 
    
    z_2e = orientation_target(:,3);
    
    xita2 = acos(z_2b'*z_2e);
    
    if xita2 == 0
        l2 = L20/2;
    else
        l2 = L20/xita2 * tan(xita2/2);
    end
    
    P_2j = P_2b + l2 * z_2b;
    position_current = P_2j + l2 * z_2e + Lg * z_2e;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    k=k+1;
    xita_one(k)=xita1;
    xita_two(k)=xita2;



    %%
    last_tar = cur_tar;
    cur_tar = position_current;
    
    last_pos = cur_pos;
    cur_pos = [P_1j;P_2j;P_1b];
    
    %%
    %计算位置误差
    [error_P,~]=error_position(position_current,position_target);
    %
    
    phi = 0;
    
    R_w_1b = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
    P_1b_1e = R_w_1b\(P_1e-P_1b);
    delta1 = atan2(-P_1b_1e(2),P_1b_1e(1));
    R_1b_1e = continuum_oriention(xita1,delta1);
    P_2b_2e = (R_w_1b*R_1b_1e) \ (P_2e - P_2b);
    delta2 = atan2(-P_2b_2e(2),P_2b_2e(1));
    R_2b_2e = continuum_oriention(xita2,delta2);
    
    psi = [phi;Ls;xita1;delta1;xita2;delta2];
    % if(k==4)
    %     cla;
    % plot_orientation(position_target,orientation_target,40,color);
    % [~,~] = plot_manipulator(psi);
    % grid off;
    % axis([-50 100 -120 30 0 150])
    % 
    % end
%     Ls,
%     pause(0.1);
end
toc;
        cla;
    plot_orientation(position_target,orientation_target,40,color);
    [~,~] = plot_manipulator(psi_target);
    grid off;
    axis([-50 100 -120 30 0 150])
    k_count(k) = k-1;
    error(k) = error_P;

if k < 2000
    phi = 0;
    
    R_w_1b = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
    P_1b_1e = R_w_1b\(P_1e-P_1b);
    delta1 = atan2(-P_1b_1e(2),P_1b_1e(1));
    R_1b_1e = continuum_oriention(xita1,delta1);
    P_2b_2e = (R_w_1b*R_1b_1e) \ (P_2e - P_2b);
    delta2 = atan2(-P_2b_2e(2),P_2b_2e(1));
    R_2b_2e = continuum_oriention(xita2,delta2);
    
    orientation_current = R_w_1b*R_1b_1e*R_2b_2e;
    [error_angle,~] = error_orientation(orientation_current,orientation_target);
    %遍历phi
    while error_angle > error_angle_desired
        phi = phi + 0.01;
    
        R_w_1b = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
        P_1b_1e = R_w_1b\(P_1e-P_1b);
        delta1 = atan2(-P_1b_1e(2),P_1b_1e(1));
        R_1b_1e = continuum_oriention(xita1,delta1);
        P_2b_2e = (R_w_1b*R_1b_1e) \ (P_2e - P_2b);
        delta2 = atan2(-P_2b_2e(2),P_2b_2e(1));
        R_2b_2e = continuum_oriention(xita2,delta2);
    
        orientation_current = R_w_1b*R_1b_1e*R_2b_2e;
    
        [error_angle,~] = error_orientation(orientation_current,orientation_target);
    end
    
    psi_end = [phi;Ls;xita1;delta1;xita2;delta2];
    %
    
    plot_manipulator(psi_end);
    
    %误差曲线
    figure(2);
    subplot(2,1,1);
    plot(k_count,error);
    hold on;
    xlabel('k');
    ylabel('Position error');
    grid;
    hold off;
    %
    subplot(2,1,2);
    plot(k_count,xita_one,'b');
    hold on;
    plot(k_count,xita_two,'r');
    xlabel('k');
    ylabel('xita');
    legend('xita1','xita2');
    grid;
    
else
    disp("failed");
end
hold off;

%%

% 生成原始数据
x = 1:1:length(error); % 原始数据点的 x 值
y = error; % 原始数据点的 y 值

% 扩展后的数据点的 x 值
x1 = 1:(length(error)+4);

% 使用 interp1 函数进行插值
y_extended = interp1(x, y, x1, 'spline'); % 使用样条插值

font_size = 25;
% x1=1:length(error);
tt=[108.124543171806	17.3715396476227	11.1702514860405	8.13843742085229	6.34812778225646	5.17118971118005	4.34037812369754	3.72348928096015	3.24784457193733	2.87024959589291	2.56344793265481	2.30940384425273	2.09571365339142	1.91357000104222	1.75654853562705	0.500748219134441	0.177815093341933	0.0401922042101324	0.00722629929756821	0.00104139773520091	0.000139961215506068];
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








