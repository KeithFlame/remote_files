function plot_delta_robot(psi,target)
%	绘制delta机器人

% 导轨长度
points_can_not_reach = [0 190;0 0;0 0];
points_can_reach = [190 490;0 0;0 0];

%导轨角度参数
alpha = -70/180*pi;
beta = 120/180*pi;

%连续体长度参数
global L D;

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
%连续体支链变量
q1 = psi(1);
xita1 = psi(2);
delta1 = psi(3);

q2 = psi(4);
xita2 = psi(5);
delta2 = psi(6);

q3 = psi(7);
xita3 = psi(8);
delta3 = psi(9);

% 连续体支链1
[proximal_1,distal_1] = continuum_points(xita1,delta1,q1,1);
% 连续体支链2
[proximal_2,distal_2] = continuum_points(xita2,delta2,q2,2);
% 连续体支链3
[proximal_3,distal_3] = continuum_points(xita3,delta3,q3,3);


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
plot3([proximal_1(1,end);distal_1(1,end)],[proximal_1(2,end);distal_1(2,end)],[proximal_1(3,end);distal_1(3,end)],'k','linewidth',1);

%绘制连续体支链2
plot3(proximal_2(1,:),proximal_2(2,:),proximal_2(3,:),'g','linewidth',1);
plot3(distal_2(1,:),distal_2(2,:),distal_2(3,:),'g','linewidth',1);
plot3([proximal_2(1,end);distal_2(1,end)],[proximal_2(2,end);distal_2(2,end)],[proximal_2(3,end);distal_2(3,end)],'k','linewidth',1);

%绘制连续体支链3
plot3(proximal_3(1,:),proximal_3(2,:),proximal_3(3,:),'b','linewidth',1);
plot3(distal_3(1,:),distal_3(2,:),distal_3(3,:),'b','linewidth',1);
plot3([proximal_3(1,end);distal_3(1,end)],[proximal_3(2,end);distal_3(2,end)],[proximal_3(3,end);distal_3(3,end)],'k','linewidth',1);

%画目标位置
plot_orientation(target,eye(3),30);
circle(target,eye(3),20);

grid on;
box on;
title('delta robot');
axis equal;
axis([-500 500  -500 500  -400 500]);
drawnow;
hold off;

end

