function FABRIKc_process_draw()
% FABRIKc delta process 绘制FABRIKc delta算法流程 
%% 点
global root target_2 target_3;
global P_1_proximal_base P_1_proximal_joint P_1_proximal_end P_1_distal_base P_1_distal_joint P_1_distal_end;
global P_2_proximal_base P_2_proximal_joint P_2_proximal_end P_2_distal_base P_2_distal_joint P_2_distal_end;
global P_3_proximal_base P_3_proximal_joint P_3_proximal_end P_3_distal_base P_3_distal_joint P_3_distal_end;
%% 结构参数
% 导轨长度
points_can_not_reach = [0 190;0 0;0 0];
points_can_reach = [190 490;0 0;0 0];

%导轨角度参数
alpha = -70/180*pi;
beta = 120/180*pi;

%连续体长度参数
L = 60;
D = 500 - 2 * L;
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

%% 绘制

%绘制三根导轨
plot3(x_n_1,y_n_1,z_n_1,'r-.','linewidth',3);
hold on;
plot3(x_1,y_1,z_1,'r','linewidth',3);

plot3(x_n_2,y_n_2,z_n_2,'g-.','linewidth',3);
plot3(x_2,y_2,z_2,'g','linewidth',3);

plot3(x_n_3,y_n_3,z_n_3,'b-.','linewidth',3);
plot3(x_3,y_3,z_3,'b','linewidth',3);

%绘制root和target
root_point = [root root+[0;0;10]];
target_2_point = [target_2 target_2+[0;0;10]];
target_3_point = [target_3 target_3+[0;0;10]];
plot3(root_point(1,1),root_point(2,1),root_point(3,1),'x','markersize',10,'color','r','linewidth',2);
plot3(target_2_point(1,1),target_2_point(2,1),target_2_point(3,1),'x','markersize',10,'color','g','linewidth',2);
plot3(target_3_point(1,1),target_3_point(2,1),target_3_point(3,1),'x','markersize',10,'color','b','linewidth',2);

%绘制虚拟关节和虚拟连杆
link_proximal_1 = [P_1_proximal_base P_1_proximal_joint P_1_proximal_end];
link_proximal_2 = [P_2_proximal_base P_2_proximal_joint P_2_proximal_end];
link_proximal_3 = [P_3_proximal_base P_3_proximal_joint P_3_proximal_end];

link_distal_1 = [P_1_distal_base P_1_distal_joint P_1_distal_end];
link_distal_2 = [P_2_distal_base P_2_distal_joint P_2_distal_end];
link_distal_3 = [P_3_distal_base P_3_distal_joint P_3_distal_end];

rigid_segment_1 = [P_1_proximal_end P_1_distal_base];
rigid_segment_2 = [P_2_proximal_end P_2_distal_base];
rigid_segment_3 = [P_3_proximal_end P_3_distal_base];

plot3(link_proximal_1(1,:),link_proximal_1(2,:),link_proximal_1(3,:),'--','color','r','linewidth',1);
plot3(rigid_segment_1(1,:),rigid_segment_1(2,:),rigid_segment_1(3,:),'color','black','linewidth',1);
plot3(link_distal_1(1,:),link_distal_1(2,:),link_distal_1(3,:),'--','color','r','linewidth',1);
plot3(P_1_proximal_joint(1,:),P_1_proximal_joint(2,:),P_1_proximal_joint(3,:),'o','markersize',5,'color','r','linewidth',0.5);
plot3(P_1_distal_joint(1,:),P_1_distal_joint(2,:),P_1_distal_joint(3,:),'o','markersize',5,'color','r','linewidth',0.5);

plot3(link_proximal_2(1,:),link_proximal_2(2,:),link_proximal_2(3,:),'--','color','g','linewidth',1);
plot3(rigid_segment_2(1,:),rigid_segment_2(2,:),rigid_segment_2(3,:),'color','black','linewidth',1);
plot3(link_distal_2(1,:),link_distal_2(2,:),link_distal_2(3,:),'--','color','g','linewidth',1);
plot3(P_2_proximal_joint(1,:),P_2_proximal_joint(2,:),P_2_proximal_joint(3,:),'o','markersize',5,'color','g','linewidth',0.5);
plot3(P_2_distal_joint(1,:),P_2_distal_joint(2,:),P_2_distal_joint(3,:),'o','markersize',5,'color','g','linewidth',0.5);

plot3(link_proximal_3(1,:),link_proximal_3(2,:),link_proximal_3(3,:),'--','color','b','linewidth',1);
plot3(rigid_segment_3(1,:),rigid_segment_3(2,:),rigid_segment_3(3,:),'color','black','linewidth',1);
plot3(link_distal_3(1,:),link_distal_3(2,:),link_distal_3(3,:),'--','color','b','linewidth',1);
plot3(P_3_proximal_joint(1,:),P_3_proximal_joint(2,:),P_3_proximal_joint(3,:),'o','markersize',5,'color','b','linewidth',0.5);
plot3(P_3_distal_joint(1,:),P_3_distal_joint(2,:),P_3_distal_joint(3,:),'o','markersize',5,'color','b','linewidth',0.5);
% 画动平台
moving_platform_2 = [-30;10*sqrt(3);0];
moving_platform_3 = [-30;-10*sqrt(3);0];
moving_platform_point = [P_1_distal_end P_1_distal_end+moving_platform_2 P_1_distal_end+moving_platform_3 P_1_distal_end];
plot3(moving_platform_point(1,:),moving_platform_point(2,:),moving_platform_point(3,:),'color','black','linewidth',1);

% 画连续体段
delta_1 = -atan2(P_1_proximal_base(2)-P_1_distal_base(2),P_1_proximal_base(1)-P_1_distal_base(1));
delta_2 = -atan2(P_2_proximal_base(2)-P_2_distal_base(2),P_2_proximal_base(1)-P_2_distal_base(1));
delta_3 = -atan2(P_3_proximal_base(2)-P_3_distal_base(2),P_3_proximal_base(1)-P_3_distal_base(1));
global xita_1 xita_2 xita_3;
global q_1 q_2 q_3;
% 连续体支链1
[proximal_1,distal_1] = continuum_points_new(xita_1,delta_1,q_1,1);
% 连续体支链2
[proximal_2,distal_2] = continuum_points_new(xita_2,delta_2,q_2,2);
% 连续体支链3
[proximal_3,distal_3] = continuum_points_new(xita_3,delta_3,q_3,3);

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



axis equal;
box on;
axis([-400 200  -200 400  -400 500]);
drawnow;
% hold off;


end

