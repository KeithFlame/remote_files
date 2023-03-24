function PS_2segs_keith(S, SL, Tend, discrete_element, init_pose, plot_type)
% Declaration
% the end-effector coordinate {g}.
% the base coordinate {b}
%
% This is a function to calculate the pose of {g} in {b}, first.
% the programme, second, can drawing the shape of the 2-segment continuum robot.
% and, third, it still draws apart from the other arms with the given initial pose 
% in the drawing procedure (max 2 arms now). 
%
% Author: Keith W.
% Ver. 1.0
% Date: 03.24.2022

% Ver. 2.0 
% add trocar and 4 arms, respectively.
% Date 05.02.2022
% Author: Keith W.


if(nargin == 2)
    discrete_element = 1;
    plot_type = 1;
    init_pose = eye(4);
    Tend = eye(4);
end
if(nargin == 3)
    plot_type = 1;
    init_pose = eye(4);
    discrete_element = 1;
end
if(nargin == 4)
    plot_type = 1;
    init_pose = eye(4);
end
if(nargin == 5)
    plot_type = 1;
end

% S(4,:) = S(4,:)*0;
S=init_pose*(S');
S(4,:) = S(4,:)*0;
n2 = ceil(SL(3)/discrete_element);
n1 = ceil(SL(1)/discrete_element);
ng = 1;
nr = 1;
n_size = size(S,2);
n0 = n_size-n1-nr-n2-ng;
if(n0 < 0)
    n1 = n1 + n0 - 1;
    n0 = 1;
end
s1 = S(1:3, 1:n0);
s2 = S(1:3, n0:n0+n1);
s3 = S(1:3, n0+n1:n0+n1+nr);
s4 = S(1:3, n0+n1+nr:n0+n1+nr+n2);
s5 = S(1:3, n0+n1+nr+n2:n0+n1+nr+n2+ng);

    
    if(plot_type == 2)
        axis equal;grid on; hold on;
        radius_L1 = 3;
        radius_L2 = 3;
        radius_Lr = 3;
        number_niti_L1 = 4;
        number_niti_L2 = 16;
        first_niti_L1 = 0;
        first_niti_L2 = 21/180*pi;
        spacer_interval = 10;
    
        % plot lstem  
        if(l>L1)
            P_Lstem = S(:,1);
            P_lsetm_d = S(:,2);
            lefc = [0.2    0.2    0.2];alpha = 0.9;
            plotcylinder(P_lsetm_d,P_Lstem,lefc,radius_Lr,alpha);
        end
        % plot L1 body
        for i = 1:number_niti_L1
            interval_angle=2*pi/number_niti_L1;
            delta_angle=delta1+first_niti_L1+interval_angle*(i-1);
            p_i=s2;
            p0=radius_L1*[cos(delta_angle) sin(delta_angle) 0]';
            for j = 1:max(size(s2)) 
                p_i(1:3,j)= s2(1:3,j)+T1(1:3,1:3)*getContinuumBending(s2(4,j), delta1)*p0;
            end
            plot3(p_i(1,:),p_i(2,:),p_i(3,:),'LineWidth',2,Color=[0.6314    0.5961    0.7647]);
        end
        for i = 1:number_niti_L2
            if(port == 3)
                interval_angle=2*pi/number_niti_L2;
                delta_angle=delta1+first_niti_L2+interval_angle*(i-1);
            else
                interval_angle=16/180*pi;
                delta_angle=delta1+first_niti_L2+interval_angle*(i-1)+26/180*pi*floor(i/4);
            end
            p_i=s2;
            p0=radius_L1*[cos(delta_angle) sin(delta_angle) 0]';
            for j = 1:max(size(s4)) 
                p_i(1:3,j)= s2(1:3,j)+T1(1:3,1:3)*getContinuumBending(s2(4,j), delta1)*p0;
            end
            plot3(p_i(1,:),p_i(2,:),p_i(3,:),'LineWidth', 0.1, Color=[0.6863    0.8510    0.9098]);
        end
        % plot L1 spacer
        dis=L1-spacer_interval;
        j=1;
        i=size(scalar,2);
        while(1) 
            if(dis<spacer_interval)
                break;
            else
                [~,i]=min(abs(scalar(1:i)*L1-scalar(i)*L1+spacer_interval));
                dis=scalar(i)*L1;
                j=j+1;
                if( i == 1 &&scalar(2)*L1>fix(scalar(2)*L1))
                    break;
                end
            end
            p1=s2(1:3,i);
            r=T1(1:3,1:3)*getContinuumBending(s2(4,i), delta1);
            p2=p1+r(:,3);
            plotcylinder(p1,p2,[ 0.9569    0.6784    0.6941],radius_Lr,0.6);
        end
    
        % rigid 
        plotcylinder(s3(1:3,1),s3(1:3,end),[0.4 0.4 0.4],radius_Lr,1);
    
        % plot L2 body
        for i = 1:number_niti_L2
            if(port == 3)
                interval_angle=2*pi/number_niti_L2;
                delta_angle=delta1+first_niti_L2+interval_angle*(i-1);
            else
                interval_angle=16/180*pi;
                delta_angle=delta1+first_niti_L2+interval_angle*(i-1)+26/180*pi*floor(i/4);
            end
            p_i=s4;
            p0=radius_L2*[cos(delta_angle) sin(delta_angle) 0]';
            for j = 1:max(size(s4)) 
                p_i(1:3,j)= s4(1:3,j)+T1(1:3,1:3)*T2(1:3,1:3)*getContinuumBending(s4(4,j), delta2)*p0;
            end
            plot3(p_i(1,:),p_i(2,:),p_i(3,:),'LineWidth',0.5,Color=[0.6863    0.8510    0.9098]);
        end
        
        % plot L2 spacer
        
        if(port == 3)
            alpha =0.5;
            for i =[floor((max(size(scalar))+1)/3) floor((max(size(scalar))+1)/3*2) max(size(scalar))]
                p1=s4(1:3,i);
                r=T1(1:3,1:3)*T2(1:3,1:3)*getContinuumBending(s4(4,i), delta2);
                p2=p1-r(:,3);
                plotcylinder(p1,p2,[0.7451    0.8392    0.5451],radius_Lr,alpha);
            end
        else
            alpha =0.5;
            for i =[(max(size(scalar))+1)/2 max(size(scalar))]
                p1=s4(1:3,i);
                r=T1(1:3,1:3)*T2(1:3,1:3)*getContinuumBending(s4(4,i), delta2);
                p2=p1-r(:,3);
                plotcylinder(p1,p2,[0.7451    0.8392    0.5451],radius_Lr,alpha);
                alpha = alpha +0.5;
            end
        end
        % draw gripper
        color = [0.5 0.5 0.5];
        Lgt = norm(s5(1:3,2)-s5(1:3,1))*1.33;
        drawGripper(init_pose*Tend,radius_Lr,Lgt,color);
        % darw coordinates
        plotCoord_keith(init_pose,1);
        plotCoord_keith(init_pose*Tend,1);
        
        
    end
    if(plot_type == 1)
        axis equal;grid on; hold on;
        plot3(s1(1,:),s1(2,:),s1(3,:),LineWidth=1,Color='black');
        plot3(s2(1,:),s2(2,:),s2(3,:),LineWidth=1,Color='r');
        plot3(s3(1,:),s3(2,:),s3(3,:),LineWidth=1,Color='black');
        plot3(s4(1,:),s4(2,:),s4(3,:),LineWidth=1,Color='#3b4');
        plot3(s5(1,:),s5(2,:),s5(3,:),LineWidth=1,Color='b');
        plotCoord_keith(init_pose,1);
        plotCoord_keith(init_pose*Tend,1);
    end
end

%% 计算每个离散点的位姿
function r=getContinuumBending(theta, delta)
    delta=-delta;
    r1=[0 cos(delta) sin(delta);0 -sin(delta) cos(delta); 1 0 0];
    r2=[cos(theta) -sin(theta) 0;sin(theta) cos(theta) 0;0 0 1];
    r3=[0 0 1; cos(delta) -sin(delta) 0; sin(delta) cos(delta) 0];
    r=r1*r2*r3;
end


%% 画间隔片和刚性段
function hd = plotcylinder(u1,u2,color,r,alpha)
% 根据空间两点画圆柱
% u1,u2 ——空间两点
% color ——颜色
% r     ——半径
% alpha ——透明度
u1 = reshape(u1, [3 1]);
u2 = reshape(u2, [3 1]);
n=u2-u1;
theta=(0:2*pi/100:2*pi)'; %theta角从0到2*pi
a=cross(n,[1 0 0]);       %n与i叉乘，求取a向量
if ~any(a) %如果a为零向量，将n与j叉乘   %any检测矩阵中是否有非零元素，如果有，则返回1，否则，返回0
    a=cross(n,[0 1 0]);
end
b=cross(n,a); %求取b向量
a=a/norm(a);  %单位化a向量
b=b/norm(b);  %单位化b向量
%a,b是满足既垂直于n，又互相垂直的任意单位向量

%底面圆方程
c1=u1(1)*ones(size(theta,1),1);
c2=u1(2)*ones(size(theta,1),1);
c3=u1(3)*ones(size(theta,1),1);
x1=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta); %圆上各点的x坐标
y1=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta); %圆上各点的y坐标
z1=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta); %圆上各点的z坐标

%顶面圆方程
c1=u2(1)*ones(size(theta,1),1);
c2=u2(2)*ones(size(theta,1),1);
c3=u2(3)*ones(size(theta,1),1);
x2=c1+r*a(1)*cos(theta)+r*b(1)*sin(theta); %圆上各点的x坐标
y2=c2+r*a(2)*cos(theta)+r*b(2)*sin(theta); %圆上各点的y坐标
z2=c3+r*a(3)*cos(theta)+r*b(3)*sin(theta); %圆上各点的z坐标

X(1,:)=x1(:);
X(2,:)=x2(:);
Y(1,:)=y1(:);
Y(2,:)=y2(:);
Z(1,:)=z1(:);
Z(2,:)=z2(:);

h1 = fill3(X(1,:),Y(1,:),Z(1,:),color,'EdgeColor','none','FaceAlpha',alpha); %底面
hold on
h2 = fill3(X(2,:),Y(2,:),Z(2,:),color,'EdgeColor','none','FaceAlpha',alpha); %顶面
hold on
h3 = surf(X,Y,Z,'facecolor',color,'edgecolor','none','FaceAlpha',alpha);    %圆柱表面
hd = [h1 h2 h3]';
end

%% 画末端执行器
function [hG]=drawGripper(Tend,rho,Lg,color)
p = Tend(1:3,4);
R = Tend(1:3,1:3);
if(nargin == 4)
    color=[0 0 0];
end
if(Lg == 0)
    hG=[];
    return;
end
Lg=Lg*1.25;
D1=rho*cos(pi/6);
D2=rho*sin(pi/6);
H1=Lg*0.3;
H2=Lg*0.4;
p0=[D1 D2 0;D1 -D2 0;-D1 -D2 0;-D1 D2 0;...
   D1 D2 H1;D1 -D2 H1;-D1 -D2 H1;-D1 D2 H1;...
   0 D2 H2;0 -D2 H2;...
   D1 D2 Lg;D1 -D2 Lg;-D1 -D2 Lg;-D1 D2 Lg]';
p1=p+R*p0;
h1=patch([p1(1,1) p1(1,5) p1(1,9) p1(1,8) p1(1,4)]',[p1(2,1) p1(2,5) p1(2,9) p1(2,8) p1(2,4)]',[p1(3,1) p1(3,5) p1(3,9) p1(3,8) p1(3,4)]',color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h2=patch([p1(1,2) p1(1,6) p1(1,10) p1(1,7) p1(1,3)],[p1(2,2) p1(2,6) p1(2,10) p1(2,7) p1(2,3)],[p1(3,2) p1(3,6) p1(3,10) p1(3,7) p1(3,3)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h3=patch([p1(1,1) p1(1,2) p1(1,6) p1(1,5)],[p1(2,1) p1(2,2) p1(2,6) p1(2,5)],[p1(3,1) p1(3,2) p1(3,6) p1(3,5)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h4=patch([p1(1,4) p1(1,3) p1(1,7) p1(1,8)],[p1(2,4) p1(2,3) p1(2,7) p1(2,8)],[p1(3,4) p1(3,3) p1(3,7) p1(3,8)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h5=patch([p1(1,5) p1(1,6) p1(1,10) p1(1,9)],[p1(2,5) p1(2,6) p1(2,10) p1(2,9)],[p1(3,5) p1(3,6) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h6=patch([p1(1,8) p1(1,7) p1(1,10) p1(1,9)],[p1(2,8) p1(2,7) p1(2,10) p1(2,9)],[p1(3,8) p1(3,7) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h7=patch([p1(1,11) p1(1,12) p1(1,10) p1(1,9)],[p1(2,11) p1(2,12) p1(2,10) p1(2,9)],[p1(3,11) p1(3,12) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h8=patch([p1(1,14) p1(1,13) p1(1,10) p1(1,9)],[p1(2,14) p1(2,13) p1(2,10) p1(2,9)],[p1(3,14) p1(3,13) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
hG=[h1 h2 h3 h4 h5 h6 h7 h8]';
end
