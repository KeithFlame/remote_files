function [Tend,S]=forwardKinematicsandPlotSnake_keith(psi, arm_serial, is_plot)
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

if(nargin == 0)
    psi=[0 0 0 0 0 0 0]';
    is_plot=0;
    arm_serial=getArmPara;
end
if(nargin == 1)
    is_plot=0;
    arm_serial=getArmPara;
end
if(nargin == 2)
    is_plot=0;
end
if(max(size(psi)) == 6)
    beta = 0;
else
    beta = psi(7);
end


SP = getStructurePara_keith;
gamma1=arm_serial.size_para(6);
L1=arm_serial.size_para(1);
Lr=arm_serial.size_para(2);
L2=arm_serial.size_para(3);
Lg=arm_serial.size_para(4);
gamma3=arm_serial.size_para(7);
port = arm_serial.port;
ZETA=arm_serial.stiffness_para(3);


 
scalar=0:0.05:1;
PHI=psi(2);
l=psi(1);
theta1=psi(3);
delta1=psi(4);
theta2=psi(5);
delta2=psi(6);

% effector
effector = arm_serial.effector;

% trocar
init_pose = SP.init_pose(:,:,arm_serial.port);




if(l<L1)
    L1=l;
    Ls=0;
else
    Ls=l-L1;
end

k2=theta2/L2;
k1=theta1/L1;
theta0=0;

%segment1 circular
if theta1==0 || Ls<0.0001 ||ZETA==0
    T1=init_pose*[cos(PHI+gamma1) -sin(PHI+gamma1) 0 0
    sin(PHI+gamma1) cos(PHI+gamma1) 0 0
    0 0 1 Ls
    0 0 0 1];
    s1=init_pose*[0 0;0 0;0 Ls;1 1];s1(4,:)=[0 0];
else
    theta0=theta1*ZETA*Ls/(ZETA*Ls+L1);
    theta1=theta1*L1/(ZETA*Ls+L1);
    k0=theta0/Ls;
    k1=theta1/L1;
    cosTHETA0=cos(theta0);sinTHETA0=sin(theta0);cosDELTA0=cos(delta1);sinDELTA0=sin(delta1);
    T1=init_pose*[cos(PHI+gamma1) -sin(PHI+gamma1) 0 0
    sin(PHI+gamma1) cos(PHI+gamma1) 0 0
    0 0 1 0
    0 0 0 1]*...
    [(cosDELTA0)^2*(cosTHETA0-1)+1 sinDELTA0*cosDELTA0*(cosTHETA0-1) cosDELTA0*sinTHETA0 cosDELTA0*(1-cosTHETA0)/k0
    sinDELTA0*cosDELTA0*(cosTHETA0-1) (cosDELTA0)^2*(1-cosTHETA0)+cosTHETA0 sinDELTA0*sinTHETA0 sinDELTA0*(1-cosTHETA0)/k0
    -cosDELTA0*sinTHETA0 -sinDELTA0*sinTHETA0 cosTHETA0 sinTHETA0/k0
    0 0 0 1];
    s1=init_pose*[cos(PHI+gamma1) -sin(PHI+gamma1) 0 0
        sin(PHI+gamma1) cos(PHI+gamma1) 0 0
        0 0 1 0
        0 0 0 1]*...
    [cosDELTA0*(1-cos(scalar*theta0))/k0;
        sinDELTA0*(1-cos(scalar*theta0))/k0;
        sin(scalar*theta0)/k0;
        ones(1,length(scalar))];s1(4,:)=scalar*theta0;
end
if theta1==0
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 L1
        0 0 0 1];
    s2=T1*[0*scalar
        0*scalar
        L1*scalar
        ones(1,length(scalar))];s2(4,:)=zeros(1,length(scalar));%Segment1 straight
else
    cosTHETA1=cos(theta1);sinTHETA1=sin(theta1);cosDELTA1=cos(delta1);sinDELTA1=sin(delta1);
    T2=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
        0 0 0 1];
    s2=T1*[cosDELTA1*(1-cos(scalar*theta1))/k1;
        sinDELTA1*(1-cos(scalar*theta1))/k1;
        sin(scalar*theta1)/k1;
        ones(1,length(scalar))];s2(4,:)=scalar*theta1;%Segment1 bent
end

%Lr
T3=[1 0 0 0
    0 1 0 0
    0 0 1 Lr
    0 0 0 1];
s3=T1*T2*[0*scalar;0*scalar;Lr*scalar;ones(1,length(scalar))];s3(4,:)=0*scalar;

%segment2 circular
if theta2==0
    T4=[1 0 0 0
        0 1 0 0
        0 0 1 L2
        0 0 0 1];
    s4=T1*T2*T3*[0*scalar
        0*scalar
        L2*scalar
        ones(1,length(scalar))];s4(4,:)=zeros(1,length(scalar));
else
    cosTHETA2=cos(theta2);sinTHETA2=sin(theta2);cosDELTA2=cos(delta2);sinDELTA2=sin(delta2);
    T4=[(cosDELTA2)^2*(cosTHETA2-1)+1 sinDELTA2*cosDELTA2*(cosTHETA2-1) cosDELTA2*sinTHETA2 cosDELTA2*(1-cosTHETA2)/k2
        sinDELTA2*cosDELTA2*(cosTHETA2-1) (cosDELTA2)^2*(1-cosTHETA2)+cosTHETA2 sinDELTA2*sinTHETA2 sinDELTA2*(1-cosTHETA2)/k2
        -cosDELTA2*sinTHETA2 -sinDELTA2*sinTHETA2 cosTHETA2 sinTHETA2/k2
        0 0 0 1];
    s4=T1*T2*T3*[cosDELTA2*(1-cos(scalar*theta2))/k2;
        sinDELTA2*(1-cos(scalar*theta2))/k2;
        sin(scalar*theta2)/k2;
        ones(1,length(scalar))];s4(4,:)=scalar*theta2;%Segment2 bent
end

%end effector
T5=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 Lg
    0 0 0 1];
s5=T1*T2*T3*T4*[0 0;0 0;0 Lg;1 1];s5(4,:)=[0 0];
Tend=zeros(4,4,5);
Tend(:,:,1)=T1;
Tend(:,:,2)=Tend(:,:,1)*T2;
Tend(:,:,3)=Tend(:,:,2)*T3;
Tend(:,:,4)=Tend(:,:,3)*T4;
Tend(:,:,5)=Tend(:,:,4)*T5;
% Tend=[T1,T1*T2,T1*T2*T3,T1*T2*T3*T4,T1*T2*T3*T4*T5];%R=T([1 2 3],[1 2 3]);p=T([1 2 3],4);
S=[s1 s2 s3 s4 s5];



if(is_plot>0)
%     figure;
    axis equal;grid on; hold on;
%     xlabel("X");
%     ylabel("Y");
%     zlabel("Z");
    assembly_para=arm_serial.assembly_para;
    radius_L1 = assembly_para(1)/2;
    radius_L2 = assembly_para(3)/2;
    radius_Lr = assembly_para(2)/2;
%     radius_Lg = radius_Lr;
    number_niti_L1 = assembly_para(4);
    number_niti_L2 = assembly_para(5);
    first_niti_L1 = assembly_para(6);
    first_niti_L2 = assembly_para(7);
    spacer_interval = assembly_para(8);

    % plot lstem
    lefc = [0.2    0.2    0.2];alpha = 0.9; 
    plotTorus(Ls, radius_Lr,theta0,delta1+PHI+gamma1,lefc, init_pose);
    if(l>L1)
        P_Lstem = T1(1:3,4);
        P_lsetm_d = P_Lstem + T1(1:3,3)*0.1;
        % lstem end face color
             
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
    plotEffector_keith(T1*T2*T3*T4, effector,beta,color);

    % darw coordinates
    plotCoord(init_pose,1);
    plotCoord(Tend(:,:,5),1);
    
    
end
if(is_plot < 0)
    axis equal;grid on; hold on;
%     xlabel("X");
%     ylabel("Y");
%     zlabel("Z");
    plot3(s1(1,:),s1(2,:),s1(3,:),LineWidth=1,Color='black');
    plot3(s2(1,:),s2(2,:),s2(3,:),LineWidth=1,Color='r');
    plot3(s3(1,:),s3(2,:),s3(3,:),LineWidth=1,Color='black');
    plot3(s4(1,:),s4(2,:),s4(3,:),LineWidth=1,Color='#3b4');
    plot3(s5(1,:),s5(2,:),s5(3,:),LineWidth=1,Color='b');
    plotCoord(init_pose,1);
    plotCoord(Tend(:,:,5),1);
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
function plotcylinder(u1,u2,color,r,alpha)
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

fill3(X(1,:),Y(1,:),Z(1,:),color,'EdgeColor','none','FaceAlpha',alpha) %底面
hold on
fill3(X(2,:),Y(2,:),Z(2,:),color,'EdgeColor','none','FaceAlpha',alpha) %顶面
hold on
surf(X,Y,Z,'facecolor',color,'edgecolor','none','FaceAlpha',alpha)    %圆柱表面
end

%% 画末端执行器
% % % % % % % % function [hG]=drawGripper(p,R,rho,Lg,color)
% % % % % % % % if(nargin == 4)
% % % % % % % %     color=[0 0 0];
% % % % % % % % end
% % % % % % % % if(Lg == 0)
% % % % % % % %     hG=[];
% % % % % % % %     return;
% % % % % % % % end
% % % % % % % % Lg=Lg*1.25;
% % % % % % % % D1=rho*cos(pi/6);
% % % % % % % % D2=rho*sin(pi/6);
% % % % % % % % H1=Lg*0.3;
% % % % % % % % H2=Lg*0.4;
% % % % % % % % p0=[D1 D2 0;D1 -D2 0;-D1 -D2 0;-D1 D2 0;...
% % % % % % % %    D1 D2 H1;D1 -D2 H1;-D1 -D2 H1;-D1 D2 H1;...
% % % % % % % %    0 D2 H2;0 -D2 H2;...
% % % % % % % %    D1 D2 Lg;D1 -D2 Lg;-D1 -D2 Lg;-D1 D2 Lg]';
% % % % % % % % p1=p+R*p0;
% % % % % % % % h1=patch([p1(1,1) p1(1,5) p1(1,9) p1(1,8) p1(1,4)]',[p1(2,1) p1(2,5) p1(2,9) p1(2,8) p1(2,4)]',[p1(3,1) p1(3,5) p1(3,9) p1(3,8) p1(3,4)]',color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h2=patch([p1(1,2) p1(1,6) p1(1,10) p1(1,7) p1(1,3)],[p1(2,2) p1(2,6) p1(2,10) p1(2,7) p1(2,3)],[p1(3,2) p1(3,6) p1(3,10) p1(3,7) p1(3,3)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h3=patch([p1(1,1) p1(1,2) p1(1,6) p1(1,5)],[p1(2,1) p1(2,2) p1(2,6) p1(2,5)],[p1(3,1) p1(3,2) p1(3,6) p1(3,5)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h4=patch([p1(1,4) p1(1,3) p1(1,7) p1(1,8)],[p1(2,4) p1(2,3) p1(2,7) p1(2,8)],[p1(3,4) p1(3,3) p1(3,7) p1(3,8)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h5=patch([p1(1,5) p1(1,6) p1(1,10) p1(1,9)],[p1(2,5) p1(2,6) p1(2,10) p1(2,9)],[p1(3,5) p1(3,6) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h6=patch([p1(1,8) p1(1,7) p1(1,10) p1(1,9)],[p1(2,8) p1(2,7) p1(2,10) p1(2,9)],[p1(3,8) p1(3,7) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h7=patch([p1(1,11) p1(1,12) p1(1,10) p1(1,9)],[p1(2,11) p1(2,12) p1(2,10) p1(2,9)],[p1(3,11) p1(3,12) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % h8=patch([p1(1,14) p1(1,13) p1(1,10) p1(1,9)],[p1(2,14) p1(2,13) p1(2,10) p1(2,9)],[p1(3,14) p1(3,13) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
% % % % % % % % hG=[h1 h2 h3 h4 h5 h6 h7 h8];
% % % % % % % % end
%% 画末端执行器，高级版
function plotEffector_keith(T, effector, beta, color)
%   this is a function to plot the gripper
%   there is two input, where the former is used to set the pose of gripper,
%   and the latter is set the open angle.
%
%   Author Keith W.
%   Ver. 1.0
%   Date 04.29.2022

if nargin == 3
    beta = 0;
end
if nargin == 2
    color = [0.5 0.5 0.5];
    beta = 0;
end
R=T(1:3,1:3);
P=T(1:3,4);
stator = effector.stator;
rotor = effector.rotor;
block_size_rotor = size(rotor,2);
block_size_stator = size(stator,2);
if(rotor(end,end) == 2)
    beta = beta/2;
    rotor2=rotor;
elseif(rotor(end,end) == 1)
    
end
r0 = eul2rotm([0 0 beta]);
for i = 1:3
    for j = 1:block_size_stator
        p1=[stator(i,j);stator(i+3,j);stator(i+6,j)];
        p1=R*p1;
        stator(i,j)=p1(1);stator(i+3,j)=p1(2);stator(i+6,j)=p1(3);
    end
    for j = 1:(block_size_rotor-1)
        p11=[rotor(i,j);rotor(i+3,j);rotor(i+6,j)];
        p1=R*r0*p11;
        rotor(i,j)=p1(1);rotor(i+3,j)=p1(2);rotor(i+6,j)=p1(3);
        if(rotor(end,end) == 2)
            p2 = R * r0'*(p11.*[1; -1; 1]);
            rotor2(i,j)=p2(1);rotor2(i+3,j)=p2(2);rotor2(i+6,j)=p2(3);
        end
    end
end
% P=[0 0 0]';
stator_x=stator(1:3,:)+P(1);stator_y=stator(4:6,:)+P(2);stator_z=stator(7:9,:)+P(3);
P=P+R*rotor(1:3,end); 
% P=[0 0 0]';
% figure;hold on;
rotor_x=rotor(1:3,1:end-1)+P(1);rotor_y=rotor(4:6,1:end-1)+P(2);rotor_z=rotor(7:9,1:end-1)+P(3);
patch(stator_x,stator_y,stator_z,'w','FaceAlpha',0.7,'EdgeColor','none','FaceColor',color);
patch(rotor_x,rotor_y,rotor_z,'w','FaceAlpha',0.7,'EdgeColor','none','FaceColor',color+0.3);
if(rotor(end,end) == 2)
    rotor_x=rotor2(1:3,1:end-1)+P(1);rotor_y=rotor2(4:6,1:end-1)+P(2);rotor_z=rotor2(7:9,1:end-1)+P(3);
    patch(rotor_x,rotor_y,rotor_z,'w','FaceAlpha',0.7,'EdgeColor','none','FaceColor',color+0.3);
end
end

%% 画半刚性段
function plotTorus(Lstem_l, r_Lstem,theta_Lstem,delta_Lstem,seg_color, T_config)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.11.2021
% Ver. 1.0
% as name said,drawing the torus.
% input1: curve radius.
% input2: curved cylinder's redius.
% input3: curve angle.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%% para
fig_resolution=128;  % figure resolution
alpha = 0.9;
% seg_color=100;  % continuum's color. 
if(nargin==4)
    T_config=eye(4);
end

if (theta_Lstem==0)
    %% clinder
    [x,y,z] = cylinder(r_Lstem,fig_resolution);
    h = Lstem_l;
    z = z*h;
else
    %% torus
    Lstem_r=Lstem_l/theta_Lstem;
    r=linspace(0, theta_Lstem, fig_resolution);
    st=linspace(0, 2*pi, fig_resolution);
    [u, v]=meshgrid(r, st);
    x=-((Lstem_r+r_Lstem*cos(v)).*cos(u)-Lstem_r);
    z=(Lstem_r+r_Lstem*cos(v)).*sin(u);
    y=r_Lstem*sin(v);
end
width_block=size(x,2);
height_block=size(x,1);
lay3=zeros(height_block,width_block,3);
lay_used=zeros(height_block,width_block,3);
lay3(:,:,1)=x;
lay3(:,:,2)=y;
lay3(:,:,3)=z;
t_delta_Lstem=[cos(delta_Lstem),-sin(delta_Lstem),0, 0;
    sin(delta_Lstem), cos(delta_Lstem), 0, 0;0, 0, 1, 0;0, 0, 0, 1];
for j =1:height_block
    for i =1:width_block
        temP= [lay3(j,i,1) lay3(j,i,2) lay3(j,i,3) 1]';
        tem=T_config*t_delta_Lstem*temP;
        lay_used(j,i,1)=tem(1);
        lay_used(j,i,2)=tem(2);
        lay_used(j,i,3)=tem(3);
    end
end
hold on;


surf(lay_used(:,:,1),lay_used(:,:,2),lay_used(:,:,3),'FaceAlpha',alpha,'EdgeColor','none','FaceColor',seg_color);

end

%% 画坐标系
function Pr=plotCoord(T,length)
if nargin==1
    length=1;
end
    arrowLen=length*10;
    lineWidth=length;
    R=T(1:3,1:3);
    P=T(1:3,4);
    px = R*[arrowLen 0 0]'+P;
    py = R*[0 arrowLen 0]'+P;
    pz = R*[0 0 arrowLen]'+P;
    line([P(1) px(1)],[P(2) px(2)],[P(3) px(3)],'Color','r','LineStyle','-','LineWidth',lineWidth);
    line([P(1) py(1)],[P(2) py(2)],[P(3) py(3)], 'Color','g','LineStyle','-','LineWidth',lineWidth);
    line([P(1) pz(1)],[P(2) pz(2)],[P(3) pz(3)], 'Color','b','LineStyle','-','LineWidth',lineWidth);
%     plot3(P(1),P(2),P(3),'c*');
    Pr=[P px P py P pz];
end

