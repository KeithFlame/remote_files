function [Posi_1,Ori_1,P_snake ] = PlotSnake(de_e, th_e, L_e, param)
%%This function plot the snake under delta_end, theta_end and L_end, on the
%%basis of global Position_1 and Orientation_1, and output new Position_1 and Orientation_1
if nargin==3
    param=[0.01 1000];
end
global Posi_1 Ori_1
de_e=-de_e;
r=param(1);
Rcb=[cos(de_e) -sin(de_e) 0; sin(de_e) cos(de_e) 0; 0 0 1];
Rbend=[cos(th_e) 0 sin(th_e);0 1 0;-sin(th_e) 0 cos(th_e)];
ratio=param(2);
N=round(L_e*ratio);
for i=1:N+1
    if N==0
      p_x=0;p_y=0;p_z=0;
      s_x1=r*cos(de_e);
      s_y1=r*sin(-de_e);
      s_z1=0;
      s_x2(i)=r*cos(pi/2-de_e);
      s_y2(i)=r*sin(pi/2-de_e);
      s_z2(i)=0;
      s_x3(i)=r*cos(pi-de_e);
      s_y3(i)=r*sin(pi-de_e);
      s_z3(i)=0;
      s_x4(i)=r*cos(pi*3/2-de_e);
      s_y4(i)=r*sin(pi*3/2-de_e);
      s_z4(i)=0;
    else
    if th_e==0
    p_x=zeros(1,N+1);
    p_y=zeros(1,N+1);
    p_z=linspace(0,L_e,N+1);
    for i=1:N+1
        s_x1(i)=r*cos(de_e);
        s_y1(i)=r*sin(-de_e);
        s_z1(i)=(i-1)/N*L_e;
        s_x2(i)=r*cos(pi/2-de_e);
        s_y2(i)=r*sin(pi/2-de_e);
        s_z2(i)=(i-1)/N*L_e;
        s_x3(i)=r*cos(pi-de_e);
        s_y3(i)=r*sin(pi-de_e);
        s_z3(i)=(i-1)/N*L_e;
        s_x4(i)=r*cos(pi*3/2-de_e);
        s_y4(i)=r*sin(pi*3/2-de_e);
        s_z4(i)=(i-1)/N*L_e;
    end
else
    for i=1:N+1
        s(i)=(i-1)/N*L_e;
        p_x(i)=L_e/(th_e)*(1-cos(s(i)/L_e*(th_e)));
        p_y(i)=0;
        p_z(i)=L_e/(th_e)*(sin(s(i)/L_e*(th_e)));
       %%===
        s_x1(i)=(L_e/(th_e)-r*cos(de_e))*(1-cos(s(i)/(L_e/(th_e))))+r*cos(de_e);
        s_y1(i)=-r*sin(de_e);
        s_z1(i)=(L_e/(th_e)-r*cos(de_e))*sin(s(i)/(L_e/(th_e)));
        
        s_x2(i)=(L_e/(th_e)-r*cos(pi/2-de_e))*(1-cos(s(i)/(L_e/(th_e))))+r*cos(pi/2-de_e);
        s_y2(i)=r*sin(pi/2-de_e);
        s_z2(i)=(L_e/(th_e)-r*cos(pi/2-de_e))*sin(s(i)/(L_e/(th_e)));
        
        s_x3(i)=(L_e/(th_e)-r*cos(pi-de_e))*(1-cos(s(i)/(L_e/(th_e))))+r*cos(pi-de_e);
        s_y3(i)=r*sin(pi-de_e);
        s_z3(i)=(L_e/(th_e)-r*cos(pi-de_e))*sin(s(i)/(L_e/(th_e)));
        
        s_x4(i)=(L_e/(th_e)-r*cos(pi*3/2-de_e))*(1-cos(s(i)/(L_e/(th_e))))+r*cos(pi*3/2-de_e);
        s_y4(i)=r*sin(pi*3/2-de_e);
        s_z4(i)=(L_e/(th_e)-r*cos(pi*3/2-de_e))*sin(s(i)/(L_e/(th_e)));  
    end
end
    end
end
P_p=[p_x;p_y;p_z];
P_s1=[s_x1;s_y1;s_z1];
P_s2=[s_x2;s_y2;s_z2];
P_s3=[s_x3;s_y3;s_z3];
P_s4=[s_x4;s_y4;s_z4];
P_p=Ori_1*Rcb*P_p;
P_s1=Ori_1*Rcb*P_s1;
P_s2=Ori_1*Rcb*P_s2;
P_s3=Ori_1*Rcb*P_s3;
P_s4=Ori_1*Rcb*P_s4;

p_x=P_p(1,:)+Posi_1(1)*ones(1,round(L_e*ratio)+1); 
p_y=P_p(2,:)+Posi_1(2)*ones(1,round(L_e*ratio)+1); 
p_z=P_p(3,:)+Posi_1(3)*ones(1,round(L_e*ratio)+1); 
s_x1=P_s1(1,:)+Posi_1(1)*ones(1,round(L_e*ratio)+1);
s_y1=P_s1(2,:)+Posi_1(2)*ones(1,round(L_e*ratio)+1);
s_z1=P_s1(3,:)+Posi_1(3)*ones(1,round(L_e*ratio)+1);
s_x2=P_s2(1,:)+Posi_1(1)*ones(1,round(L_e*ratio)+1);
s_y2=P_s2(2,:)+Posi_1(2)*ones(1,round(L_e*ratio)+1);
s_z2=P_s2(3,:)+Posi_1(3)*ones(1,round(L_e*ratio)+1);
s_x3=P_s3(1,:)+Posi_1(1)*ones(1,round(L_e*ratio)+1);
s_y3=P_s3(2,:)+Posi_1(2)*ones(1,round(L_e*ratio)+1);
s_z3=P_s3(3,:)+Posi_1(3)*ones(1,round(L_e*ratio)+1);
s_x4=P_s4(1,:)+Posi_1(1)*ones(1,round(L_e*ratio)+1);
s_y4=P_s4(2,:)+Posi_1(2)*ones(1,round(L_e*ratio)+1);
s_z4=P_s4(3,:)+Posi_1(3)*ones(1,round(L_e*ratio)+1);

a=linspace(0,2*pi,100);
x_0=r*cos(a);
y_0=r*sin(a);
x_0=x_0(:)';
y_0=y_0(:)';
z_0=zeros(1,100);
P_0=[x_0;y_0;z_0];
P_4=Rcb*Rbend*Rcb'*P_0;
P_0=Ori_1*P_0;
P_4=Ori_1*P_4;
x_0=P_0(1,:)+Posi_1(1)*ones(1,100);
y_0=P_0(2,:)+Posi_1(2)*ones(1,100);
z_0=P_0(3,:)+Posi_1(3)*ones(1,100);
x_4=P_4(1,:)+p_x(round(L_e*ratio)+1)*ones(1,100);
y_4=P_4(2,:)+p_y(round(L_e*ratio)+1)*ones(1,100);
z_4=P_4(3,:)+p_z(round(L_e*ratio)+1)*ones(1,100);


P_snake=[p_x;p_y;p_z];

hold on;
plot3(p_x,p_y,p_z,'-','linewidth',3,'Color',[.824 .608 .126]);
%plot3(s_x1,s_y1,s_z1,'-r','linewidth',2);
%plot3(s_x2,s_y2,s_z2,'-r','linewidth',2);
%plot3(s_x3,s_y3,s_z3,'-r','linewidth',2);
%plot3(s_x4,s_y4,s_z4,'-r','linewidth',2);
%patch(x_0,y_0,z_0,'r','FaceAlpha',1,'EdgeColor',[1 0 0],'LineWidth',2);
%patch(x_4,y_4,z_4,'r','FaceAlpha',1,'EdgeColor',[1 0 0],'LineWidth',2);

Posi_2=[p_x(round(L_e*ratio)+1) p_y(round(L_e*ratio)+1) p_z(round(L_e*ratio)+1)];
Ori_2=Ori_1*Rcb*Rbend*Rcb';
    c=0:0.001;
    t_x1=c*Ori_2(1,1)+Posi_2(1);
    t_x2=c*Ori_2(2,1)+Posi_2(2);
    t_x3=c*Ori_2(3,1)+Posi_2(3);
    t_y1=c*Ori_2(1,2)+Posi_2(1);
    t_y2=c*Ori_2(2,2)+Posi_2(2);
    t_y3=c*Ori_2(3,2)+Posi_2(3);
    t_z1=c*Ori_2(1,3)+Posi_2(1);
    t_z2=c*Ori_2(2,3)+Posi_2(2);
    t_z3=c*Ori_2(3,3)+Posi_2(3);
    
    %plot3(t_x1,t_x2,t_x3,'m-','LineWidth',2);
    %plot3(t_y1,t_y2,t_y3,'b-','LineWidth',2);
    %plot3(t_z1,t_z2,t_z3,'k-','LineWidth',2);
Ori_1=Ori_2;
Posi_1=Posi_2';
end