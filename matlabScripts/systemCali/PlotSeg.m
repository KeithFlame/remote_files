function [T_base,P_snake,h] = PlotSeg(delta, theta, Length, T_base, param)
%%This function plot the snake under delta_end, theta_end and L_end, on the
%%basis of global Position_1 and Orientation_1, and output new Position_1 and Orientation_1
if nargin==4
    param=[0.004 1000];%radius,resolution (defautly 1mm a point)
end
Posi_bae = T_base(1:3,4);
Ori_base = T_base(1:3,1:3);
delta=-delta;
r=param(1);
Rcb=[cos(delta) -sin(delta) 0; sin(delta) cos(delta) 0; 0 0 1];
Rbend=[cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
ratio=param(2);
N=round(Length*ratio);
for i=1:N+1
    if N==0
      p_x=0;p_y=0;p_z=0;
      s_x1=r*cos(delta);
      s_y1=r*sin(-delta);
      s_z1=0;
      s_x2(i)=r*cos(pi/2-delta);
      s_y2(i)=r*sin(pi/2-delta);
      s_z2(i)=0;
      s_x3(i)=r*cos(pi-delta);
      s_y3(i)=r*sin(pi-delta);
      s_z3(i)=0;
      s_x4(i)=r*cos(pi*3/2-delta);
      s_y4(i)=r*sin(pi*3/2-delta);
      s_z4(i)=0;
    else
    if theta==0
    p_x=zeros(1,N+1);
    p_y=zeros(1,N+1);
    p_z=linspace(0,Length,N+1);
    for i=1:N+1
        s_x1(i)=r*cos(delta);
        s_y1(i)=r*sin(-delta);
        s_z1(i)=(i-1)/N*Length;
        s_x2(i)=r*cos(pi/2-delta);
        s_y2(i)=r*sin(pi/2-delta);
        s_z2(i)=(i-1)/N*Length;
        s_x3(i)=r*cos(pi-delta);
        s_y3(i)=r*sin(pi-delta);
        s_z3(i)=(i-1)/N*Length;
        s_x4(i)=r*cos(pi*3/2-delta);
        s_y4(i)=r*sin(pi*3/2-delta);
        s_z4(i)=(i-1)/N*Length;
    end
else
    for i=1:N+1
        s(i)=(i-1)/N*Length;
        p_x(i)=Length/(theta)*(1-cos(s(i)/Length*(theta)));
        p_y(i)=0;
        p_z(i)=Length/(theta)*(sin(s(i)/Length*(theta)));
       %%===
        s_x1(i)=(Length/(theta)-r*cos(delta))*(1-cos(s(i)/(Length/(theta))))+r*cos(delta);
        s_y1(i)=-r*sin(delta);
        s_z1(i)=(Length/(theta)-r*cos(delta))*sin(s(i)/(Length/(theta)));
        
        s_x2(i)=(Length/(theta)-r*cos(pi/2-delta))*(1-cos(s(i)/(Length/(theta))))+r*cos(pi/2-delta);
        s_y2(i)=r*sin(pi/2-delta);
        s_z2(i)=(Length/(theta)-r*cos(pi/2-delta))*sin(s(i)/(Length/(theta)));
        
        s_x3(i)=(Length/(theta)-r*cos(pi-delta))*(1-cos(s(i)/(Length/(theta))))+r*cos(pi-delta);
        s_y3(i)=r*sin(pi-delta);
        s_z3(i)=(Length/(theta)-r*cos(pi-delta))*sin(s(i)/(Length/(theta)));
        
        s_x4(i)=(Length/(theta)-r*cos(pi*3/2-delta))*(1-cos(s(i)/(Length/(theta))))+r*cos(pi*3/2-delta);
        s_y4(i)=r*sin(pi*3/2-delta);
        s_z4(i)=(Length/(theta)-r*cos(pi*3/2-delta))*sin(s(i)/(Length/(theta)));  
    end
end
    end
end
P_p=[p_x;p_y;p_z];
P_s1=[s_x1;s_y1;s_z1];
P_s2=[s_x2;s_y2;s_z2];
P_s3=[s_x3;s_y3;s_z3];
P_s4=[s_x4;s_y4;s_z4];
P_p=Ori_base*Rcb*P_p;
P_s1=Ori_base*Rcb*P_s1;
P_s2=Ori_base*Rcb*P_s2;
P_s3=Ori_base*Rcb*P_s3;
P_s4=Ori_base*Rcb*P_s4;

p_x=P_p(1,:)+Posi_bae(1)*ones(1,round(Length*ratio)+1); 
p_y=P_p(2,:)+Posi_bae(2)*ones(1,round(Length*ratio)+1); 
p_z=P_p(3,:)+Posi_bae(3)*ones(1,round(Length*ratio)+1); 
s_x1=P_s1(1,:)+Posi_bae(1)*ones(1,round(Length*ratio)+1);
s_y1=P_s1(2,:)+Posi_bae(2)*ones(1,round(Length*ratio)+1);
s_z1=P_s1(3,:)+Posi_bae(3)*ones(1,round(Length*ratio)+1);
s_x2=P_s2(1,:)+Posi_bae(1)*ones(1,round(Length*ratio)+1);
s_y2=P_s2(2,:)+Posi_bae(2)*ones(1,round(Length*ratio)+1);
s_z2=P_s2(3,:)+Posi_bae(3)*ones(1,round(Length*ratio)+1);
s_x3=P_s3(1,:)+Posi_bae(1)*ones(1,round(Length*ratio)+1);
s_y3=P_s3(2,:)+Posi_bae(2)*ones(1,round(Length*ratio)+1);
s_z3=P_s3(3,:)+Posi_bae(3)*ones(1,round(Length*ratio)+1);
s_x4=P_s4(1,:)+Posi_bae(1)*ones(1,round(Length*ratio)+1);
s_y4=P_s4(2,:)+Posi_bae(2)*ones(1,round(Length*ratio)+1);
s_z4=P_s4(3,:)+Posi_bae(3)*ones(1,round(Length*ratio)+1);

a=linspace(0,2*pi,100);
x_0=r*cos(a);
y_0=r*sin(a);
x_0=x_0(:)';
y_0=y_0(:)';
z_0=zeros(1,100);
P_0=[x_0;y_0;z_0];
P_4=Rcb*Rbend*Rcb'*P_0;
P_0=Ori_base*P_0;
P_4=Ori_base*P_4;
x_0=P_0(1,:)+Posi_bae(1)*ones(1,100);
y_0=P_0(2,:)+Posi_bae(2)*ones(1,100);
z_0=P_0(3,:)+Posi_bae(3)*ones(1,100);
x_4=P_4(1,:)+p_x(round(Length*ratio)+1)*ones(1,100);
y_4=P_4(2,:)+p_y(round(Length*ratio)+1)*ones(1,100);
z_4=P_4(3,:)+p_z(round(Length*ratio)+1)*ones(1,100);


P_snake=[p_x;p_y;p_z];%all points

hold on;
hc=plot3(p_x,p_y,p_z,'k','linewidth',1);
h1=[];h2=[];h3=[];h4=[];hb=[];he=[];
% h1=plot3(s_x1,s_y1,s_z1,'r','linewidth',1);
% h2=plot3(s_x2,s_y2,s_z2,'r','linewidth',1);
% h3=plot3(s_x3,s_y3,s_z3,'r','linewidth',1);
% h4=plot3(s_x4,s_y4,s_z4,'r','linewidth',1);
% hb=patch(x_0,y_0,z_0,'r','FaceAlpha',0.3);
% he=patch(x_4,y_4,z_4,'r','FaceAlpha',0.3);

Posi_2=[p_x(round(Length*ratio)+1) p_y(round(Length*ratio)+1) p_z(round(Length*ratio)+1)]';
Ori_2=Ori_base*Rcb*Rbend*Rcb';
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
T_base = [Ori_2 Posi_2; 0 0 0 1];
h=[hc h1 h2 h3 h4 hb he];
end