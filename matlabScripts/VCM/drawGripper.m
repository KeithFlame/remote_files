function [hG]=drawGripper(p,R,rho,Lg,color)
if(nargin == 4)
    color=[0 0 0];
end
if(Lg == 0)
    hG=[];
    return;
end
D1=rho*cos(pi/6);
D2=rho*sin(pi/6);
H1=Lg*0.3;
H2=Lg*0.4;
p0=[D1 D2 0;D1 -D2 0;-D1 -D2 0;-D1 D2 0;...
   D1 D2 H1;D1 -D2 H1;-D1 -D2 H1;-D1 D2 H1;...
   0 D2 H2;0 -D2 H2;...
   D1 D2 Lg;D1 -D2 Lg;-D1 -D2 Lg;-D1 D2 Lg]';
p1=p+R*p0;
X1=[p1(1,1) p1(1,5) p1(1,9) p1(1,8) p1(1,4)];
Y1=[p1(2,1) p1(2,5) p1(2,9) p1(2,8) p1(2,4)];
Z1=[p1(3,1) p1(3,5) p1(3,9) p1(3,8) p1(3,4)];
h1=patch([p1(1,1) p1(1,5) p1(1,9) p1(1,8) p1(1,4)]',[p1(2,1) p1(2,5) p1(2,9) p1(2,8) p1(2,4)]',[p1(3,1) p1(3,5) p1(3,9) p1(3,8) p1(3,4)]',color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h2=patch([p1(1,2) p1(1,6) p1(1,10) p1(1,7) p1(1,3)],[p1(2,2) p1(2,6) p1(2,10) p1(2,7) p1(2,3)],[p1(3,2) p1(3,6) p1(3,10) p1(3,7) p1(3,3)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h3=patch([p1(1,1) p1(1,2) p1(1,6) p1(1,5)],[p1(2,1) p1(2,2) p1(2,6) p1(2,5)],[p1(3,1) p1(3,2) p1(3,6) p1(3,5)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h4=patch([p1(1,4) p1(1,3) p1(1,7) p1(1,8)],[p1(2,4) p1(2,3) p1(2,7) p1(2,8)],[p1(3,4) p1(3,3) p1(3,7) p1(3,8)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h5=patch([p1(1,5) p1(1,6) p1(1,10) p1(1,9)],[p1(2,5) p1(2,6) p1(2,10) p1(2,9)],[p1(3,5) p1(3,6) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h6=patch([p1(1,8) p1(1,7) p1(1,10) p1(1,9)],[p1(2,8) p1(2,7) p1(2,10) p1(2,9)],[p1(3,8) p1(3,7) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h7=patch([p1(1,11) p1(1,12) p1(1,10) p1(1,9)],[p1(2,11) p1(2,12) p1(2,10) p1(2,9)],[p1(3,11) p1(3,12) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
h8=patch([p1(1,14) p1(1,13) p1(1,10) p1(1,9)],[p1(2,14) p1(2,13) p1(2,10) p1(2,9)],[p1(3,14) p1(3,13) p1(3,10) p1(3,9)],color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
hG=[h1 h2 h3 h4 h5 h6 h7 h8];
end