function [hD]=drawCircle(p,R,rho,color)
%-----plot circles (disks) with pose {p,R} and radius rho
% ver p1.0
% By Yuyang Chen
% Date 20200606
%--------------------------------------------------------%
N=20;
for i=1:N
    alpha=i/N*2*pi;
    px(i)=R(1,1)*rho*cos(alpha)+R(1,2)*rho*sin(alpha)+p(1);
    py(i)=R(2,1)*rho*cos(alpha)+R(2,2)*rho*sin(alpha)+p(2);
    pz(i)=R(3,1)*rho*cos(alpha)+R(3,2)*rho*sin(alpha)+p(3);
end
hD=patch(px,py,pz,color,'FaceColor',color,'FaceAlpha',0.4,'EdgeColor',color,'LineWidth',1);
end