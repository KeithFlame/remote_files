function [x2,y2,z2]=plotCircumscribedpolygon(r,n,heigh)
%hui zhi yuan xing de wai jie n bian xing
%r=1; %圆半径
%n=6; %n边形,n>=3
%heigh yan z fangxiang de gao du 
%画圆
% tt = (0:0.01:1)*2*pi;
% x=r*sin(tt);
% y=r*cos(tt);
% z=zeros(1,length(tt));
%plot3(x,y,z);
axis equal
hold on;
%外接
theta = pi/n*ones(1,n+1)+(0:1/n:1)*2*pi;
r1 = r/cos(2*pi/2/n)*ones(1,n+1);
[x2,y2]=pol2cart(theta,r1);
z2=ones(1,length(theta))*heigh;
%plot3(x2,y2,z2);
%cc=patch(x2,y2,z2,'facecolor','interp');
end



