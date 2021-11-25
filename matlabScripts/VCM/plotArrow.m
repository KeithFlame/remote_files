function []=plotArrow(p,dir,Len,color)
%-----plot an arrow
% p- arrow root position
% dir- arrow direction
% Len- arrow length
%------Info---------
% ver p1.0
% By Yuyang Chen
% Date 20200606
%----------------------------------------------------%
if(nargin==2)
    Len=5;
    color=[1 0 0]';
end
angle=15/180*pi;dir_n=dir/norm(dir);
%dir=dir_n*Len;
if(cross(dir_n,[0 0 1]')<1e-3)
    normal=cross([1 1 0]',dir);normal_1=normal/norm(normal)*tan(angle)*(Len);
else
    normal=cross([0 0 1]',dir);normal_1=normal/norm(normal)*tan(angle)*(Len);
end

plot3([p(1) p(1)+dir(1)],[p(2) p(2)+dir(2)],[p(3) p(3)+dir(3)],'-','linewidth',1,'Color',color); 
plot3([p(1)+dir(1),p(1)+dir(1)-Len*dir_n(1)+normal_1(1)],[p(2)+dir(2) p(2)+dir(2)-Len*dir_n(2)+normal_1(2)],[p(3)+dir(3) p(3)+dir(3)-Len*dir_n(3)+normal_1(3)],'-','linewidth',1,'Color',color);
plot3([p(1)+dir(1),p(1)+dir(1)-Len*dir_n(1)-normal_1(1)],[p(2)+dir(2) p(2)+dir(2)-Len*dir_n(2)-normal_1(2)],[p(3)+dir(3) p(3)+dir(3)-Len*dir_n(3)-normal_1(3)],'-','linewidth',1,'Color',color);
end