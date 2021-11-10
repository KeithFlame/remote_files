function [h]=PlotCircle(rad,center,vecZ)
color=[0.5 0 0];
line_width = 2;
N=100;
p=zeros(3,N+1);
vecZ = vecZ/norm(vecZ);
while(1)
    vecX = cross(vecZ,rand(3,1));
    if(norm(vecX)>0.0001)
        break;
    end
end
vecX = vecX/norm(vecX);
vecY = cross(vecZ,vecX);
for i=1:N
    p(:,i) = center+(vecX*cos(i/N*2*pi)+vecY*sin(i/N*2*pi))*rad;
end
p(:,N+1) = p(:,1);
plot3(p(1,:),p(2,:),p(3,:),'color',color,'lineWidth',line_width,'lineStyle','-');
end