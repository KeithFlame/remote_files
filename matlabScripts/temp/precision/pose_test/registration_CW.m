function [Tt7, Tt19, Tt31] = registration_CW(path_file,flag)
if(nargin==0)
    path_file = 'M1';
    flag=1;
end
[C7, C19, C31]=getData71931(path_file,flag);
T7=getCoord(C7);
T31=getCoord(C31);
T19=getCoord(C19);
[d719, p719] = getCrossLine(T19(1:3,3),T19(1:3,4),T7(1:3,3),T7(1:3,4),T7(1:3,2));
p0=lineXplane(T31(1:3,3),T31(1:3,4),d719,p719);

z=T19(1:3,3)+T7(1:3,3)+T31(1:3,3);
z = z/norm(z);
x = T7(1:3,4)-p0;
y = cross(z,x)/norm(cross(z,x));
x = cross(y,z);
T0 = [x,y,z,p0;0 0 0 1];


Tt7=T0\T7;
Tt19=T0\T19;
Tt31=T0\T31;

figure(1); hold on; grid on; axis equal;
plot3(C7(:,1),C7(:,2),C7(:,3),'r*');
plot3(C19(:,1),C19(:,2),C19(:,3),'g*');
plot3(C31(:,1),C31(:,2),C31(:,3),'b*');
PlotAxis(10,T0);
PlotAxis(5,T7);
PlotAxis(5,T19);
PlotAxis(5,T31);
end

