function [Tt7, Tt19, Tt31] = registration_CW2(path_file1,path_file2,flag)

if(nargin==0)
    path_file1 = 'pt0';
    path_file2 = 'pt63';
    flag=1;
end
[C7, C191, ~]=getData71931(path_file1,flag);
[~, C192, C31]=getData71931(path_file2,flag);

T7=getCoord(C7);
T31=getCoord(C31);
T191=getCoord(C191);
T192=getCoord(C192);
T19 = eye(4);
T197=T191\T7;
T1931=T192\T31;


[d719, p719] = getCrossLine(T19(1:3,3),T19(1:3,4),T197(1:3,3),T197(1:3,4),T197(1:3,2));
p0=lineXplane(T1931(1:3,3),T1931(1:3,4),d719,p719);

z=T19(1:3,3)+T197(1:3,3)+T1931(1:3,3);
z = z/norm(z);
x = T197(1:3,4)-p0;
y = cross(z,x)/norm(cross(z,x));
x = cross(y,z);
T0 = [x,y,z,p0;0 0 0 1];

Tt7=T0\T197;
Tt19=T0\T19;
Tt31=T0\T1931;

figure(2); hold on; grid on; axis equal;
PlotAxis(10,T0);
PlotAxis(5,T197);
PlotAxis(5,T19);
PlotAxis(5,T1931);
end



