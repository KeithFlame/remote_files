function [Tt4, Tt14, Tt40] = registration_CW3


flag=2;
[C4_11, C14_11, ~]=getData71931('M6',flag);
[C4_12, C14_12, ~]=getData71931('M7',flag);
[C4_13, C14_13, ~]=getData71931('M8',flag);
[C4_14, C14_14, ~]=getData71931('M9',flag);
[C4_15, C14_15, ~]=getData71931('M10',flag);
[~, C14_21, C40_11]=getData71931('M1',flag);
[~, C14_22, C40_12]=getData71931('M2',flag);
[~, C14_23, C40_13]=getData71931('M3',flag);
[~, C14_24, C40_14]=getData71931('M4',flag);
[~, C14_25, C40_15]=getData71931('M5',flag);

C4 = (C4_11+C4_12+C4_13+C4_14+C4_15)/5;
C141 = (C14_11+C14_12+C14_13+C14_14+C14_15)/5;
C142 = (C14_21+C14_22+C14_23+C14_24+C14_25)/5;
C40 = (C40_11+C40_12+C40_13+C40_14+C40_15)/5;

T4=getCoord(C4);
T40=getCoord(C40);
T141=getCoord(C141);
T142=getCoord(C142);
T14 = eye(4);
T144=T141\T4;
T1440=T142\T40;


[d414, p414] = getCrossLine(T14(1:3,3),T14(1:3,4),T144(1:3,3),T144(1:3,4),T144(1:3,2));
p0=lineXplane(T1440(1:3,3),T1440(1:3,4),d414,p414);

z=T14(1:3,3)+T144(1:3,3)+T1440(1:3,3);
z = z/norm(z);
x = T144(1:3,4)-p0;
y = cross(z,x)/norm(cross(z,x));
x = cross(y,z);
T0 = [x,y,z,p0;0 0 0 1];

Tt4=T0\T144;
Tt14=T0\T14;
Tt40=T0\T1440;

figure(2); hold on; grid on; axis equal;
PlotAxis(10,T0);
PlotAxis(5,T144);
PlotAxis(5,T14);
PlotAxis(5,T1440);
end



