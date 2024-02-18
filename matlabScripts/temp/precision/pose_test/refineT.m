function [T,flag] = refineT(T0)
T = T0;
r1=T0(1,1:3);r2=T0(2,1:3);
r1 = r1/norm(r1);
r3 = cross(r1,r2)/norm(cross(r1,r2));
r2 = cross(r3,r1);
Rtem=[r1;r2;r3];
c1=Rtem(1:3,1);c2=Rtem(1:3,2);
c1 = c1/norm(c1);
c3 = cross(c1,c2)/norm(cross(c1,c2));
c2 = cross(c3,c1);
T(1:3,1:3)=[c1,c2,c3];
axang = rotm2axang(T(1:3,1:3)'*T0(1:3,1:3));
flag = axang(4)*180/pi;
end