function T=getCoord(C)
plane = fitPlane_SVD(C);
ps1=pointProjection(plane,C(5,:)');
ps2=pointProjection(plane,C(1,:)');



z=plane(1:3);
x= ps1-ps2;
y = cross(z,x)/norm(cross(z,x));
x = cross(y,z);



ps1=pointProjection(plane,C(3,:)');
ps2=pointProjection(plane,C(4,:)');
ps3=pointProjection(plane,C(5,:)');
x = ps1-ps2;
y = ps3-ps2;
x = x/norm(x);
z = cross(x,y)/norm(cross(x,y));
y=cross(z,x);
T = [x,y,z,ps2;0 0 0 1];
end