
function f = cost_4_Length(x)

[T10,T20]=getT12;
tar=[1.32668  4.04989  95.0272  -0.642751  0.691515  -0.272235  -0.18592]';
Ttar=fromQuat2T_mm(tar);
dR1 = T10(1:3,1:3)'*Ttar(1:3,1:3);
dr1=rotm2eul(dR1);
dR2 = T20(1:3,1:3)'*(Ttar(1:3,1:3)*eul2rotm([0 pi 0]));
dr2=rotm2eul(dR2);
l11=T10(1:3,4)+T10(1:3,3)*(90+x(1));
l12=l11+Ttar(1:3,3)*x(2);
l21=T20(1:3,4)+T20(1:3,3)*(120+x(3));
dd=Ttar(1:3,1:3)*eul2rotm([0 pi 0]);
l22=l21+dd(1:3,3)*x(4);
p = Ttar(1:3,4);
f = norm(p-l12)+norm(p-l22);

end




