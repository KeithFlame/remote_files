syms lsi thetasi c d x0 r zeta;
P0=[-c,0];
P1=[lsi/thetasi-c, 0];
P2=[lsi/thetasi*(1-cos(thetasi))-c,lsi/thetasi*sin(thetasi)];
P3=[0,d];
L1=[(P2(2)-P1(2))/(P2(1)-P1(1)) 0];
L1(2)=P1(2)-L1(1)*P1(1);
c1=(P3(1)-x0)^2+(P3(2)+L1(1)*x0+L1(2))^2-r^2;
c2=(P2(1)-x0)^2+(P2(2)+L1(1)*x0+L1(2))^2-r^2;
c3=(P2(1)-x0)^2+(P2(2)+L1(1)*x0+L1(2))^2-(P3(1)-x0)^2+(P3(2)+L1(1)*x0+L1(2))^2;
f=solve(c3,x0);
xp=simplify(f);
yp=xp*L1(1)+L1(2);
c4=thetasi/lsi -zeta*sqrt(xp(1)^2+(d-yp(1))^2);
c=0.5;d=5;
zeta=0.2;
c5=thetasi/lsi -zeta*sqrt(xp(1)^2+(d-yp(1))^2);
flsi=solve(c4,lsi)
