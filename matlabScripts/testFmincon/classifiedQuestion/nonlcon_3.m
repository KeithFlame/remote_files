function [g,h]=nonlcon_3(x)


SP=getSP_3(x);
x=x./[50 50 10 20 20 1 20];
theta1=SP.psi.theta1;
l=SP.psi.l;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
L1=SP.structure.L1;
zeta=SP.structure.zeta;
c=SP.trocar.c;

g=[];

h1=theta1-sum(x(4:6));
h2=zeta*x(5)/x(2)-x(4)/x(1);
%   d  - lsi/Tsi*sinTsi-2*l1i*sin(T1i/2)*(1-cos(T1i/2+Tsi))
h3=x(7)-x(1)/x(4)*sin(x(4))-2*x(2)/x(5)*sin(x(5)/2)*cos(x(5)/2+x(4));
%   d -2*l1i*sin(T1i/2)*sin(T1i/2+Tsi)  - lsi/Tsi(1-cosTsi)
h4=c-2*x(2)/x(5)*sin(x(5)/2)*sin(x(5)/2+x(4))...
    -x(1)/x(4)*(1-cos(x(4)));
h5=l-L1-Lr-L2+x(7)-x(1);
h6=L1-x(2)-x(3);
h=[h1 h2 h3 h4 h5 h6];

end