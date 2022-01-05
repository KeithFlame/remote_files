function [g,h]=nonlcon_1(x)

SP=getSP_1(x);
x=x./[50 50 20 20 1 20];
theta1=SP.psi.theta1;
l=SP.psi.l;
L1=SP.structure.L1;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
zeta=SP.structure.zeta;
c=SP.trocar.c;


g=[];

h1=theta1-sum(x(3:5));
% h2=zeta*x(5)/x(2)-x(4)/x(1);
h2=zeta*x(5)/L1-x(4)/x(2);

h3=x(6)-x(1)/x(3)*sin(x(3));
h4=c-x(1)/x(3)*(1-cos(x(3)));
% h5=l-Lr-L2+x(5)-x(1)-x(2); 
h5=l-Lr-L2-L1+x(6)-x(1)-x(2); 
h=[h1 h2 h3 h4 h5];


end