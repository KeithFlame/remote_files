function [g,h]=nonlcon_2(x)


SP=getSP_2(x);
x=x./[50 10 20 1 20];
theta1=SP.psi.theta1;
l=SP.psi.l;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
c=SP.trocar.c;
g=[];

h1=theta1-sum(x(3:4));
h2=x(5)-x(1)/x(3)*sin(x(3));
h3=c-x(1)/x(3)*(1-cos(x(3)));
h4=l-Lr-L2+x(5)-x(1)-x(2);
h=[h1 h2 h3 h4];

end