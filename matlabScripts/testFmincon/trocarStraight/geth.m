
function [g,h]=geth(x)
x=x./[100 100 300 200];

SP=getSPV1;
c=SP.trocar.c;
Ls=SP.dependent_psi.Ls;

L1=SP.structure.L1;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
l=SP.psi.l;

d=x(4);
thetasi=x(1);
theta1i=x(2);

g=x(3)-x(4);
if(Ls>1e-5)
    g=[g;d/sin(thetasi)*(1-cos(thetasi))-c;-1;-1;-1];
else
    dsi=d-x(3);
    d1=dsi/sin(thetasi)*(1-cos(thetasi));
%     d2=x(3)/sin(theta1i)*(1-cos(theta1i));
    d2=2*x(3)/cos(thetasi)/sin(theta1i)*sin(theta1i/2)*sin(theta1i/2+thetasi);
    g=[g;d1+d2-c;x(3)+(l-L2-Lr-L1)-c;-x(3);-x(4)];
end
% x1=x,
h=[];
end