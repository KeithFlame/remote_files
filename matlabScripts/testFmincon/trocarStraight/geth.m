
function [g,h]=geth(x)
SP=getSPV1(x);
c=SP.trocar.c;
Ls=SP.dependent_psi.Ls;

L1=SP.structure.L1;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
l=SP.psi.l;
d1i=SP.dependent_psi.d1i;
d=SP.trocar.d;
thetasi=SP.dependent_psi.thetasi;
theta1i=SP.dependent_psi.theta1i;

g=d1i-d;
if(Ls>1e-5)
    if(thetasi==0)
        g=[g;-c;-1;-1;-1];
    else
        g=[g;d/sin(thetasi)*(1-cos(thetasi))-c;-1;-1;-1];
    end
else
    if(theta1i==0)
        g=[g;-c;-1;-1;-1];
    else
        dsi=d-d1i;
        if(thetasi==0)
            d1=0;
        else
            d1=dsi/sin(thetasi)*(1-cos(thetasi));
        end
    %     d2=x(3)/sin(theta1i)*(1-cos(theta1i));
        d2=2*d1i/cos(thetasi)/sin(theta1i)*sin(theta1i/2)*sin(theta1i/2+thetasi);
        g=[g;d1+d2-c;d1i+(l-L2-Lr-L1)-c;-x(3);-x(4)];
    end
end
% x1=x,
h=[];
end