function SP_res=getSP_2(x)

persistent SP0;
if(nargin==0)
    SP_res=SP0;
    return;
end
SP=setInitValV2;
x=x./[50 10 20 1 20];
l1i=x(1);
l1o=x(2);
theta1i=x(3);
theta1o=x(4);
d=x(5);

SP.dependent_psi.theta1i=theta1i;
SP.dependent_psi.l1i=l1i;
if(theta1i>1e-4)
    SP.dependent_psi.d1i=l1i/theta1i*sin(theta1i);
else
    SP.dependent_psi.d1i=0;
end
SP.dependent_psi.theta1o=theta1o;
SP.dependent_psi.l1o=l1o;

SP.dependent_psi.dsi=0;
SP.dependent_psi.lsi=0;
SP.dependent_psi.thetasi=0;
SP.dependent_psi.thetaso=0;
SP.dependent_psi.lso=0;
SP.trocar.d=d;
SP0=SP;
SP_res=SP;
end