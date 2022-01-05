function SP_res=getSP_3(x)

persistent SP0;
if(nargin==0)
    SP_res=SP0;
    return;
end
SP=setInitValV2;
x=x./[50 50 10 20 20 1 20];
lsi=x(1);
l1i=x(2);
l1o=x(3);
thetasi=x(4);
theta1i=x(5);
theta1o=x(6);
d=x(7);

SP.dependent_psi.thetasi=thetasi;
SP.dependent_psi.lsi=lsi;
if(thetasi>1e-4)
    SP.dependent_psi.dsi=lsi/thetasi*sin(thetasi);
else
    SP.dependent_psi.dsi=0;
end
SP.dependent_psi.theta1i=theta1i;
SP.dependent_psi.theta1o=theta1o;
SP.dependent_psi.l1i=l1i;
SP.dependent_psi.dsi=lsi/thetasi*sin(thetasi);
SP.dependent_psi.l1o=l1o;
SP.dependent_psi.d1i=d-SP.dependent_psi.dsi;
SP.trocar.d=d;

SP.dependent_psi.lso=0;
SP.dependent_psi.thetaso=0;
SP0=SP;
SP_res=SP;
end