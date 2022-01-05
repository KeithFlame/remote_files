function SP_res=getSP_1(x)

persistent SP0;
if(nargin==0)
    SP_res=SP0;
    return;
end
SP=setInitValV2;
x=x./[50 50 20 20 1 20];
L1=SP.structure.L1;
lsi=x(1);
lso=x(2);
thetasi=x(3);
thetaso=x(4);
theta1o=x(5);
d=x(6);


SP.dependent_psi.thetasi=thetasi;
SP.dependent_psi.lsi=lsi;
if(thetasi>1e-4)
    SP.dependent_psi.dsi=lsi/thetasi*sin(thetasi);
else
    SP.dependent_psi.dsi=0;
end
SP.dependent_psi.thetaso=thetaso;
SP.dependent_psi.theta1o=theta1o;
SP.dependent_psi.lso=lso;
SP.dependent_psi.l1o=L1;


SP.dependent_psi.d1i=0;
SP.dependent_psi.l1i=0;
SP.dependent_psi.theta1i=0;

SP.trocar.d=d;
SP0=SP;
SP_res=SP;
end