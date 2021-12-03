function SP_res=getSP_1(x)

persistent SP0;
if(nargin==0)
    SP_res=SP0;
    return;
end
SP=setInitValV2;
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
SP.dependent_psi.l1o=l1o;

SP.trocar.d=d;
SP0=SP;
SP_res=SP;
end