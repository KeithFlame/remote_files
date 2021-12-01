function SP_res=getSPV1(x)

persistent SP0;
if(nargin==0)
    SP_res=SP0;
    return;
end
SP=setInitValV1;
x=x./[100 100 300 200];
thetasi=x(1);
d_l1i=x(3);
d=x(4);

L1=SP.structure.L1;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
Lstem=SP.structure.Lstem;
zeta=SP.structure.zeta;
l=SP.psi.l;
theta1=SP.psi.theta1;

Ls=SP.dependent_psi.Ls;

if(Ls>0)
    thetaso=x(2);
    theta1i=0;
    d1i=0;
else
    theta1i=x(2);
    thetaso=0;
    d1i=d_l1i;    
end

dsi=d-d1i;
[l1i,~]=getLU1Trocar(theta1i,d1i);
if(l-L2-Lr-L1+l1i<0)
    dsi=0;
    thetasi=0;
    d=d1i;
end

if(thetasi==0)
    lso=Ls;
else
    lso=Ls+d-d/sin(thetasi)*thetasi;
    if(lso<0)
        lso=0;
    end
end
if(theta1i==0)
    l1o=L1-l1i;
else
    l1o=l-L2-Lr+d1i-l1i;
    if(l1o>L1-l1i)
        l1o=L1-l1i;
    end
end

theta1_b=L1/(Lstem*zeta+L1)*theta1;
thetas_b=Lstem*zeta/(Lstem*zeta+L1)*theta1;
u1_b=[0 theta1_b/L1 0]';
us_b=[0 thetas_b/Lstem 0]';
theta1o=theta1-thetasi-thetaso-theta1i;

SP.dependent_psi.u1_b=u1_b;
SP.dependent_psi.us_b=us_b;
SP.dependent_psi.thetasi=thetasi;
SP.dependent_psi.theta1i=theta1i;
SP.dependent_psi.dsi=dsi;
SP.dependent_psi.d1i=d1i;
SP.dependent_psi.thetaso=thetaso;
SP.dependent_psi.theta1o=theta1o;
SP.dependent_psi.lso=lso;
SP.dependent_psi.l1o=l1o;

SP.trocar.d=d;
SP0=SP;
SP_res=SP;
end