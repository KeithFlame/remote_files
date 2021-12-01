function SP=getSP(x)

persistent SP0;
if(nargin==0)
    SP=SP0;
    return;
end

x=x./[100 100 100 100 100];
thetasi=x(1);
theta1i=x(2);
lsi=x(3);
l1i=x(4);
d=x(5);

SP=setInitVal;
L1=SP.structure.L1;
Lr=SP.structure.Lr;
L2=SP.structure.L2;
Lstem=SP.structure.Lstem;
zeta=SP.structure.zeta;
l=SP.psi.l;
theta1=SP.psi.theta1;

theta1_b=L1/(Lstem*zeta+L1)*theta1;
thetas_b=Lstem*zeta/(Lstem*zeta+L1)*theta1;
u1_b=[0 theta1_b/L1 0]';
us_b=[0 thetas_b/Lstem 0]';



if(l-L1-Lr-L2+d-l1i-lsi>0)
    lso=l-L1-Lr-L2+d-l1i-lsi;
    l1o=L1;
    theta1_left=theta1-thetasi-theta1i;
    theta1o=theta1_left*l1o/(lso*zeta+l1o);
    thetaso=theta1_left*lso*zeta/(lso*zeta+l1o);
    l1i=0;
    theta1i=0;
else
    lso=0;
    thetaso=0;
    l1o=l-Lr-L2+d-l1i-lsi;
    theta1o=theta1-thetasi-theta1i;
    if(l1o+l1i<L1)
        thetasi=0;
        lsi=0;
    end
end

SP.dependent_psi.u1_b=u1_b;
SP.dependent_psi.us_b=us_b;
SP.dependent_psi.thetasi=thetasi;
SP.dependent_psi.theta1i=theta1i;
SP.dependent_psi.lsi=lsi;
SP.dependent_psi.l1i=l1i;
SP.dependent_psi.thetaso=thetaso;
SP.dependent_psi.theta1o=theta1o;
SP.dependent_psi.lso=lso;
SP.dependent_psi.l1o=l1o;

SP.trocar.d=d;
SP0=SP;
end