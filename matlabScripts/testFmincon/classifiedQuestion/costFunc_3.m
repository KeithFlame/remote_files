function f=costFunc_3(x)

SP=getSP(x);

K1=SP.structure.K1;
Kr=1;

u1_b=SP.dependent_psi.u1_b;
us_b=SP.dependent_psi.us_b;
thetasi=SP.dependent_psi.thetasi;
theta1i=SP.dependent_psi.theta1i;
dsi=SP.dependent_psi.dsi;
d1i=SP.dependent_psi.d1i;
theta1o=SP.dependent_psi.theta1o;
l1o=SP.dependent_psi.l1o;


[Lsi,usi]=getLU1Trocar(thetasi,dsi);
[L1i,u1i]=getLU1Trocar(theta1i,d1i);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);

f=Lsi*(usi-us_b)'*Kr*Ks*(usi-us_b)+...
    L1i*(u1i-u1_b)'*Kr*K1*(u1i-u1_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b);

end