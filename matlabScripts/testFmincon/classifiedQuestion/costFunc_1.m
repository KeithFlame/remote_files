function f=costFunc_1(x)

SP=getSP_1(x);


K1=SP.structure.K1;
Ks=SP.structure.Ks;
Kr=1;

u1_b=SP.dependent_psi.u1_b;
us_b=SP.dependent_psi.us_b;
thetasi=SP.dependent_psi.thetasi;
dsi=SP.dependent_psi.dsi;
thetaso=SP.dependent_psi.thetaso;
theta1o=SP.dependent_psi.theta1o;
lso=SP.dependent_psi.lso;
l1o=SP.dependent_psi.l1o;


[Lsi,usi]=getLU1Trocar(thetasi,dsi);
[Lso,uso]=getLUNot1Trocar(thetaso,lso);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);


f=Lsi*(usi-us_b)'*Kr*Ks*(usi-us_b)+...
    Lso*(uso-us_b)'*Ks*(uso-us_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b);

end