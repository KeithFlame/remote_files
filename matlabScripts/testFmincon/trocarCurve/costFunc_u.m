function f=costFunc_u(x)




SP=getSP;

L1=SP.structure.L1;
Lstem=SP.structure.Lstem;
K1=SP.structure.K1;
Ks=SP.structure.Ks;

u1_b=SP.dependent_psi.u1_b;
us_b=SP.dependent_psi.us_b;
thetasi=SP.dependent_psi.thetasi;
theta1i=SP.dependent_psi.theta1i;
lsi=SP.dependent_psi.lsi;
l1i=SP.dependent_psi.l1i;
thetaso=SP.dependent_psi.thetaso;
theta1o=SP.dependent_psi.theta1o;
lso=SP.dependent_psi.lso;
l1o=SP.dependent_psi.l1o;

[Lsi,usi]=getLU1Trocar(thetasi,lsi);
[L1i,u1i]=getLU1Trocar(theta1i,l1i);
[Lso,uso]=getLUNot1Trocar(thetaso,lso);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);

f=Lsi*(usi-us_b)'*Ks*(usi-us_b)+...
    Lso*(uso-us_b)'*Ks*(uso-us_b)+...
    L1i*(u1i-u1_b)'*K1*(u1i-u1_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b)+...
    (L1-L1o-L1i)*u1_b'*K1*u1_b+...
     (Lstem-Lsi-Lso)*us_b'*Ks*us_b;

end