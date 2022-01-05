function f=costFunc_u_Straight(x)
SP=getSPV1(x);
L1=SP.structure.L1;
Lstem=SP.structure.Lstem;
K1=SP.structure.K1;
Ks=SP.structure.Ks;

u1_b=SP.dependent_psi.u1_b;
us_b=SP.dependent_psi.us_b;
thetasi=SP.dependent_psi.thetasi;
theta1i=SP.dependent_psi.theta1i;
dsi=SP.dependent_psi.dsi;
d1i=SP.dependent_psi.d1i;
thetaso=SP.dependent_psi.thetaso;
theta1o=SP.dependent_psi.theta1o;
lso=SP.dependent_psi.lso;
l1o=SP.dependent_psi.l1o;


[Lsi,usi]=getLU1Trocar(thetasi,dsi);
[Lso,uso]=getLUNot1Trocar(thetaso,lso);
[L1i,u1i]=getLU1Trocar(theta1i,d1i);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);
Kr=0.3;
f=Lsi*(usi-us_b)'*Kr*Ks*(usi-us_b)+...
    Lso*(uso-us_b)'*Ks*(uso-us_b)+...
    L1i*(u1i-u1_b)'*Kr*K1*(u1i-u1_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b)+...
    (L1-L1o-L1i)^2*u1_b'*K1*u1_b+...
     (Lstem-Lsi-Lso)^2*us_b'*Ks*us_b;
% us_i1=[0 0 0]';us_o1=[0 SP.psi.theta1/(SP.psi.l-SP.structure.L2-SP.structure.Lr) 0]';
% f=L1i*(u1i-us_i1)'*Kr*K1*(u1i-us_i1)+...
%     L1o*(u1o-us_o1)'*K1*(u1o-us_o1);
% +...
%     (L1-L1o-L1i)^2*u1_b'*K1*u1_b+...
%      (Lstem-Lsi-Lso)^2*us_b'*Ks*us_b
end