function f=costFunc_2(x)

SP=getSP_2(x);


K1=SP.structure.K1;
Kr=1;

u1_b=SP.dependent_psi.u1_b;
theta1i=SP.dependent_psi.theta1i;
d1i=SP.dependent_psi.d1i;
theta1o=SP.dependent_psi.theta1o;
l1o=SP.dependent_psi.l1o;


[L1i,u1i]=getLU1Trocar(theta1i,d1i);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);


f=L1i*(u1i-u1_b)'*Kr*K1*(u1i-u1_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b);

end