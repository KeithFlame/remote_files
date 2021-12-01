function f=costFunc_u_V2(x)

SP=setInitVal;
L1=SP.L1;
L2=SP.L2;
Lr=SP.Lr;
Ls=SP.Ls;
K1=SP.K1;
Ks=SP.Ks;
theta1=SP.theta1;
Lstem=SP.Lstem;
zeta=SP.zeta;


x=x./[100 100 300 200];
thetasi=x(1);
d_l1i=x(3);
d=x(4);
if(Ls>0)
    thetaso=x(2);
    theta1i=0;
    dl1i=0;
else
    theta1i=x(2);
    thetaso=0;
    dl1i=d_l1i;
    
end
dsi=d-dl1i;
if(thetasi==0)
    lso=Ls;
else
    lso=Ls+d-d/sin(thetasi)*thetasi;
    if(lso<0)
        lso=0;
    end
end
if(theta1i==0)
    l1o=L1-dl1i;
else
    l1o=SP.l-L2-Lr+dl1i-dl1i/cos(thetasi)/sin(theta1i)*theta1i;
end

theta1_b=L1/(Lstem*zeta+L1)*theta1;
thetas_b=Lstem*zeta/(Lstem*zeta+L1)*theta1;
u1_b=[0 theta1_b/L1 0]';
us_b=[0 thetas_b/Lstem 0]';
theta1o=theta1-thetasi-thetaso-theta1i;
[Lsi,usi]=getLU1Trocar(thetasi,dsi);
[Lso,uso]=getLUNot1Trocar(thetaso,lso);
[L1i,u1i]=getLU1Trocar(theta1i,dl1i);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);

f=Lsi*(usi-us_b)'*Ks*(usi-us_b)+...
    Lso*(uso-us_b)'*Ks*(uso-us_b)+...
    L1i*(u1i-u1_b)'*K1*(u1i-u1_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b)+...
    (L1-L1o-L1i)*u1_b'*K1*u1_b+...
     (Lstem-Lsi-Lso)*us_b'*Ks*us_b;

end