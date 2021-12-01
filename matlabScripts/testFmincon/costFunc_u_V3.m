function f=costFunc_u_V3(x)

SP=setInitVal;
L1=SP.L1;
Lr=SP.Lr;
L2=SP.L2;
Ls=SP.Ls;
K1=SP.K1;
Ks=SP.Ks;
theta1=SP.theta1;
Lstem=SP.Lstem;
zeta=SP.zeta;
theta1_b=L1/(Lstem*SP.zeta+L1)*theta1;
thetas_b=Lstem*SP.zeta/(Lstem*SP.zeta+L1)*theta1;
u1_b=[0 theta1_b/L1 0]';
us_b=[0 thetas_b/Lstem 0]';


x=x./[100 100 200];
thetasi=x(1);
theta1i=x(2);
d=x(3);
% d=x(4);
if(Ls>SP.c)
    d1i=0;
    dsi=d;
elseif(d+SP.l-Lr-L2>L1&&SP.l-Lr-L2<L1)
    d1i=L1-(SP.l-Lr-L2);
    dsi=d-d1i;
else
    dsi=0;
    d1i=d;
end

if(dsi==0)
    thetasi=0;
elseif(d1i==0)
    theta1i=0;
%     d1i=0;
%     dsi=d;
else
    %comment it. means ignore zeta.
    if(theta1i==0)

    else
        tem=sin(theta1i)/d1i*zeta*dsi;
    if(norm(tem)>1||tem<0.001)
%         thetasi=theta1i*zeta;
    else
        thetasi=asin(tem);
    end
    end
    d1i=d-dsi/sin(thetasi)*thetasi;
    
end

% if(SP.c==0)
%     thetasi=0;
%     theta1i=0;
% end
[Lsi,usi]=getLU1Trocar(thetasi,dsi);

[L1i,u1i]=getLU1Trocar(theta1i,d1i);
if(Ls>0)
    lso=Ls+d-Lsi;
    if(lso<0)
        lso=0;
    end
    l1o=L1;
else
    lso=0;
    l1o=SP.l-Lr-L2+d-L1i-Lsi;
end


theta1_left=theta1-thetasi-theta1i;
theta1o=theta1_left*l1o/(lso*zeta+l1o);
thetaso=theta1_left*lso*zeta/(lso*zeta+l1o);
[Lso,uso]=getLUNot1Trocar(thetaso,lso);
[L1o,u1o]=getLUNot1Trocar(theta1o,l1o);

if(norm(u1o)<norm(u1i))
    u1i=u1o*5;
end

f=Lsi*(usi-us_b)'*Ks*(usi-us_b)+...
    Lso*(uso-us_b)'*Ks*(uso-us_b)+...
    L1i*(u1i-u1_b)'*K1*(u1i-u1_b)+...
    L1o*(u1o-u1_b)'*K1*(u1o-u1_b)+...
    (L1-L1o-L1i)*u1_b'*K1*u1_b+...
     (Lstem-Lsi-Lso)*us_b'*Ks*us_b;
% f=f*d;

end