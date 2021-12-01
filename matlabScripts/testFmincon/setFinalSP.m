function SPP=setFinalSP(xh)
SP=setInitVal;
L1=SP.L1;
Lr=SP.Lr;
L2=SP.L2;
Ls=SP.Ls;
zeta=SP.zeta;

thetasi=xh(1);
theta1i=xh(2);
SP.d=xh(3);
if(Ls>1e-9)
    d1i=0;
    dsi=SP.d;
elseif(SP.d+SP.l-Lr-L2>L1&&SP.l-Lr-L2<L1)
    d1i=L1-(SP.l-Lr-L2);
    dsi=SP.d-d1i;
else
    dsi=0;
    d1i=SP.d;
end

if(dsi==0)
    thetasi=0;
elseif(d1i<1e-3)
    theta1i=0;
    d1i=0;
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
    
end
% if(SP.c==0)
%     thetasi=0;
%     theta1i=0;
% end
SP.thetasi=thetasi;
SP.theta1i=theta1i;
[Lsi,~]=getLU1Trocar(thetasi,dsi);
[L1i,~]=getLU1Trocar(theta1i,d1i);
SP.Lsi=Lsi;
SP.L1i=L1i;
if(Ls>1e-9)
    lso=Ls+SP.d-Lsi;
    l1o=L1;
else
    lso=0;
    l1o=SP.l-Lr-L2+SP.d-L1i-Lsi;
end
theta1_left=SP.theta1-thetasi-theta1i;
theta1o=theta1_left*l1o/(lso*zeta+l1o);
thetaso=theta1_left*lso*zeta/(lso*zeta+l1o);
[Lso,~]=getLUNot1Trocar(thetaso,lso);
[L1o,~]=getLUNot1Trocar(theta1o,l1o);
SP.thetaso=thetaso;
SP.theta1o=theta1o;
SP.Lso=Lso;
SP.L1o=L1o;
setInitVal(SP);
SPP=SP;
end