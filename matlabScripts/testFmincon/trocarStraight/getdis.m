function P=getdis(phi)
x=phi;
R=[cos(x) -sin(x) 0;sin(x) cos(x) 0;0 0 1];
P=[0 0 0]';
T=[R,P;[0 0 0 1]];

Tc=plotResult(0);
Tm=setTm;
Tc=T*Tc;
P=Tm(1:3,4)-Tc(1:3,4);
end