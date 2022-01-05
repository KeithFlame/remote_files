function f=costFunc_phi(x)

R=[cos(x) -sin(x) 0;sin(x) cos(x) 0;0 0 1];
P=[0 0 0]';
T=[R,P;[0 0 0 1]];

Tc=plotResult(0);
Tm=setTm;
Tc=T*Tc;
P=Tm(1:3,4)-Tc(1:3,4);
z_d=dot(Tc(1:3,3),Tm(1:3,3));
if(z_d>1)
    z_d=1;
end
da=acosd(z_d);
f=norm(P)+da/1000;
end