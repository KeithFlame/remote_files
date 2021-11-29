function f=costFunc_u(x)

SP=setInitVal;
L1=SP.L1;
Ls=SP.Ls;
K1=SP.K1;
Ks=SP.Ks;
theta1=SP.theta1;

thetasi=x(1);
thetaso=x(2);
if(x(3)>x(4))
    x(3)=x(4);
end
d_l1i=x(3);
d=x(4);

Ls0=Ls+200;
theta1_b=L1/(Ls0*SP.zeta+L1)*theta1;
thetas_b=Ls0*SP.zeta/(Ls0*SP.zeta+L1)*theta1;
u1_b=[0 theta1_b/L1 0]';
us_b=[0 thetas_b/Ls0 0]';

if(Ls>0)
    theta1o=theta1-thetaso-thetasi;
    usi=[0 sin(thetasi)/d 0]';
    if(thetasi>1e-6)
        Ls_real=Ls+d-d/sin(thetasi)*thetasi;
    else
        Ls_real=Ls;
    end
    uso=[0 thetaso/Ls_real 0]';
    u1o=[0 theta1o/L1 0]';



    f=usi'*Ks*usi+(uso-us_b)'*Ks*(uso-us_b)+(u1o-u1_b)'*K1*(u1o-u1_b);
    
%     f=usi'*Ks*usi+uso'*Ks*uso+u1o'*K1*u1o;%-us_b'*Ks*us_b-u1_b'*K1*u1_b;
    
    SP.L1i=0;    
else
    u1_b=[0, theta1/(SP.l-SP.L2-SP.Lr), 0]';
    theta1i=x(2);
    theta1o=theta1-thetasi-theta1i;
    dsi=d-d_l1i;
%     u1_b=[0 theta1/(SP.l-SP.L2-SP.Lr) 0]';
    if(dsi<1e-5)
        
        u1i=[0 sin(theta1i)/(d_l1i/cos(thetasi)) 0]';
        L1o=SP.l-SP.L2-SP.Lr+d-d/sin(theta1i)*theta1i;
        u1o=[0,theta1o/L1o 0]';
        f=(u1i-u1_b)'*K1*(u1i-u1_b)*L1i+L1o*(u1o-u1_b)'*K1*(u1o-u1_b);
        if(theta1i>1e-6)
            SP.L1i=1/u1i(2)*theta1i;
        else
            SP.L1i=theta1i;
        end
    else
        Lsi=dsi/sin(thetasi) *thetasi;
        usi=[0 thetasi/Lsi 0]';
        L1i=(d_l1i/cos(thetasi))/sin(theta1i)*theta1i;
        u1i=[0 theta1i/L1i 0]';
        L1o=SP.l-SP.L2-SP.Lr+d_l1i-d_l1i/cos(thetasi)/sin(theta1i)*theta1i;
        u1o=[0,theta1o/L1o 0]';
        f=Lsi*usi'*Ks*usi+L1i*(u1i-u1_b)'*K1*(u1i-u1_b)+L1o*(u1o-u1_b)'*K1*(u1o-u1_b);
        if(theta1i>1e-6)
            SP.L1i=1/u1i(2)*theta1i;
        else
            SP.L1i=theta1i;
        end
    end
% x
end
setInitVal(SP);



end