function [qa,MP]=fC2M_2(u,x,l1)


if(x(1)>l1)
    L1=l1;
    flag = 0;
else
    L1=x(1);
    flag = 1;       %flag是指地日柔性段是否完全伸出，是对应1
end

Lr=x(2);
L2=x(3);
Lg=x(4);
K1=x(6);
K2=x(7);
E=50e9;
Lstem=x(8);
qa=zeros(size(u,1),4);

e3=[0 0 1]';

MP.E=E;                          %Rod Young's modules
MP.mu=0.25;                     %Rod Poisson rate
MP.G=MP.E/2/(1+MP.mu);          %Rod modules of rigidity
MP.d1=0.95e-3;                  %Rod 1 diameter
MP.d2=0.4e-3;                   %Rod 2 diameter
MP.rho1=2.5e-3;               %Rod 1 pitch circle radius
MP.rho2=2.7e-3;               %Rod 2 pitch circle radius
if(flag)
    MP.L=Lstem*1e-3;                    %Robot stem length
    MP.L1=L1*1e-3;                 %Robot seg1 length
else
    MP.L=Lstem*1e-3 + (x(1)-l1)*1e-3;                    %Robot stem length
    MP.L1=l1*1e-3;                 %Robot seg1 length
end
MP.L2=L2*1e-3;                 %Robot seg2 length
MP.Lr=Lr*1e-3;                   %Robot rigid seg length
MP.Lg=Lg*1e-3;                 %Robot gipper length

MP.L0=0;
MP.delta_t = -45/180*pi;  % 60 degree
MP.gamma =0.0;

MP.r21=[cos(MP.delta_t) sin(MP.delta_t) 0]'*MP.rho2;
MP.r22=[cos(-MP.delta_t) sin(-MP.delta_t) 0]'*MP.rho2;

MP.r11=[1 0 0]'*MP.rho1;
MP.r12=[0 1 0]'*MP.rho1;  

MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];

MP.I1=pi*MP.d1^4/64;
MP.I2=pi*MP.d2^4/64;

MP.A1=pi*MP.d1^2/4;
MP.A2=pi*MP.d2^2/4;

MP.J1=2*MP.I1;
MP.J2=2*MP.I2;

MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
MP.Ke2=diag(4*[MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);

MP.Kb1=diag((5/4+1)*[MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag(4*[MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]...
    + 1/4*[MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);


dGamma = zeros(4,6);
Ell=zeros(4,4);
Theta=zeros(6,4);
K=zeros(6,6);


dGamma(1:4,1:3)=(MP.L1+MP.L0*MP.gamma)*MP.Q1';
dGamma(3:4,4:6)=MP.L2*MP.Q2';

Ell(1:2,1:2)=diag([1 1])*(MP.L1+MP.L);
Ell(3:4,3:4)=diag([1 1])*(MP.L2+MP.Lr+MP.L1+MP.L);

Theta(1:3,1:4)=-MP.A1*MP.E*MP.Q1;
Theta(1:3,3:4)=-MP.A1*MP.E*MP.Q2*0;
Theta(4:6,3:4)=-MP.A2*MP.E*MP.Q2;

K(1:3,1:3)=4*MP.Kb1+4*MP.Kb2+K1*MP.Kb1;
K(1:3,4:6)=-4*MP.Kb2-K2*MP.Kb1;
K(4:6,4:6)=4*MP.Kb2+K2*MP.Kb1;

G = dGamma-Ell*pinv(Theta)/2*K; % transfer matrix  ==> from u -> q
for i = 1:size(u,1)
    uc=u(i,:);
    q=G*uc';
    qa(i,:)=q';
end
end