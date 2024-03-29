function qa=tem_fromCurvature2Movitation(u,x)


L1=x(1);
Lr=x(2);
L2=x(3);
Lg=x(4);
K1=x(6);
K2=x(7);
E=50e9;
Lstem=x(8);
qa=zeros(size(u,1),8);

e1=[1 0 0]';
e2=[0 1 0]';
e3=[0 0 1]';

MP.E=E;                          %Rod Young's modules
MP.mu=0.25;                     %Rod Poisson rate
MP.G=MP.E/2/(1+MP.mu);          %Rod modules of rigidity
MP.d1=0.88e-3;                  %Rod 1 diameter
MP.d2=0.7e-3;                   %Rod 2 diameter
MP.rho1=8e-3*0.5;               %Rod 1 pitch circle radius
MP.rho2=8e-3*0.5;               %Rod 2 pitch circle radius
MP.L=Lstem*1e-3;                    %Robot stem length
MP.L1=L1*1e-3;                 %Robot seg1 length
MP.L2=L2*1e-3;                 %Robot seg2 length
MP.Lr=Lr*1e-3;                   %Robot rigid seg length
MP.Lg=Lg*1e-3;                 %Robot gipper length

MP.L0=0;
MP.delta_t = -45/180*pi;  % 60 degree
MP.gamma =0.0;

MP.r11=[cos(MP.delta_t) sin(MP.delta_t) 0]'*MP.rho1;
MP.r12=[cos(-MP.delta_t) sin(-MP.delta_t) 0]'*MP.rho1;
MP.r13=[cos(MP.delta_t + pi) sin(MP.delta_t + pi) 0]'*MP.rho1;
MP.r14=[cos(-MP.delta_t + pi) sin(-MP.delta_t + pi) 0]'*MP.rho1;

MP.r21=[1 0 0]'*MP.rho2;
MP.r22=[0 1 0]'*MP.rho2;
MP.r23=[-1 0 0]'*MP.rho2;
MP.r24=[0 -1 0]'*MP.rho2;    

MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r13)*e3 S(MP.r14)*e3];
MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3 S(MP.r23)*e3 S(MP.r24)*e3];

MP.I1=pi*MP.d1^4/64;
MP.I2=pi*MP.d2^4/64;

MP.A1=pi*MP.d1^2/4;
MP.A2=pi*MP.d2^2/4;

MP.J1=2*MP.I1;
MP.J2=2*MP.I2;

MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
MP.Ke2=diag([MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);
MP.Kb1=diag(4*[MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag(4*[MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);

dGamma = zeros(8,6);
Ell=zeros(8,8);
Theta=zeros(6,8);
K=zeros(6,6);


dGamma(1:4,1:3)=(MP.L1+MP.L0*MP.gamma)*MP.Q1';
dGamma(5:8,1:3)=(MP.L1+MP.L0*MP.gamma)*MP.Q2';
dGamma(5:8,4:6)=MP.L2*MP.Q2';

Ell(1:4,1:4)=diag([1 1 1 1])*(MP.L1+MP.L);
Ell(5:8,5:8)=diag([1 1 1 1])*(MP.L2+MP.Lr+MP.L1+MP.L);

Theta(1:3,1:4)=-MP.A1*MP.E*MP.Q1;
Theta(4:6,5:8)=-MP.A2*MP.E*MP.Q2;

K(1:3,1:3)=4*MP.Kb1+4*MP.Kb2+K1*MP.Kb1;
K(1:3,4:6)=-4*MP.Kb2-K2*MP.Kb1;
K(4:6,4:6)=4*MP.Kb2+K2*MP.Kb1;

G = dGamma-Ell*pinv(Theta)*K; % transfer matrix  ==> from u -> q
for i = 1:size(u,1)
    uc=u(i,:);
    q=G*uc';
    qa(i,:)=q';
end
end