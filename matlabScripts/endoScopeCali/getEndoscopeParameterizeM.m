syms L1;
syms Lr;
syms L2;
syms Lg;
syms K1;
syms K2;
syms Lstem;
syms E;
% qa=zeros(size(u,1),8);

e1=[1 0 0]';
e2=[0 1 0]';
e3=[0 0 1]';

MP.E=E;                      %Rod Young's modules
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
MP.delta_t = -60/180*pi;  % 60 degree
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
MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);

% MP.tpuE=7e6;
% MP.tpuG=MP.tpuE/2/(1+MP.mu);  % mu 
% MP.tpuD=10e-3;
% MP.tpud=9.53e-3;
% MP.tpurho=(MP.tpuD+MP.tpud)/4;
% MP.tpuA = pi/4*(MP.tpuD^2-MP.tpud^2);
% MP.tpuI = pi*(MP.tpuD^4-MP.tpud^4)/64;
% MP.tpuKb=diag([MP.tpuE*MP.tpuI MP.tpuE*MP.tpuI 2*MP.tpuG*MP.tpuI]);
% MP.tpuM = pi*(MP.tpuD^4-MP.tpud^4)/128*MP.tpuE;
% 
% % bellow
% MP.bellowE=160.99825*1e9;
% MP.bellowmu=0.3;
% MP.bellowRc=0.199*1e-3;
% MP.bellowRr=0.199*1e-3;
% MP.bellowD=9.50*1e-3;
% MP.bellowd=6.405*1e-3;
% MP.bellowN1=25;
% MP.bellowN2=18*2;
% MP.bellowt=0.025*1e-3;
% MP.bellowDeff=(MP.bellowD+MP.bellowd)/2;
% MP.bellowKax1=4.3*MP.bellowE*(MP.bellowD+MP.bellowd)*MP.bellowt^3/((MP.bellowD-MP.bellowd-MP.bellowt)^3*MP.bellowN1);
% MP.bellowKax2=4.3*MP.bellowE*(MP.bellowD+MP.bellowd)*MP.bellowt^3/((MP.bellowD-MP.bellowd-MP.bellowt)^3*MP.bellowN2);
% 
% MP.bellowKt1=pi*MP.bellowE*MP.bellowt/(1-MP.bellowmu^2)*...
%     (1-MP.bellowmu)*(MP.bellowd/2)^3/(MP.bellowN1*(pi-2)*(MP.bellowRc+MP.bellowRr)+2*(MP.bellowD-MP.bellowd));
% MP.bellowKt2=pi*MP.bellowE*MP.bellowt/(1-MP.bellowmu^2)*...
%     (1-MP.bellowmu)*(MP.bellowd/2)^3/(MP.bellowN2*(pi-2)*(MP.bellowRc+MP.bellowRr)+2*(MP.bellowD-MP.bellowd));
% MP.bellowKb1=diag([MP.bellowKax1*MP.bellowDeff^2/8 MP.bellowKax1*MP.bellowDeff^2/8 MP.bellowKt1]);
% MP.bellowKb2=diag([MP.bellowKax2*MP.bellowDeff^2/8 MP.bellowKax2*MP.bellowDeff^2/8 MP.bellowKt2]);

dGamma = sym(zeros(8,6));
Ell=sym(zeros(8,8));
Theta=sym(zeros(6,8));
K=sym(zeros(6,6));
% tpuK=zeros(8,6);

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

u=[1 1 1 1 1 1]';
q=G*u;

