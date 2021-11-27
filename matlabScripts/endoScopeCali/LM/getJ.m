syms L1;
syms Lr;
syms L2;
syms Lg;
syms K1;
syms K2;
syms Lstem;
syms E;
syms gamma3;
syms Ls;syms phi;syms theta1;syms delta1;syms theta2;syms delta2;
% qa=zeros(size(u,1),8);
%% get iterative M
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
G=simplify(G);


%% get iterative U
uc=sym(zeros(6,1));
uc(1)=theta1/L1*cos(pi/2-delta1);
uc(2)=theta1/L1*sin(pi/2-delta1);
uc(4)=theta2/L2*cos(pi/2-delta2);
uc(5)=theta2/L2*sin(pi/2-delta2);
%% get initial M
M=getInitialM;

%% get truth Q
q=M*uc;

%% get iterative U
u_=pinv(G)*q;
 
%% get iterative Psi
psi=sym(zeros(6,1));
psi(3)=L1*norm(u_(1:3));
psi(4)=pi/2-atan2(u_(2),u_(1));
psi(5)=L2*norm(u_(4:6));
psi(6)=pi/2-atan2(u_(5),u_(4));

%% get Config
PHI=phi;
THETA1=psi(3);
DELTA1=psi(4);%+config(1);
THETA2=psi(5);
DELTA2=psi(6);
k2=THETA2/L2;
k1=THETA1/L1;

%Ls
T1=[cos(PHI) -sin(PHI) 0 0
    sin(PHI) cos(PHI) 0 0
    0 0 1 Ls
    0 0 0 1];


%segment1 circular
if THETA1==0
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 L1
        0 0 0 1];

else
    cosTHETA1=cos(THETA1);sinTHETA1=sin(THETA1);cosDELTA1=cos(DELTA1);sinDELTA1=sin(DELTA1);
    T2=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
        0 0 0 1];

end

%Lr
T3=[1 0 0 0
    0 1 0 0
    0 0 1 Lr
    0 0 0 1];


%segment2 circular
if THETA2==0
    T4=[1 0 0 0
        0 1 0 0
        0 0 1 L2
        0 0 0 1];

else
    cosTHETA2=cos(THETA2);sinTHETA2=sin(THETA2);cosDELTA2=cos(DELTA2);sinDELTA2=sin(DELTA2);
    T4=[(cosDELTA2)^2*(cosTHETA2-1)+1 sinDELTA2*cosDELTA2*(cosTHETA2-1) cosDELTA2*sinTHETA2 cosDELTA2*(1-cosTHETA2)/k2
        sinDELTA2*cosDELTA2*(cosTHETA2-1) (cosDELTA2)^2*(1-cosTHETA2)+cosTHETA2 sinDELTA2*sinTHETA2 sinDELTA2*(1-cosTHETA2)/k2
        -cosDELTA2*sinTHETA2 -sinDELTA2*sinTHETA2 cosTHETA2 sinTHETA2/k2
        0 0 0 1];

end

%end effector
T5=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 Lg
    0 0 0 1];
T=T1*T2*T3*T4*T5;
%% get X
x=sym(zeros(6,1));
x(4)=T(1,4);x(5)=T(2,4);x(6)=T(3,4);