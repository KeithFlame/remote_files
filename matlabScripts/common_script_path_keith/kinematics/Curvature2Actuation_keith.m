function qa = Curvature2Actuation_keith(u,l,MBP)
% this is function to convert the curvature of continuum manipulator to the
% actuation length with several variables.
%
% input1: u curvature of the 2-seg continuum manipulator (6 X 1 vector).
% input2: l the feeding motor's extending length of this manipulator (a scalar).
% input3: MBP the multi-backbone parameters (a structural variable). 
% output: QA the actuation variables (6 X 1 vector)
%
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

e3=[0 0 1]';
if(nargin == 2 || isempty(MBP))
    MBP.E=50e9;                          %Rod Young's modules
    MBP.mu=0.25;                         %Rod Poisson rate
    MBP.d1=0.95e-3;                      %Rod diameter seg1
    MBP.d2=0.40e-3;                      %Rod diameter seg2
    MBP.rho1=2.5e-3;                     %Rod pitch circle radius seg1
    MBP.rho2=2.7e-3;                     %Rod pithc circle radius seg2
    MBP.L=600e-3;                        %Robot stem length
    MBP.L1=100e-3;                       %Robot seg1 length
    MBP.L2=20e-3;                        %Robot seg2 length
    MBP.Lr=10e-3;                        %Robot rigid seg length
    MBP.Lg=15e-3;                        %Robot gipper length
    MBP.delta_t = -45/180*pi;             % 45 degree

    MBP.r21=[cos(MBP.delta_t) sin(MBP.delta_t) 0]'*MBP.rho2;
    MBP.r22=[cos(-MBP.delta_t) sin(-MBP.delta_t) 0]'*MBP.rho2;
    
    MBP.r11=[1 0 0]'*MBP.rho1;
    MBP.r12=[0 1 0]'*MBP.rho1;
    MBP.Q1=[skewMatrix_keith(MBP.r11)*e3 skewMatrix_keith(MBP.r12)*e3 ...
        skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.Q2=[skewMatrix_keith(MBP.r21)*e3 skewMatrix_keith(MBP.r22)*e3];
    MBP.I1=pi*MBP.d1^4/64;MBP.A1=pi*MBP.d1^2/4;
    MBP.I2=pi*MBP.d2^4/64;MBP.A2=pi*MBP.d2^2/4;
    MBP.G=MBP.E/2/(1+MBP.mu);MBP.J=2*MBP.I1;
    MBP.Ke1=diag([3e8 3e8 MBP.A1*MBP.E]);
    MBP.Kb1=diag([MBP.E*MBP.I1 MBP.E*MBP.I1 2*MBP.G*MBP.I1]);
    MBP.Ke2=4*diag([3e8 3e8 MBP.A2*MBP.E]);
    MBP.Kb2=4*diag([MBP.E*MBP.I2 MBP.E*MBP.I2 2*MBP.G*MBP.I2]);
    MBP.zeta = 0.2;
    MBP.L1o = MBP.L1;
    MBP.Lo = MBP.L;
    MBP.Ls=0;
    MBP.K1 = 0;
    MBP.K2 = 0;
end

if(l<MBP.L1)
    MBP.L1o=l;
    MBP.Lo = MBP.L - l + MBP.L1;
    MBP.Ls=0;
else
    MBP.Ls=l-MBP.L1;
end
dGamma = zeros(4,6);
Ell=zeros(4,4);
Theta=zeros(6,4);
K=zeros(6,6);

dGamma(1:4,1:3)=(MBP.L1o+MBP.Ls*MBP.zeta)*MBP.Q1';
dGamma(3:4,4:6)=MBP.L2*MBP.Q2';

Ell(1:2,1:2)=diag([1 1])*(MBP.L1+MBP.L);
Ell(3:4,3:4)=diag([1 1])*(MBP.L2+MBP.Lr+MBP.L1+MBP.L);

Theta(1:3,1:4)=-MBP.A1*MBP.E*MBP.Q1;
Theta(1:3,3:4)=-MBP.A1*MBP.E*MBP.Q2*0;
Theta(4:6,3:4)=-MBP.A2*MBP.E*MBP.Q2;

K(1:3,1:3)=4*MBP.Kb1+4*MBP.Kb2+MBP.K1*MBP.Kb1;
K(1:3,4:6)=-4*MBP.Kb2-MBP.K2*MBP.Kb1;
K(4:6,4:6)=4*MBP.Kb2+MBP.K2*MBP.Kb1;

G=dGamma-Ell*pinv(Theta)/2*K;

qa=G*u;
end
