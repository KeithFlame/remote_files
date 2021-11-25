function [qa]=calcActuation_endo(psi,MP)
%---------constant curvature model for unloaded robot--------------------%
% calculating actuation lengths from configs
%----Info
% By Yuyang Chen
% Date 20200610
% Ver c1.2
%-------------------------------------------------------------------------%
theta1=psi(3);delta1=psi(4);theta2=psi(5);delta2=psi(6);%the config used in this file is the conventional one, while the input is from actual one
if(nargin == 1)
    e1=[1 0 0]';e2=[0 1 0]';e3=[0 0 1]';
    MP.E=55e9;%Rod Young's modules
    MP.mu=0.33;%Rod Poisson rate
    MP.G=MP.E/2/(1+MP.mu);
    MP.d2=0.4e-3;%Rod diameter
    MP.d1=0.5e-3;
    MP.rho=0.85e-3;%Rod pitch circle radius
    MP.L=500e-3;%Robot stem length
    MP.L1=37.5e-3;%Robot seg1 length
    MP.L2=25e-3;%Robot seg2 length
    MP.Lr=2.5e-3;%Robot rigid seg length
    MP.Lg=5.0e-3;%Robot gipper length
    MP.r11=[1 0 0]'*MP.rho;MP.r12=[0 1 0]'*MP.rho;
    MP.r21=[1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;MP.r22=[-1/sqrt(2) 1/sqrt(2) 0]'*MP.rho;
    MP.Q1=[S(MP.r11)*e3 S(MP.r12)*e3 S(MP.r21)*e3 S(MP.r22)*e3];
    MP.Q2=[S(MP.r21)*e3 S(MP.r22)*e3];
    MP.I1=pi*MP.d1^4/64;MP.A1=pi*MP.d1^2/4;
    MP.I2=pi*MP.d2^4/64;MP.A2=pi*MP.d2^2/4;%MP.G=MP.E/2/(1+MP.mu);MP.J=2*MP.I;
    MP.Ke1=diag([MP.G*MP.A1 MP.G*MP.A1 MP.A1*MP.E]);
    MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
    MP.Ke2=diag([MP.G*MP.A2 MP.G*MP.A2 MP.A2*MP.E]);
    MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
end
u1=[theta1/MP.L1*cos(pi/2-delta1) theta1/MP.L1*sin(pi/2-delta1) 0]';
u2=[theta2/MP.L2*cos(pi/2-delta2) theta2/MP.L2*sin(pi/2-delta2) 0]';
%---- for non-coupled stem-seg1 bending
qa=([MP.L1*MP.Q1(:,1:2)' zeros(2,3);MP.L1*MP.Q1(:,3:4)' MP.L2*MP.Q1(:,3:4)']- ...
    2*diag([1/MP.A1/MP.E,1/MP.A1/MP.E,1/MP.A2/MP.E,1/MP.A2/MP.E])*[(MP.L+MP.L1)*eye(2) zeros(2,2);zeros(2,2) (MP.L+MP.L1+MP.Lr+MP.L2)*eye(2)]*...
    [-pinv(MP.Q1(:,1:2))*(MP.Kb1+MP.Kb2) pinv(MP.Q1(:,1:2))*MP.Kb2;zeros(2,3) -pinv(MP.Q1(:,3:4))*MP.Kb2]) *[u1;u2];
%---- for coupled stem-seg1 bending
%gamma = 10.2078;
%d=50e-3;
%qa =([(MP.L1+d/gamma)*MP.Q1(:,1:2)' zeros(2,3);(MP.L1+d/gamma)*MP.Q1(:,3:4)' MP.L2*MP.Q1(:,3:4)']- ...
%    2/MP.A/MP.E*[(MP.L+MP.L1)*eye(2) zeros(2,2);zeros(2,2) (MP.L+MP.L1+MP.Lr+MP.L2)*eye(2)]*...
%    [-2*pinv(MP.Q1(:,1:2))*MP.Kb pinv(MP.Q1(:,1:2))*MP.Kb;zeros(2,3) -pinv(MP.Q1(:,3:4))*MP.Kb]) *[u1;u2];

    

end