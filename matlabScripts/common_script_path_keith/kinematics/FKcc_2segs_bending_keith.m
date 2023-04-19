function [Tend,S]=FKcc_2segs_bending_keith(psi, SL,discrete_element)
% Declaration
% the end-effector coordinate {g}.
% the base coordinate {b}
%
% This is a function to calculate the pose of {g} in {b} for a 2-seg continuum
% manipulator with the constant curvature assumption.
%
% input1: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg)
%                             or (8 X 1 vector additional gamma1 gamma2 gamma3 zeta)
% output1: Tend (4 X 4 matrix)
% output2: S (discrete points on the central curve, N X 4 matrix, last column
%             denotes the curvature on relative point)
%
% Author: Keith W.
% Ver. 1.0
% Date: 09.02.2022

if(nargin == 2)
    discrete_element = 1;
end
L1=SL(1);Lr=SL(2);L2=SL(3);Lg=SL(4);
if(max(size(SL))<5)
    gamma1 = 0;gamma3 = 0;zeta = 0.2;
else
    gamma1 = SL(8);gamma3 = 0;zeta = SL(5);
end
n1 = ceil(L1/discrete_element);
n2 = ceil(L2/discrete_element);
scalar1=0:(1/n1):1;
scalar2=0:(1/n2):1;
phi=psi(1);
l=psi(2);
theta1=psi(3);
delta1=psi(4);
theta2=psi(5);
delta2=psi(6);

if(l<L1)
    L1=l;
    Ls=0;
else
    Ls=l-L1;
end

k2=theta2/L2;
k1=theta1/L1;

%segment0 circular
n0 = ceil(Ls/discrete_element);
if theta1==0 || Ls<0.0001 ||zeta==0
    T1=[cos(phi+gamma1) -sin(phi+gamma1) 0 0
    sin(phi+gamma1) cos(phi+gamma1) 0 0
    0 0 1 Ls
    0 0 0 1];
    if(n0>0)
        s1=[zeros(1,n0);zeros(1,n0);linspace(0, Ls,n0);ones(1,n0)];s1(4,:)=zeros(1,n0);
    else
        s1=[0 0 0 0]';
    end
else
    theta0=theta1*zeta*Ls/(zeta*Ls+L1);
    theta1=theta1*L1/(zeta*Ls+L1);
    k0=theta0/Ls;
    k1=theta1/L1;
    scalar0=0:(1/n0):1;
    cosTHETA0=cos(theta0);sinTHETA0=sin(theta0);cosDELTA0=cos(delta1);sinDELTA0=sin(delta1);
    T1=[cos(phi+gamma1) -sin(phi+gamma1) 0 0
    sin(phi+gamma1) cos(phi+gamma1) 0 0
    0 0 1 0
    0 0 0 1]*...
    [(cosDELTA0)^2*(cosTHETA0-1)+1 sinDELTA0*cosDELTA0*(cosTHETA0-1) cosDELTA0*sinTHETA0 cosDELTA0*(1-cosTHETA0)/k0
    sinDELTA0*cosDELTA0*(cosTHETA0-1) (cosDELTA0)^2*(1-cosTHETA0)+cosTHETA0 sinDELTA0*sinTHETA0 sinDELTA0*(1-cosTHETA0)/k0
    -cosDELTA0*sinTHETA0 -sinDELTA0*sinTHETA0 cosTHETA0 sinTHETA0/k0
    0 0 0 1];
    s1=[cos(phi) -sin(phi) 0 0
        sin(phi) cos(phi) 0 0
        0 0 1 0
        0 0 0 1]*...
    [cosDELTA0*(1-cos(scalar0*theta0))/k0;
        sinDELTA0*(1-cos(scalar0*theta0))/k0;
        sin(scalar0*theta0)/k0;
        ones(1,length(scalar0))];s1(4,:)=ones(1,length(scalar0))*k0;%Segment0 bent
end

%segment1 circular
if theta1==0
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 L1
        0 0 0 1];
    s2=T1*[0*scalar1
        0*scalar1
        L1*scalar1
        ones(1,length(scalar1))];s2(4,:)=zeros(1,length(scalar1));%Segment1 straight
else
    cosTHETA1=cos(theta1);sinTHETA1=sin(theta1);cosDELTA1=cos(delta1);sinDELTA1=sin(delta1);
    T2=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
        0 0 0 1];
    s2=T1*[cosDELTA1*(1-cos(scalar1*theta1))/k1;
        sinDELTA1*(1-cos(scalar1*theta1))/k1;
        sin(scalar1*theta1)/k1;
        ones(1,length(scalar1))];s2(4,:)=ones(1,length(scalar1))*k1;%Segment1 bent
end

%Lr
T3=[1 0 0 0
    0 1 0 0
    0 0 1 Lr
    0 0 0 1];
s3=T1*T2*[0 0;0 0;0 Lr;1 1];s3(4,:)=[0 0];

%segment2 circular
if theta2==0
    T4=[1 0 0 0
        0 1 0 0
        0 0 1 L2
        0 0 0 1];
    s4=T1*T2*T3*[0*scalar2
        0*scalar2
        L2*scalar2
        ones(1,length(scalar2))];s4(4,:)=zeros(1,length(scalar2));
else
    cosTHETA2=cos(theta2);sinTHETA2=sin(theta2);cosDELTA2=cos(delta2);sinDELTA2=sin(delta2);
    T4=[(cosDELTA2)^2*(cosTHETA2-1)+1 sinDELTA2*cosDELTA2*(cosTHETA2-1) cosDELTA2*sinTHETA2 cosDELTA2*(1-cosTHETA2)/k2
        sinDELTA2*cosDELTA2*(cosTHETA2-1) (cosDELTA2)^2*(1-cosTHETA2)+cosTHETA2 sinDELTA2*sinTHETA2 sinDELTA2*(1-cosTHETA2)/k2
        -cosDELTA2*sinTHETA2 -sinDELTA2*sinTHETA2 cosTHETA2 sinTHETA2/k2
        0 0 0 1];
    s4=T1*T2*T3*[cosDELTA2*(1-cos(scalar2*theta2))/k2;
        sinDELTA2*(1-cos(scalar2*theta2))/k2;
        sin(scalar2*theta2)/k2;
        ones(1,length(scalar2))];s4(4,:)=ones(1,length(scalar2))*k2;%Segment2 bent
end

%end effector
T5=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 Lg
    0 0 0 1];
s5=T1*T2*T3*T4*[0 0;0 0;0 Lg;1 1];s5(4,:)=[0 0];
Tend=T1*T2*T3*T4*T5;
S=[s1 s2(:,2:end) s3(:,2:end) s4(:,2:end) s5(:,2:end)]';
end
