function [Tend,S]=FKcc_2segs_nobending_keith(psi, SL,discrete_number)
% Declaration
% the end-effector coordinate {g}.
% the base coordinate {b}
%
% This is a function to calculate the pose of {g} in {b} for a 2-seg continuum
% manipulator with the constant curvature assumption.
%
% input1: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg)
%                             or (7 X 1 vector additional gamma1 gamma2 gamma3)
% output1: Tend (4 X 4 matrix)
% output2: S (discrete points on the central curve, N X 4 matrix, last column
%             denotes the curvature on relative point)
%
% Author: Keith W.
% Ver. 1.0
% Date: 09.02.2022

if(nargin == 2)
    discrete_number = 20;
end
L1=SL(1);Lr=SL(2);L2=SL(3);Lg=SL(4);
if(max(size(SL))<5)
    gamma1 = 0;gamma3 = 0;
else
    gamma1 = SL(5);gamma3 = SL(7);
end

scalar=0:(1/discrete_number):1;
PHI=psi(1);
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
T1=[cos(PHI+gamma1) -sin(PHI+gamma1) 0 0
sin(PHI+gamma1) cos(PHI+gamma1) 0 0
0 0 1 Ls
0 0 0 1];
s1=[0 0;0 0;0 Ls;1 1];s1(4,:)=[0 0];

%segment1 circular
if theta1==0
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 L1
        0 0 0 1];
    s2=T1*[0*scalar
        0*scalar
        L1*scalar
        ones(1,length(scalar))];s2(4,:)=zeros(1,length(scalar));%Segment1 straight
else
    cosTHETA1=cos(theta1);sinTHETA1=sin(theta1);cosDELTA1=cos(delta1);sinDELTA1=sin(delta1);
    T2=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
        0 0 0 1];
    s2=T1*[cosDELTA1*(1-cos(scalar*theta1))/k1;
        sinDELTA1*(1-cos(scalar*theta1))/k1;
        sin(scalar*theta1)/k1;
        ones(1,length(scalar))];s2(4,:)=ones(1,length(scalar))*k1;%Segment1 bent
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
    s4=T1*T2*T3*[0*scalar
        0*scalar
        L2*scalar
        ones(1,length(scalar))];s4(4,:)=zeros(1,length(scalar));
else
    cosTHETA2=cos(theta2);sinTHETA2=sin(theta2);cosDELTA2=cos(delta2);sinDELTA2=sin(delta2);
    T4=[(cosDELTA2)^2*(cosTHETA2-1)+1 sinDELTA2*cosDELTA2*(cosTHETA2-1) cosDELTA2*sinTHETA2 cosDELTA2*(1-cosTHETA2)/k2
        sinDELTA2*cosDELTA2*(cosTHETA2-1) (cosDELTA2)^2*(1-cosTHETA2)+cosTHETA2 sinDELTA2*sinTHETA2 sinDELTA2*(1-cosTHETA2)/k2
        -cosDELTA2*sinTHETA2 -sinDELTA2*sinTHETA2 cosTHETA2 sinTHETA2/k2
        0 0 0 1];
    s4=T1*T2*T3*[cosDELTA2*(1-cos(scalar*theta2))/k2;
        sinDELTA2*(1-cos(scalar*theta2))/k2;
        sin(scalar*theta2)/k2;
        ones(1,length(scalar))];s4(4,:)=ones(1,length(scalar))*k2;%Segment2 bent
end

%end effector
T5=[cos(gamma3) -sin(gamma3) 0 0
    sin(gamma3) cos(gamma3) 0 0
    0 0 1 Lg
    0 0 0 1];
s5=T1*T2*T3*T4*[0 0;0 0;0 Lg;1 1];s5(4,:)=[0 0];
Tend=T1*T2*T3*T4*T5;
S=[s1 s2 s3 s4 s5]';
end
