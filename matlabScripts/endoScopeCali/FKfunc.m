function [T,T1,T2,T3,T4,T5,s1,s2,s3,s4,s5]=FKfunc(config)
L1=config(4);Lr=config(6);L2=config(8);Lg=config(10);
scaler=0:0.1:1;
PHI=config(1);
Ls=config(2);
THETA1=config(3);
DELTA1=config(5);%+config(1);
THETA2=config(7);
DELTA2=config(9);
k2=THETA2/L2;
k1=THETA1/L1;

%Ls
T1=[cos(PHI) -sin(PHI) 0 0
    sin(PHI) cos(PHI) 0 0
    0 0 1 Ls
    0 0 0 1];
s1=[0 0;0 0;0 Ls];

%segment1 circular
if THETA1==0
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 L1
        0 0 0 1];
    s2=T1*[0*scaler
        0*scaler
        L1*scaler
        ones(1,length(scaler))];s2=s2([1 2 3],:);%Segment1 straight
else
    cosTHETA1=cos(THETA1);sinTHETA1=sin(THETA1);cosDELTA1=cos(DELTA1);sinDELTA1=sin(DELTA1);
    T2=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k1
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k1
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k1
        0 0 0 1];
    s2=T1*[cosDELTA1*(1-cos(scaler*THETA1))/k1;
        sinDELTA1*(1-cos(scaler*THETA1))/k1;
        sin(scaler*THETA1)/k1;
        ones(1,length(scaler))];s2=s2([1 2 3],:);%Segment2 bent
end

%Lr
T3=[1 0 0 0
    0 1 0 0
    0 0 1 Lr
    0 0 0 1];
s3=T1*T2*[0 0;0 0;0 Lr;1 1];s3=s3([1 2 3],:);

%segment2 circular
if THETA2==0
    T4=[1 0 0 0
        0 1 0 0
        0 0 1 L2
        0 0 0 1];
    s4=T1*T2*T3*[0*scaler
        0*scaler
        L2*scaler
        ones(1,length(scaler))];s4=s4([1 2 3],:);%Segment2 straight
else
    cosTHETA2=cos(THETA2);sinTHETA2=sin(THETA2);cosDELTA2=cos(DELTA2);sinDELTA2=sin(DELTA2);
    T4=[(cosDELTA2)^2*(cosTHETA2-1)+1 sinDELTA2*cosDELTA2*(cosTHETA2-1) cosDELTA2*sinTHETA2 cosDELTA2*(1-cosTHETA2)/k2
        sinDELTA2*cosDELTA2*(cosTHETA2-1) (cosDELTA2)^2*(1-cosTHETA2)+cosTHETA2 sinDELTA2*sinTHETA2 sinDELTA2*(1-cosTHETA2)/k2
        -cosDELTA2*sinTHETA2 -sinDELTA2*sinTHETA2 cosTHETA2 sinTHETA2/k2
        0 0 0 1];
    s4=T1*T2*T3*[cosDELTA2*(1-cos(scaler*THETA2))/k2;
        sinDELTA2*(1-cos(scaler*THETA2))/k2;
        sin(scaler*THETA2)/k2;
        ones(1,length(scaler))];s4=s4([1 2 3],:);%Segment2 bent
end

%end effector
T5=[1 0 0 0
    0 1 0 0
    0 0 1 Lg
    0 0 0 1];
s5=T1*T2*T3*T4*[0 0;0 0;0 Lg;1 1];s5=s5([1 2 3],:);

T=T1*T2*T3*T4*T5;
end%R=T([1 2 3],[1 2 3]);p=T([1 2 3],4);