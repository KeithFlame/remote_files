function [T_end,s]=plotSeg(l,THETA,DELTA,T_base)


scaler=0:0.1:1;
if THETA==0 ||l==0
    T=[1 0 0 0
        0 1 0 0
        0 0 1 l
        0 0 0 1];
    s=T_base*[0*scaler
        0*scaler
        l*scaler
        ones(1,length(scaler))];s=s([1 2 3],:);%Segment1 straight
else
    k=THETA/l;
    cosTHETA1=cos(THETA);sinTHETA1=sin(THETA);cosDELTA1=cos(DELTA);sinDELTA1=sin(DELTA);
    T=[(cosDELTA1)^2*(cosTHETA1-1)+1 sinDELTA1*cosDELTA1*(cosTHETA1-1) cosDELTA1*sinTHETA1 cosDELTA1*(1-cosTHETA1)/k
        sinDELTA1*cosDELTA1*(cosTHETA1-1) (cosDELTA1)^2*(1-cosTHETA1)+cosTHETA1 sinDELTA1*sinTHETA1 sinDELTA1*(1-cosTHETA1)/k
        -cosDELTA1*sinTHETA1 -sinDELTA1*sinTHETA1 cosTHETA1 sinTHETA1/k
        0 0 0 1];
    s=T_base*[cosDELTA1*(1-cos(scaler*THETA))/k;
        sinDELTA1*(1-cos(scaler*THETA))/k;
        sin(scaler*THETA)/k;
        ones(1,length(scaler))];s=s([1 2 3],:);%Segment2 bent
end
T_end=T_base*T;
end