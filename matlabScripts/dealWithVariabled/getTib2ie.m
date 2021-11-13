function [T, s]=getTib2ie(theta,delta,l)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.11.2021
% Ver. 1.0
% purpose: calc one segment (continuum) pose.
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%%
    if(theta==0)
        T=eye(4);
        T(3,4)=l;
        s=[0,0;0,0;0,l;1,1];
        return;
    end

    scaler=0:0.1:1;
    k=theta/l;
    cosT=cos(theta);sinT=sin(theta);cosD=cos(delta);sinD=sin(delta);
    T=[(cosD)^2*(cosT-1)+1 sinD*cosD*(cosT-1) cosD*sinT cosD*(1-cosT)/k
        sinD*cosD*(cosT-1) (cosD)^2*(1-cosT)+cosT sinD*sinT sinD*(1-cosT)/k
        -cosD*sinT -sinD*sinT cosT sinT/k
        0 0 0 1];
    s=[cosD*(1-cos(scaler*theta))/k;
        sinD*(1-cos(scaler*theta))/k;
        sin(scaler*theta)/k;
        ones(1,length(scaler))];%Seg2 
end