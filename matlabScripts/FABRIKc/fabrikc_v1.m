% this is a script for FABRIKc algerithm.
%
% Author Keith W.
% Ver. 1.0
% Date 08.17.2022

rpy = randn(1,3);
t = [40 40 100]';
target = [eul2rotm(rpy),t;[0 0 0 1]];


segment = setSegment;
l = segment([1 3 5])/2;
L = 2 * l;
zee = target(1:3,3);
ze0 = [0;0;1];
pj = zeros(3,4);
pj(3,:) = [l(1), L(1) + l(2), L(1) + L(2) + l(3), L(1) + L(2) + L(3)];
ep = 0.01;
err = 1;
t0 = [0; 0; 0];
while (err>ep)
   %backward
   
   [pv,pb,l, zb] = getFabPara(zee,t,pj(:,2),l(3),segment(5),1);
   l(3) = l;
   pj(:,3) = pv;
   [pv,pb,l, zb] = getFabPara(zb,pj(:,1),pb,l(2),segment(3),1);
   l(2) = l;
   pj(:,2) = pv;
   [pv,pb,l, zb] = getFabPara(zb,t0,pb,l(1),segment(1),1);
   l(1) = l;
   pj(:,1) = pv;

   %forward
   t0 = [0; 0; pb(3)];
   [pv,pb,l, zb] = getFabPara(ze0,t0,pj(:,2),l(1),segment(5),-1);
   pj(:,1) = pv;
   l(1) = l;
   [pv,pb,l, zb] = getFabPara(zb,pb,pj(:,1),l(1),segment(5),-1);
   pj(:,2) = pv;
   l(2) = l;
   [pv,pb,l, zb] = getFabPara(zb,pb,t,l(1),segment(5),-1);
   pj(:,3) = pv;
   l(3) = l;
end


function [pv,pb, l, zb] = getFabPara(ze,pje,pvf,l,L, flag)
    pv = pje - l * ze*flag;    
    zb = (pv - pvf)/norm(pv - pvf);
    xita = acos(zb'*ze);
    if xita == 0
        l = L/2;
    else
        l = L/xita * tan(xita/2);
    end
    
    pv = pje - l * ze*flag;
    pb = pv - l *zb*flag;

end

function segment = setSegment
    %  Ls L1 Lr L2 Lg
    segment = [100 0 100 0 100 0];
end