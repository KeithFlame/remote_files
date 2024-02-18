function [J]=calcJacobian(psi,seg,zeta,d,ga)
L1x=0;
%=====psi = {l, phi, theta1, delta1, theta2, delta2}
l=psi(1)+d;
phi=psi(2);
theta1=psi(3);
delta1=psi(4);
theta2=psi(5);
delta2=psi(6);
%====mechanics based coupling effects:parameters
length = [max(0,l-seg(1)-seg(2)-seg(3)) ...
          min(max(0,l-seg(2)-seg(3)),seg(1)) ...
          min(max(0,l-seg(3)),seg(2)) ...
          min(l,seg(3))]';
L0=length(1);
L1=length(2);
Lr=length(3);
L2=length(4);


Jphi = [0 0 1]';
if(L1>0)
    JvL = JvLCouple(theta1,delta1,L1,L0,zeta);
else
    JvL = JvLCouple(theta2,delta2,L2,Lr,0);
end
JwL = zeros(3,1);
JvSeg1 = JvSegCouple(theta1,delta1,L1,L0,zeta);
JwSeg1 = JwSegCouple(theta1,delta1,L1,L0,zeta);
JvSeg2 = JvSegCouple(theta2,delta2,L2,0,0);
JwSeg2 = JwSegCouple(theta2,delta2,L2,0,0);

kmp=forward(psi,seg,zeta,d,ga);
J(1:3,2) = kmp.RO_sb*(-S( kmp.Psb_g_in_sb )*Jphi);
J(4:6,2) = Jphi;
J(1:3,1) = kmp.RO_sb*(-S( kmp.P1e_g_in_sb )*JwL+JvL);
J(4:6,1) = kmp.RO_sb*JwL;
J(1:3,3:4) = kmp.RO_sb*(-S( kmp.P1e_g_in_sb )*JwSeg1 + JvSeg1);
J(4:6,3:4) = kmp.RO_sb*JwSeg1;
J(1:3,5:6) = kmp.RO_sb*kmp.Rsb_1b*kmp.R1b_1e*kmp.R1e_2b*(-S( kmp.P2e_g_in_2b )*JwSeg2 + JvSeg2);
J(4:6,5:6) = kmp.RO_sb*kmp.Rsb_1b*kmp.R1b_1e*kmp.R1e_2b*JwSeg2;
end