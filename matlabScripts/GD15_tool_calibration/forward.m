function [kmp]=forward(psi,seg,zeta,d,ga)
L1x=0;
%=====psi = {l, phi, theta1, delta1, theta2, delta2}
l=psi(1)+d+L1x;
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
Lg=seg(4);
if(L1<1e-3)
    Theta0 = 0;
    Theta1 = 0;
else
    Theta0 = theta1*L0*zeta/(L1+L0*zeta);
    Theta1 = theta1*L1/(L1+L0*zeta);
end

kmp.PO_sb = [0 0 -d]';
kmp.RO_sb = Expm([0 0 phi+ga/180*pi]');

kmp.Psb_1b = calcSegP(Theta0,delta1,L0);
kmp.Rsb_1b = calcSegR(Theta0,delta1);

kmp.P1b_1e = calcSegP(Theta1,delta1,L1);
kmp.R1b_1e = calcSegR(Theta1,delta1);

kmp.P1e_2b = [0 0 Lr]';
kmp.R1e_2b = eye(3);

kmp.P2b_2e = calcSegP(theta2,delta2,L2);
kmp.R2b_2e = calcSegR(theta2,delta2);

kmp.P2e_g = [0 0 Lg]';
kmp.R2e_g = eye(3);


kmp.Psb_g_in_sb = kmp.Psb_1b+kmp.Rsb_1b*(kmp.P1b_1e+(kmp.R1b_1e*(kmp.P1e_2b+kmp.R1e_2b*(kmp.P2b_2e+kmp.R2b_2e*kmp.P2e_g))));
kmp.P1e_g_in_sb = kmp.Rsb_1b*kmp.R1b_1e*(kmp.P1e_2b+kmp.R1e_2b*(kmp.P2b_2e+kmp.R2b_2e*kmp.P2e_g));
kmp.P2e_g_in_2b = kmp.R2b_2e*kmp.P2e_g;

kmp.PO_g = kmp.PO_sb + kmp.RO_sb*(kmp.Psb_1b+kmp.Rsb_1b*(kmp.P1b_1e+kmp.R1b_1e*(kmp.P1e_2b+kmp.R1e_2b*(kmp.P2b_2e+kmp.R2b_2e*kmp.P2e_g))));
kmp.RO_g = kmp.RO_sb*kmp.Rsb_1b*kmp.R1b_1e*kmp.R1e_2b*kmp.R2b_2e*kmp.R2e_g;
end