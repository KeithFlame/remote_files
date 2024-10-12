function psi = Curvature2Psi_keith(u,SL,l)
% this is afunction to get configuration variables of a 2-seg continuum 
% manipulator form a known curvature and several structural parameters.
% 
% input1: u curvature of two continuum segment (6X1 vector, 1/m)
% input2: structural length SL (4 X 1 vector, L1 Lr L2 Lg) (mm)
%                           or (10 X 1 vector additional zeta K1 K2 gamma1
%                           Lstem gamma3) (mm, rad, or dimensionless)
% input3: l the feed length (mm)
%
% output: psi (6 X 1 vector, phi L theta1 delta1 theta2 delta2) (mm, rad)
%
% Author: Keith W.
% Ver. 1.0
% Date: 15.02.2022

if(length(SL)<5)
    zeta = 0.2;
else
    zeta = SL(5);
end
L1=SL(1)*1e-3;
L2=SL(3)*1e-3;
psi=zeros(6,1);
l = l*1e-3;

u_=u;
if(l<L1)
    L1=l;
    psi(3)=L1*norm(u_(1:3));
else
    psi(3)=(L1+zeta*(l-L1))*norm(u_(1:3));
end
psi(4)=-pi/2+atan2(u_(2),u_(1));
psi(5)=L2*norm(u_(4:6));
psi(6)=-pi/2+atan2(u_(5),u_(4));

end


