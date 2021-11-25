function [J]=getJacobianCoupled(psi)
%=====psi = {phi, L0, theta1, delta1, theta2, delta2}
phi=psi(1);L0=psi(2);theta1=psi(3);delta1=psi(4);theta2=psi(5);delta2=psi(6);
%====mechanics based coupling effects:parameters
E=50e9;I=pi*(0.37e-3)^4/64;E0=190e9;L1=39e-3;Lr=4e-3;L2=25e-3;Lg=3.5e-3;
%bending stiffness of notch patterns
q=2.0e-3/2;q1=0.5e-3;do=2.7e-3;di=2.46e-3;h=1.3e-3;
%refer to 'cross_section_cal.m' and Needle(ICRA) paper script
Ia=0.0048*(1e-3)^4;Ib=0.8110*(1e-3)^4;Ic=0.0741*(1e-3)^4;
K0=1/(q1/2/q/E0/Ia+(q-q1)/q/E0/Ib+q1/2/q/E0/Ic);
gamma=(8*E*I+K0)/(8*E*I);
theta0=L0/L1/gamma*theta1;

%----Jacobians
if(theta1~=0)
dxdL0=cos(delta1)/gamma*(sin(theta0)*(gamma-1+cos(theta1))+cos(theta0)*sin(theta1));
dxdtheta1=-cos(delta1)*L1/theta1^2*(gamma+cos(theta0)*(1-gamma-cos(theta1))+sin(theta0)*sin(theta1)) ...
         +cos(delta1)*L1/theta1*(L0/gamma/L1*sin(theta0)*(gamma-1+cos(theta1))+sin(theta0)*cos(theta1)+(1+L0/gamma/L1)*cos(theta0)*sin(theta1));
dxddelta1=-sin(delta1)*L1/theta1*(gamma+cos(theta0)*(1-gamma-cos(theta1))+sin(theta0)*sin(theta1));

dydL0=-sin(delta1)/gamma*(sin(theta0)*(gamma-1+cos(theta1))+cos(theta0)*sin(theta1));
dydtheta1=sin(delta1)*L1/theta1^2*(gamma+cos(theta0)*(1-gamma-cos(theta1))+sin(theta0)*sin(theta1)) ...
         -sin(delta1)*L1/theta1*(L0/gamma/L1*sin(theta0)*(gamma-1+cos(theta1))+sin(theta0)*cos(theta1)+(1+L0/gamma/L1)*cos(theta0)*sin(theta1));
dyddelta1=-cos(delta1)*L1/theta1*(gamma+cos(theta0)*(1-gamma-cos(theta1))+sin(theta0)*sin(theta1));

dzdL0=1/gamma*(-sin(theta0)*sin(theta1)+cos(theta0)*(cos(theta1)+gamma-1));
dzdtheta1=-L1/theta1^2*(cos(theta0)*sin(theta1)+sin(theta0)*(cos(theta1)+gamma-1)) ...
         +L1/theta1*(L0/gamma/L1*cos(theta0)*(gamma-1+cos(theta1))+cos(theta0)*cos(theta1)-(1+L0/gamma/L1)*sin(theta0)*sin(theta1));
dzddelta1=0;
else
dxdL0=0;
dxdtheta1=cos(delta1)*L1*(L0^2/L1^2/gamma+2*L0/gamma/L1+1)/2;
dxddelta1=0;
dydL0=0;
dydtheta1=-sin(delta1)*L1*(L0^2/L1^2/gamma+2*L0/gamma/L1+1)/2;
dyddelta1=0;
dzdL0=1;
dzdtheta1=0;
dzddelta1=0;
end
J1v=[dxdL0 dxdtheta1 dxddelta1;dydL0 dydtheta1 dyddelta1;dzdL0 dzdtheta1 dzddelta1];
J1w=[theta1/gamma/L1*sin(delta1) (1+L0/gamma/L1)*sin(delta1) cos(delta1)*sin(theta0+theta1); ...
     theta1/gamma/L1*cos(delta1) (1+L0/gamma/L1)*cos(delta1) -sin(delta1)*sin(theta0+theta1); ...
     0 0 cos(theta0+theta1)-1];

if(theta2~=0)
J2v=L2*[cos(delta2)*(sin(theta2)/theta2+(cos(theta2)-1)/theta2^2) sin(delta2)*(cos(theta2)-1)/theta2; ...
       -sin(delta2)*(sin(theta2)/theta2+(cos(theta2)-1)/theta2^2) cos(delta2)*(cos(theta2)-1)/theta2; ...
       cos(theta2)/theta2-sin(theta2)/theta2^2 0];
J2w=[sin(delta2) cos(delta2)*sin(theta2); ...
     cos(delta2) -sin(delta2)*sin(theta2); ...
     0 cos(theta2)-1];
else
J2v=[L2*cos(delta2)/2 0;-L2*sin(delta2)/2 0;0 0];
J2w=[sin(delta2) 0;cos(delta2) 0;0 0];
end
Jphi = [0 0 1]';

%----forward kinematics configs
if(theta1~=0)
p10=[cos(delta1)*L1/theta1*(gamma+cos(theta0)*(1-gamma-cos(theta1))+sin(theta0)*sin(theta1)); ...
    -sin(delta1)*L1/theta1*(gamma+cos(theta0)*(1-gamma-cos(theta1))+sin(theta0)*sin(theta1)); ...
     L1/theta1*(cos(theta0)*sin(theta1)+sin(theta0)*(cos(theta1)+gamma-1))];
else
p10=[0 0 L0+L1]';
end
R10=expm(S([0 0 -delta1]'))*expm(S([0 theta1+theta0 0]'))*expm(S([0 0 delta1]'));
if(theta2~=0)
p21=L2/theta2*[cos(delta2)*(1-cos(theta2));sin(delta2)*(cos(theta2)-1);sin(theta2)]+[0 0 Lr]';
else
p21=[0 0 L2+Lr]';
end
R21=expm(S([0 0 -delta2]'))*expm(S([0 theta2 0]'))*expm(S([0 0 delta2]'));

Rphi=expm(S([0 0 phi]'));
pg2=[0 0 Lg]';

%---combining Jacobians
W1 = -S(Rphi*(p10+R10*p21+R10*R21*pg2))*Jphi;
W2 = -S(R10*(p21+R21*pg2))*J1w+J1v;
W3 = -S(R21*pg2)*J2w+J2v;
J=[W1 Rphi*W2 Rphi*R10*W3; Jphi Rphi*J1w Rphi*R10*J2w];

end