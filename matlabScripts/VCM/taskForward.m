function [R,p] = taskForward(psi,Tg)
%=====forward kinematics considering coupled bending of the stem
%=====psi = {phi, L0, theta1, delta1, theta2, delta2}
%-----mechanics based coupled bending relation
E=50e9;I=pi*(0.37e-3)^4/64;E0=190e9;L1=39e-3;Lr=4e-3;L2=25e-3;Lg=3.5e-3;
%bending stiffness of notch patterns
q=2.0e-3/2;q1=0.5e-3;do=2.7e-3;di=2.46e-3;h=1.3e-3;
%refer to 'cross_section_cal.m' and Needle(ICRA) paper script
Ia=0.0048*(1e-3)^4;
Ib=0.8110*(1e-3)^4;
Ic=0.0741*(1e-3)^4;
K0=1/(q1/2/q/E0/Ia+(q-q1)/q/E0/Ib+q1/2/q/E0/Ic);
phi=psi(1);L0=psi(2);theta1=psi(3);delta1=psi(4);theta2=psi(5);delta2=psi(6);
gamma=(8*E*I+K0)/(8*E*I);
gamma=1/0.62;
theta0=L0/L1/gamma*theta1;

%-----forward kinematics config space
if(theta1~=0)
%p10_old=[cos(delta1)*(L0/theta0*(1-cos(theta0))+L1/theta1*(cos(theta0)*(1-cos(theta1))+sin(theta0)*sin(theta1))); ...
%        -sin(delta1)*(L0/theta0*(1-cos(theta0))+L1/theta1*(cos(theta0)*(1-cos(theta1))+sin(theta0)*sin(theta1))); ...
%         (L0/theta0*sin(theta0)+L1/theta1*(-sin(theta0)*(1-cos(theta1))+cos(theta0)*sin(theta1)))];
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
p=Rphi*(p10+R10*p21+R10*R21*pg2);
R=Rphi*R10*R21;

%-----plots
global Posi_1 Ori_1;
Posi_1=zeros(3,1);Ori_1=Rphi;
figure(1);hold on;grid on;%cla;
PlotSnake(delta1,theta0,L0,[1.45e-3 1000]);
PlotSnake(delta1,theta1,L1,[1.45e-3 1000]);
PlotSnake(0,0,Lr,[1.45e-3 1000]);
PlotSnake(delta2,theta2,L2,[1.45e-3 1000]);
PlotSnake(0,0,Lg,[1.45e-3 1000]);
PlotAxis(0.01);
if(nargin==2)
    Posi_1=Tg(1:3,4);
    Ori_1=Tg(1:3,1:3);
    PlotAxis(0.01);
end
axis equal;axis([-0.06 0.06 -0.06 0.06 0 0.15])
end