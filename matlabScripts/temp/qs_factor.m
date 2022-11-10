% constant curvature model for static
%% init para
L1 = 0.100;
Lr = 0.010;
L2 = 0.020;
Lg = 0.015;
E = 50e9;
d1 = 0.95e-3;
d2 = 0.4e-3;
d3 = 0.95e-3;
rho1 = 2.5e-3;
rho2 = 2.7e-3;
rho3 = 12e-3;
A1 = pi * d1 ^2/4;
A2 = pi * d2 ^2/4;
A3 = pi * d3 ^2/4;
I1 = pi * d1^4 / 64;
I2 = pi * d2^4 / 64;
I3 = pi * d3^4 / 64;
G = E/2/(1 + 0.3);
K1 = 4 * diag([E*I1 E*I1 2*G*I1]);
K2 = 16 * diag([E*I2 E*I2 2*G*I2]);
K3 = 4 * diag([E*I3 E*I3 2*G*I3]);

r11=[1 0 0]'*rho1;r12=[0 1 0]'*rho1;r13=[-1 0 0]'*rho1;r14=[0 -1 0]'*rho1;
r21=[cos(21/180*pi) sin(21/180*pi) 0]'*rho2;
r22=[cos(37/180*pi) sin(37/180*pi) 0]'*rho2;
r23=[cos(53/180*pi) sin(53/180*pi) 0]'*rho2;
r24=[cos(69/180*pi) sin(69/180*pi) 0]'*rho2;
r25=[cos(111/180*pi) sin(111/180*pi) 0]'*rho2;
r26=[cos(127/180*pi) sin(127/180*pi) 0]'*rho2;
r27=[cos(143/180*pi) sin(143/180*pi) 0]'*rho2;
r28=[cos(159/180*pi) sin(159/180*pi) 0]'*rho2;
r29=[cos(201/180*pi) sin(201/180*pi) 0]'*rho2;
r2a=[cos(217/180*pi) sin(217/180*pi) 0]'*rho2;
r2b=[cos(233/180*pi) sin(233/180*pi) 0]'*rho2;
r2c=[cos(249/180*pi) sin(249/180*pi) 0]'*rho2;
r2d=[cos(291/180*pi) sin(291/180*pi) 0]'*rho2;
r2e=[cos(307/180*pi) sin(307/180*pi) 0]'*rho2;
r2f=[cos(323/180*pi) sin(323/180*pi) 0]'*rho2;
r2g=[cos(339/180*pi) sin(339/180*pi) 0]'*rho2;
r31=[1 0 0]'*rho3;r32=[0 1 0]'*rho3;r33=[-1 0 0]'*rho3;r34=[0 -1 0]'*rho3;
Q1=[S(r11)*e3 S(r12)*e3 S(r13)*e3 S(r14)*e3];
Q2=[S(r21)*e3 S(r22)*e3 S(r23)*e3 S(r24)*e3 ...
       S(r25)*e3 S(r26)*e3 S(r27)*e3 S(r28)*e3 ...
       S(r29)*e3 S(r2a)*e3 S(r2b)*e3 S(r2c)*e3 ...
       S(r2d)*e3 S(r2e)*e3 S(r2f)*e3 S(r2g)*e3];
Q3=[S(r31)*e3 S(r32)*e3 S(r33)*e3 S(r34)*e3];

F = [10 0 0
    -10 0 0
    0 10 0
    0 -10 0
    0 0 10
    0 0 -10]';

K_ratio = K2*L2/K1/L1; % du1 = kr * du2
%% u for sH
R = expm(S([45; 0; 45]*pi/180));
P = [60 -60 60]';
Tt= [R P;0 0 0 1];
config = invKineC3(Tt,Lr*1e3,L2*1e3,Lg*1e3);
psi = [config(1) config(4) config(3) config(5) config(7) config(9)];
xx = [L1 Lr L2 Lg 0.0001] * 1e3;
u = fromPsi2Curvature(psi, xx);
u1_init = u(1:3)';
u2_init = u(4:6)';
u3_init = u2_init*rho2/rho3;
p_init = P*1e-3;

%%
% moment_ = cross(F,p_init);

%%
% du2 = moment_(K1*K_ratio^2 +L1 +K2*L2);


%% ep2
du1 = zeros(3,6);
ep2 = zeros(2,6);
for i = 1: 6
    mo = cross(F(:,i),p_init);
    du1(:,i) = (4*K1 + 16*K2)\mo;
    ep2(1,i) = d1/(2*1e-3*norm(u1_init+du1(:,i))-rho1);
    ep2(2,i) = d1/(2*1e-3*max([norm(u1_init+du1(:,i)) norm(u2_init)])-rho2);
end

u = [u1_init;u2_init;u3_init];

%% ep1
G = zeros(24,9);
G(1:4,1:3) = 1/A1/E*pinv(Q1);
G(5:20,1:3) = 1/A2/E*pinv(Q2);
G(21:24,1:3) = 1/A3/E*pinv(Q3);
K = zeros(9,9);
K(1:3,1:3) = K1+K2;
K(1:3,4:6) = -K2;

K(4:6,4:6) = K2;
K(7:9,4:6) = rho2/rho3*K2;
K(7:9,7:9) = K2 + K3;
eps = G*K*u;
ep1 = max(eps);
ep = (ep1 +ep2)*100
