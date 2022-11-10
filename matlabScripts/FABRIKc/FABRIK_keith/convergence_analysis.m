%
% 纯粹代数法

%% 临时验证
lamda_a = 50:1000;
vP1 = [0 0 0]';ve1 = [1 0 0]';
vP2 = [0 30 0]';
block_size = size(lamda_a,2);
L = zeros(block_size,1);
for i = 1:block_size
    vP3 = vP1 + lamda_a(i)*ve1;
    ve2 = (vP3-vP2)/norm(vP2-vP3);
    ang = pi-acos(ve2'*ve1);
    L(i) = lamda_a(i)*ang/tan(ang/2);
end
figure;axis equal;
plot(lamda_a,L);
%% P0-P6;
% 证明|P6-P1|<|P4-P3|
% 三个变量 theta, x4,z4; lamda

theta = 30 * pi / 180;
x4 = 100;
z4 = 200;

L = 100;

m = 30;
n = 40;

l0 = L/theta*tan(theta/2);
k = norm([n,m]);

P0 = [0 0 0]';
P1 = [m,n,0]';
e1 = P1/norm(P1);
e2 = [cos(theta)*e1(1)-sin(theta)*e1(2) sin(theta)*e1(1)+cos(theta)*e1(2) 0]';

P2 = (k+l0)/k*P1;
P3 = (k+l0+l0*cos(theta))/k*P1 + l0*sin(theta)*e2;
P4= [x4,P2(2),z4]';

e3 = (P4 - P3)/norm(P4 - P3);
e4 = (P4 - P2)/norm(P4 - P2);

P5 = e4*lamda +P4;
e5 = (P0 - P5)/norm(P0 - P0);
P6 = e5*lamda + P5;
theta2 = acos(e5'*e4);
lamda = L/theta2*tan(theta2/2);

% function L = getLamda_a(P1,e1,P2,L)



%% test
l11 = 50;
theta_const = pi/5;
N = 3;
sum_1 = 0;
for i = 1:N-1
    sum_1 = sum_1+ 2*l11*cos(theta_const-i*theta_const/N);
end
x1 = sum_1 + l11*cos(theta_const) +l11;
x2 = l11/tan(theta_const/2/N)*tan(theta_const/2)*(cos(theta_const) + 1);
x1-x2
%% 
theta_const = pi/4;
N = 1;
sum_1 = 0;
for i = 0:N
    sum_1 = sum_1+ cos(theta_const-i*theta_const/N);
end
x1 = 2*sum_1;
x2 = (1+tan(theta_const/2)/tan(theta_const/2/N))*(cos(theta_const) + 1);
x3 = 1+cos(theta_const)+sin(theta_const)/tan(theta_const/2/N);
% x1-x2
x2-x3
%%
theta_const = pi/2*1.232;
N = 300;
sum_1 = 0;
for i = 1:N
    sum_1 = sum_1+ cos(i*theta_const/N);
end
x1 = sum_1;
x2 = (sin(theta_const*(2*N+1)/2/N))/2/sin(theta_const/2/N)-0.5;
x1-x2

%%
%%
theta_const = pi/4*1.232;
N = 300;
sum_1 = 0;
for i = 1:N
    sum_1 = sum_1+ sin(i*theta_const/N);
end
x1 = sum_1;
x2 = (1-cos(theta_const) +sin(theta_const)*tan(theta_const/2/N))/2/tan(theta_const/2/N);
x1-x2