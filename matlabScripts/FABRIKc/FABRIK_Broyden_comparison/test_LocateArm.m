% FABRIK + Broyden 
% in Locate Arm


%% initialization
% SL = [100 10 20 15];
SL = [200 200 200 100 pi/4 200];
err_epsilon = 1e-3;
kmax = 100000;
initial_pose = eye(4);
% initial_pose(3,4)=160;
% limit = pi/3;
% r1 = 55.1329;
% r2 = r1;
% r3 = r2;
% r4 = r3;
% Pj1 = [0 0 r1]';
% Pj2 = [0 86.6 100]';
% Pj3 = [0 173.2 r1]';
% Pj4 = [0 173.2 -r1]';
% Pe = [0 143.24 -82.70]';
% initial_pose(4,:) = [r1 r2 r3 r4];           % 虚拟连杆的长度
% initial_pose(1:3,1:4) = [Pj1 Pj2 Pj3 Pe];      % 虚拟旋转中心位置


data = load('data4.txt');
block_size = size(data,1);

%% initialization
% SL = [100 10 20 15];
SL = [50 30 50 20];
err_epsilon = 1e-9;
kmax = 1000000;
initial_pose = eye(4);
initial_pose(3,4)=160;
limit = [pi/2 pi*2/3];
R1 = [1 0 0; 0 cos(limit(1)) sin(limit(1)); 0 -sin(limit(1)) cos(limit(1))];
r1 = SL(1)/limit(1);r2 = SL(3)/limit(2);
Pr = [0 r1*(1-cos(limit(1)))+SL(2)*sin(limit(1)) r1*sin(limit(1))+SL(2)*cos(limit(1))]';
P2 = Pr + R1 * [0 r2*(1-cos(limit(2))) r2*sin(limit(2))]';
R2 = R1 * [1 0 0; 0 cos(limit(2)) sin(limit(2)); 0 -sin(limit(2)) cos(limit(2))];
% Pe = R2 * [0 SL(2)*sin(limit(1)) SL(2)*cos(limit(1))]';
initial_pose(4,1) = r1 * tan(limit(1)/2);           % 第一虚拟连杆的长度
initial_pose(1:3,1) = [0 0 initial_pose(4,1)]';      % 第一虚拟旋转中心位置
initial_pose(4,2) = r2 * tan(limit(2)/2);           % 第二虚拟连杆长度
initial_pose(1:3,2) = Pr + R1 * ...                 % 第二虚拟旋转中心位置
    [0 initial_pose(4,2)*sin(limit(1)) initial_pose(4,2)*cos(limit(1))]';
initial_pose(1:3,3) = R2(:,3);                      % 末端指向姿态
initial_pose(1:3,4) = P2;                           % 末端位置

data = load('data2.txt');
block_size = size(data,1);
k_nB_block = zeros(block_size,1);
time_nB_block = zeros(block_size,1);
flag_nB_block = zeros(block_size,1);
error_nB_PR_block = zeros(block_size,2);
unsolved_nB_block = zeros(block_size,6);
theta_nB_block = zeros(block_size,2);
k_B_block = zeros(block_size,1);
time_B_block = zeros(block_size,1);
flag_B_block = zeros(block_size,1);
error_B_PR_block = zeros(block_size,2);
unsolved_B_block = zeros(block_size,6);
theta_B_block = zeros(block_size,2);
jm = 1;
k = 1;

times = zeros(5,1);
ks = zeros(5,1);
errors = zeros(5,2);
thetas = zeros(5,2);
%% execution
for i = 314:block_size
    target = fromX2T(data(i,:)');
    [flag_nB, time_nB,k_nB,error_PR_nB,THETA_nB]=constraints_FABRIK_without_Broyden(target,SL,initial_pose,err_epsilon,kmax,limit);
    k_nB_block(i) = k_nB;
    time_nB_block(i) = time_nB;
    flag_nB_block(i) = flag_nB;
    error_nB_PR_block(i,:) = error_PR_nB;
    theta_nB_block(i,:) = THETA_nB; 
    if(flag_nB==-2)
        unsolved_nB_block(jm,:) = data(i,:);
        jm = jm + 1;
    end
    for j = 1:2
        [flag_B, time_B,k_B,error_PR_B,THETA_B]=constraints_FABRIK_with_Broyden(target,SL,initial_pose,err_epsilon,kmax,limit);
        times(j)=time_B;
        ks(j) = k_B;
        errors(j,:) = error_PR_B;
        thetas(j,:) = THETA_B;
        [min_v, min_p]=min(times);
        min_p = 1;
    end
    k_B_block(i) = ks(min_p);
    time_B_block(i) = times(min_p);
    flag_B_block(i) = flag_B;
    error_B_PR_block(i,:) = errors(min_p);
    theta_B_block(i,:) = thetas(min_p);
    if(flag_B==-2)
        unsolved_B_block(k,:) = data(i,:);
        k = k + 1;
    end
end
%% data process
figure(2);
semilogy(1:100,100:-1:1)
cla;
x =1:block_size;
semilogy(x,k_nB_block,'ro-');

hold on;
semilogy(x,k_B_block,'b--');
% set(gca, 'YDir', 'reverse');           % 反转 Y 轴方向

% ylim(1.1*[-max(k_B_block), max(k_nB_block)]); % 设置 Y 轴范围为负的最大值到正的最大值

% axis([0 100000 -30 10000])
xlabel("Serial of Target")
ylabel("Iteration Number")
legend("Conventional FABRIK","FABRIK+Broyden");
set(gca,"FontName","Times New Roman");
set(gca,"FontSize",20);

figure(3);
semilogy(1:100,100:-1:1)
cla;
x =1:block_size;
semilogy(x,1000*time_nB_block,'ro-');

hold on;
semilogy(x,1000*time_B_block,'b--');
% set(gca, 'YDir', 'reverse');           % 反转 Y 轴方向

% ylim(1.1*[-max(k_B_block), max(k_nB_block)]); % 设置 Y 轴范围为负的最大值到正的最大值

% axis([0 100000 -30 10000])
xlabel("Serial of Target")
ylabel("Time Cost (ms)")
legend("Conventional FABRIK","FABRIK+Broyden");
set(gca,"FontName","Times New Roman");
set(gca,"FontSize",20);
