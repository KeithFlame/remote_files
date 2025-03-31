% FABRIK + Broyden 
% in continuum robot


%% initialization
% SL = [100 10 20 15];
SL = [100 100 100 100];
err_epsilon = 1e-3;
kmax = 100000;
initial_pose = eye(4);
initial_pose(3,4)=160;
limit = pi/3;
r1 = 55.1329;
r2 = r1;
r3 = r2;
r4 = r3;
Pj1 = [0 0 r1]';
Pj2 = [0 86.6 100]';
Pj3 = [0 173.2 r1]';
Pj4 = [0 173.2 -r1]';
Pe = [0 143.24 -82.70]';
initial_pose(4,:) = [r1 r2 r3 r4];           % 虚拟连杆的长度
initial_pose(1:3,1:4) = [Pj1 Pj2 Pj3 Pe];      % 虚拟旋转中心位置


data = load('data3.txt');
block_size = size(data,1);
k_nB_block = zeros(block_size,1);
time_nB_block = zeros(block_size,1);
flag_nB_block = zeros(block_size,1);
error_nB_PR_block = zeros(block_size,2);
unsolved_nB_block = zeros(block_size,6);
theta_nB_block = zeros(block_size,4);
k_B_block = zeros(block_size,1);
time_B_block = zeros(block_size,1);
flag_B_block = zeros(block_size,1);
error_B_PR_block = zeros(block_size,2);
unsolved_B_block = zeros(block_size,6);
theta_B_block = zeros(block_size,4);
jm = 0;
k = 0;

times = zeros(5,1);
ks = zeros(5,1);
errors = zeros(5,2);
thetas = zeros(5,4);
%% execution
for i = 296:1000%block_size                 %546
    target = fromX2T(data(i,:)');
    [flag_nB, time_nB,k_nB,error_PR_nB,THETA_nB]=constraints_FABRIK_without_Broyden_RCCCC(target,SL,initial_pose,err_epsilon,kmax,limit);
    k_nB_block(i) = k_nB;
    time_nB_block(i) = time_nB;
    flag_nB_block(i) = flag_nB;
    error_nB_PR_block(i,:) = error_PR_nB;
    theta_nB_block(i,:) = THETA_nB; 
    if(flag_nB==-2)
        jm = jm + 1;
        unsolved_nB_block(jm,:) = data(i,:);
    end
    for j = 1:5
        [flag_B, time_B,k_B,error_PR_B,THETA_B]=cFABRIK_with_Broyden_RCCCC(target,SL,initial_pose,err_epsilon,kmax,limit);
        times(j)=time_B;
        ks(j) = k_B;
        errors(j,:) = error_PR_B;
        thetas(j,:) = THETA_B;
        [min_v, min_p]=min(times);
    end
    k_B_block(i) = ks(min_p);
    time_B_block(i) = times(min_p);
    flag_B_block(i) = flag_B;
    error_B_PR_block(i,:) = errors(min_p);
    theta_B_block(i,:) = thetas(min_p);
    if(flag_B==-2)
        k = k + 1;
        unsolved_B_block(k,:) = data(i,:);
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