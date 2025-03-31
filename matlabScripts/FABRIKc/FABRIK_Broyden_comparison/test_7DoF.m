% FABRIK + Broyden 
% in Locate Arm


%% initialization
% SL = [100 10 20 15];
SL = [200 100 75 50];
err_epsilon = 1e-3;
kmax = 1000;
initial_pose = eye(4);
data = load('data6.txt');
block_size = size(data,1);
limits = [80 80 80 45]*pi/180;

k_nB_block = zeros(block_size,1);
time_nB_block = zeros(block_size,1);
flag_nB_block = zeros(block_size,1);
error_nB_PR_block = zeros(block_size,2);
unsolved_nB_block = zeros(block_size,6);
theta_nB_block = zeros(block_size,4);
jm1 = 0;
jm2 = 0;

k_MMT_block = zeros(block_size,1);
time_MMT_block = zeros(block_size,1);
flag_MMT_block = zeros(block_size,1);
error_MMT_PR_block = zeros(block_size,2);
unsolved_MMT_block = zeros(block_size,6);
theta_MMT_block = zeros(block_size,4);
jm1_MMT = 0;
jm2_MMT = 0;

k_B_block = zeros(block_size,1);
time_B_block = zeros(block_size,1);
flag_B_block = zeros(block_size,1);
error_B_PR_block = zeros(block_size,2);
unsolved_B_block = zeros(block_size,6);
theta_B_block = zeros(block_size,4);
jm1_B = 0;
jm2_B = 0;

times = zeros(5,1);
ks = zeros(5,1);
errors = zeros(5,2);
thetas = zeros(5,4);
%% execution
for i = 205:block_size
    target = fromX2T(data(i,:)');
    [flag_nB, time_nB,k_nB,error_PR_nB,THETA_nB]=cFABRIK_7DoF_without_Broyden(target,SL,initial_pose,err_epsilon,kmax,limits);
    k_nB_block(i) = k_nB;
    time_nB_block(i) = time_nB;
    flag_nB_block(i) = flag_nB;
    error_nB_PR_block(i,:) = error_PR_nB;
    theta_nB_block(i,:) = THETA_nB; 
    if(flag_nB==-2)
        jm2 = jm2 + 1;
        unsolved_nB_block(jm2,:) = data(i,:);
    end
    if(flag_nB==-1)
        jm1 = jm1 + 1;
    end

    [flag_MMT, time_MMT,k_MMT,error_PR_MMT,THETA_MMT]=cFABRIK_7DoF_MMT(target,SL,initial_pose,err_epsilon,kmax,limits);
    k_MMT_block(i) = k_MMT;
    time_MMT_block(i) = time_MMT;
    flag_MMT_block(i) = flag_MMT;
    error_MMT_PR_block(i,:) = error_PR_MMT;
    theta_MMT_block(i,:) = THETA_MMT; 
    if(flag_MMT==-2)
        jm2_MMT = jm2_MMT + 1;
        unsolved_MMT_block(jm2_MMT,:) = data(i,:);
    end
    if(flag_MMT==-1)
        jm1_MMT = jm1_MMT + 1;
    end

    for j = 1:5
        [flag_B, time_B,k_B,error_PR_B,THETA_B]=cFABRIK_7DoF_with_Broyden(target,SL,initial_pose,err_epsilon,kmax,limits);
        % [flag_B, time_B,k_B,error_PR_B,THETA_B]=cFABRIK_7DoF_with_Broyden_Plus(target,SL,initial_pose,err_epsilon,kmax,limits);
        times(j)=time_B;
        ks(j) = k_B;
        errors(j,:) = error_PR_B;
        thetas(j,:) = THETA_B;
        [min_v, min_p]=min(times);
        % min_p = 1;
    end
    k_B_block(i) = ks(min_p);
    time_B_block(i) = times(min_p);
    flag_B_block(i) = flag_B;
    error_B_PR_block(i,:) = errors(min_p);
    theta_B_block(i,:) = thetas(min_p);
    if(flag_B==-2)
        jm2_B = jm2_B + 1;
        unsolved_nB_block(jm2_B,:) = data(i,:);
    end
    if(flag_B==-1)
        jm1_B = jm1_B + 1;
    end
end
%% data process
figure(2);
semilogy(1:100,100:-1:1)
cla;
x =1:block_size;
semilogy(x,k_nB_block,'ro-');

hold on;
semilogy(x,k_MMT_block,'g--');
semilogy(x,k_B_block,'b--');


% set(gca, 'YDir', 'reverse');           % 反转 Y 轴方向

% ylim(1.1*[-max(k_B_block), max(k_nB_block)]); % 设置 Y 轴范围为负的最大值到正的最大值

% axis([0 100000 -30 10000])
xlabel("Serial of Target")
ylabel("Iteration Number")
legend("Conventional FABRIK","MMT FABRIK","FABRIK+Broyden");
set(gca,"FontName","Times New Roman");
set(gca,"FontSize",20);

figure(3);
semilogy(1:100,100:-1:1)
cla;
x =1:block_size;
semilogy(x,1000*time_nB_block,'ro-');

hold on;
semilogy(x,1000*time_MMT_block,'g--');
semilogy(x,1000*time_B_block,'b--');
% set(gca, 'YDir', 'reverse');           % 反转 Y 轴方向

% ylim(1.1*[-max(k_B_block), max(k_nB_block)]); % 设置 Y 轴范围为负的最大值到正的最大值

% axis([0 100000 -30 10000])
xlabel("Serial of Target")
ylabel("Time Cost (ms)")
legend("Conventional FABRIK","MMT FABRIK","FABRIK+Broyden");
set(gca,"FontName","Times New Roman");
set(gca,"FontSize",20);


%% data

mean_time = [mean(time_nB_block) mean(time_MMT_block) mean(time_B_block)];
mean_K = [mean(k_nB_block) mean(k_MMT_block) mean(k_B_block)];