% a function to get the target for continuum robot.
% Detail: 
% this robot structural parameters are 100 10 20 15. (mm)
% by Monte-Calo method, using forwrad kinematics model get 1X10^5 poses.
%
% Ver. 1.0
% Date 2024.12.06
%
%

%% initial
range = [pi/3 pi/3 pi/3 300 pi/3 pi/3]';
SL = [200 200 400 100 pi/4 200];
filename = 'data4.txt';   % 文件名

% 以追加模式 ('a') 打开文件
fileID = fopen(filename, 'a');

% 检查文件是否成功打开
if fileID == -1
    error('无法打开文件 %s', filename);
end

%% execute
for i = 1:100000
    psi = 2*(rand(6,1)-0.5).*range; 
    psi = [0 0 0 0 0 0];
    % psi(2) = psi(2) + SL(1);
    % T = FKcc_2segs_nobending_keith(psi,SL); % continuum surgical manipulator.

    T = ForwardKinematicsLocateArm(psi,SL);     % RCCCC cotinuum manipulator.    
    if(1)
        X = fromT2X(T);
        
        % 将数据写入文件
        fprintf(fileID, '%6f ', X); % 将数据以浮点数形式写入，用空格分隔
        fprintf(fileID, '\n');        % 换行
    end
end

fclose(fileID);

%% 

data = load('data4.txt');

p= data(:,1:3);
figure;
grid on; hold on; axis equal;
plot3(p(:,1),p(:,2),p(:,3),'r.');
view([90 0])

%% auxillary fuction
function T = ForwardKinematicsLocateArm(psi,SL)
P1 = [0 0 0]';
R1 = eul2rotm([psi(1) 0 0]);
P2 = R1 * [SL(1) 0 0]' + P1;
R2 = R1 * eul2rotm([psi(2) 0 0]);
P3 = P2 + R2 * [SL(2) 0 0]';
R3 = R2 * eul2rotm([psi(3) 0 0]);
P4 = P3 - [0 0 SL(3)+psi(4)]';
R4 = R3;
P5 = P4 - R4 * eul2rotm([0 SL(5) 0]) * [0 0 SL(4)]';
R5 = R4 * eul2rotm([0 SL(5) 0]) * eul2rotm([psi(5) 0 0]);
P6 = P5 - R5 * [0 0 SL(6)]';
R6 = R5 * eul2rotm([0 psi(6) 0]);
T = [R6 P6; [0 0 0 1]];
end
