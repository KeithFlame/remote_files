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
range = [pi*2 pi/3 pi*2 pi/3 pi*2 pi/3 pi*2 pi/3 pi*2]';
SL = [100 100 100 100];
filename = 'data3.txt';   % 文件名

% 以追加模式 ('a') 打开文件
fileID = fopen(filename, 'a');

% 检查文件是否成功打开
if fileID == -1
    error('无法打开文件 %s', filename);
end

%% execute
for i = 1:1000000
    psi = rand(9,1).*range; 

    % psi(2) = psi(2) + SL(1);
    % T = FKcc_2segs_nobending_keith(psi,SL); % continuum surgical manipulator.

    T = ForwardKinematicsRCCCC(psi,SL);     % RCCCC cotinuum manipulator.    
    if(T(1,4)<1 && T(1,4)>-1)
        X = fromT2X(T);
        
        % 将数据写入文件
        fprintf(fileID, '%6f ', X); % 将数据以浮点数形式写入，用空格分隔
        fprintf(fileID, '\n');        % 换行
    end
end

fclose(fileID);

%% 

data = load('data3.txt');

p= data(:,1:3);
figure;
grid on; hold on; axis equal;
plot3(p(:,1),p(:,2),p(:,3),'r.');
view([90 0])

%% auxillary fuction
