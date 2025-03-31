% another function to get the target for continuum robot.
% Detail: 
% this robot structural parameters are 100 10 20 15. (mm)
% by Monte-Calo method, using forwrad kinematics model get 1X10^5 poses.
%
% Ver. 1.0
% Date 2024.12.06
%
%

%% initial
range = [360 80 80 80 360 45 360]'*pi/180;
SL = [200 100 75 50];
filename = 'data6.txt';   % 文件名

% 以追加模式 ('a') 打开文件
fileID = fopen(filename, 'a');

% 检查文件是否成功打开
if fileID == -1
    error('无法打开文件 %s', filename);
end

%% execute
for i = 1:1000
    trand = rand(7,1);
    psi = trand.*range; 
    trand2 = trand>0.5;
    if(trand(2)>0.5)
        psi(2) = 80/180*pi;
    end
    if(trand(3)>0.5)
        psi(3) = 80/180*pi;
    end
    if(trand(4)>0.5)
        psi(4) = 80/180*pi;
    end
    if(trand(6)>0.5)
        psi(6) = 45/180*pi;
    end
    % psi = [pi pi/4 pi/4 pi/4 pi 0 0]';
    % psi(2) = psi(2) + SL(1);
    % T = FKcc_2segs_nobending_keith(psi,SL); % continuum surgical manipulator.

    T = ForwardKinematics7DoF(psi,SL);     % RCCCC cotinuum manipulator.    
    if(1)
        X = fromT2X(T);
        
        % 将数据写入文件
        fprintf(fileID, '%6f ', X); % 将数据以浮点数形式写入，用空格分隔
        fprintf(fileID, '\n');        % 换行
    end
end

fclose(fileID);

%% 

data = load('data5.txt');

p= data(:,1:3);
figure;
grid on; hold on; axis equal;
plot3(p(:,1),p(:,2),p(:,3),'r.');
view([90 0])

function T = ForwardKinematics7DoF(psi,SL)
La = 20;
L1 = SL(1) + La/2/cos(psi(3)/2);
L2 = SL(2) + La/2/cos(psi(4)/2) + La/2/cos(psi(3)/2);
L3 = SL(3) + La/2/cos(psi(6)/2) + La/2/cos(psi(4)/2);
L4 = SL(4) + La/2/cos(psi(6)/2);

P1 = [0 0 50]';
R1 = eul2rotm([psi(1) 0 0]);
R2 = R1 * eul2rotm([0 0 psi(2)]);
P2 = R2 * [0 0 L1]' + P1;

R3 = R2 * eul2rotm([0 0 psi(3)]);
P3 = P2 + R3 * [0 0 L2]';

R4 = R3 * eul2rotm([0 0 psi(4)]);
P4 = P3 + R4 * [0 0 L3]';

R5 = R4 * eul2rotm([psi(5) 0 0]);
P5 = P4;

R6 = R5 * eul2rotm([0 0 psi(6)]);
P6 = P5;

R7 = R6 * eul2rotm([psi(7) 0 0]);
P7 = P6 + R7 * [0 0 L4]';
T = [R7 P7; [0 0 0 1]];
end