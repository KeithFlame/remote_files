SL = [100 10 20 15 0.1]';
psi = [deg2rad(20.715) 
        93.410
        deg2rad(30.845) 
        deg2rad(67.878) 
        deg2rad(46.676) 
        deg2rad(-82.297)];
[Tend, Scc] = FKcc_2segs_bending_keith(psi,SL);
PS_2segs_keith(Scc,SL,Tend);
xlabel('x');
ylabel('y');
p = Tend(1:3,4)';
r = rotm2eul(Tend(1:3,1:3),'ZYX');

V = [p r]

%% 100 000 psi
psi_max = [150 pi pi/2 pi pi/3*2 pi]';
psi_min = [30 -pi 0 -pi 0 -pi]';
step = 20;
filename = 'input.log';   % 文件名
filename2 = 'output.log';   % 文件名

% 以追加模式 ('a') 打开文件
fileID = fopen(filename, 'a');
fileID2 = fopen(filename2, 'a');
% 检查文件是否成功打开
if fileID == -1 || fileID2 == -1
    error('无法打开文件 %s', filename);
end
x = zeros(6,1);
for i = 1:10000
    X = rand(6,1).*(psi_max-psi_min)+psi_min;
    % t = X;t(2:end)=t(2:end)*180/pi;
    if(i>1)
        % if(norm(x-t)>20)
        %     t = x+20*(t-x)/norm(t-x);
        %     X=t;
        %     % X(2:end)=X(2:end)/180*pi;
        % end
    end
    fprintf(fileID, '%6f ', X); % 将数据以浮点数形式写入，用空格分隔
    fprintf(fileID, '\n');        % 换行
    % x = t;

    X(2:end)=X(2:end)/180*pi;mm = X(1);X(1) = X(2);X(2)=mm;
    [Tend,~]=FKcc_2segs_bending_keith(X, SL);
    XX = fromT2X(Tend);
    fprintf(fileID2, '%6f ', XX); % 将数据以浮点数形式写入，用空格分隔
    fprintf(fileID2, '\n');        % 换行
end
fclose(fileID);
fclose(fileID2);
%%



function deg = rad2deg(rad)
    deg = rad*180/pi;
end

function rad = deg2rad(deg)
    rad = deg*pi/180;
end