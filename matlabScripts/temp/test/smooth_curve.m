%%
data = load('output.log');
incremental = 2;
block_size = size(data,1)+1;
T0 = [0 0 100 0 0 0];
data = [T0;data];
X = zeros(6,1);
%%
T1 = fromX2T(data(1,:)');
T2 = fromX2T(data(2,:)');
% p = getMiddleT(T1,T2,incremental);
% p = getMiddleT(p,T2,incremental);
p = T1;
i = 1;
filename = 'input_inv.log';   % 文件名
fileID = fopen(filename, 'a');
if fileID == -1 
    error('无法打开文件 %s', filename);
end
while (1)
    if(sum(sum(abs(p-T2)))<1e-6)
        i = i+1;
    end
    if(i == 1000)
        break;
    end
    T1 = fromX2T(data(i,:)');
    T2 = fromX2T(data(i+1,:)');
    p = getMiddleT(p,T2,incremental);
    X = fromT2X(p);
    fprintf(fileID, '%6f ', X); % 将数据以浮点数形式写入，用空格分隔
    fprintf(fileID, '\n');        % 换行
end
fclose(fileID);