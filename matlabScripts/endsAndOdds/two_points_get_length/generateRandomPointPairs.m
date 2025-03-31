function pointPairs = generateRandomPointPairs(distance,numPairs)
    % generateRandomPointPairs 生成100对随机的空间点，距离相同
    % 输入：
    % distance - 每对空间点之间的固定距离
    %
    % 输出：
    % pointPairs - 100对空间点 (6x100)，每列为一个点对 [x1, y1, z1, x2, y2, z2]
    
    % numPairs = 100; % 生成的点对数量
    pointPairs = zeros(6, numPairs); % 初始化点对矩阵

    % 随机生成点对
    for i = 1:numPairs
        % 随机生成一个空间点 A
        A = rand(3, 1) * 10; % 生成范围在 [0, 10] 内的随机点
        
        % 随机生成一个方向向量，并归一化
        direction = rand(3, 1);
        direction = direction / norm(direction); % 归一化方向
        
        % 计算点 B，使得 A 和 B 之间的距离为指定值
        B = A + direction * distance; % 生成第二个点

        % 将点对存储到矩阵中
        pointPairs(:, i) = [A; B]; % [x1, y1, z1, x2, y2, z2]
    end
end