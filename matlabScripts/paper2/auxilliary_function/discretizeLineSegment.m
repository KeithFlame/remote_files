function [points] = discretizeLineSegment(point1, point2, numPoints)
    % 输入参数：
    % point1 - 线段的起点 [x1, y1, z1]
    % point2 - 线段的终点 [x2, y2, z2]
    % numPoints - 生成的离散点的数量
    
    % 计算线段的方向向量
    direction = point2 - point1;
    
    % 生成均匀分布的离散点
    t = linspace(0, 1, numPoints); % 在 [0, 1] 之间生成 numPoints 个点
    points = point1 + t' * direction; % 使用广播计算点的位置
    
    % % 输出结果
    % disp('离散化的点:');
    % disp(points);
end