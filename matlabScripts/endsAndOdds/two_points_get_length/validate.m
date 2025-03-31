%%
result:

%% 空间点对
% 设置每对空间点之间的距离
fixedDistance = 65; % 例如，距离为 5

% 生成的点对数量
numPairs = 100;

% 生成点对
pointPairs = generateRandomPointPairs(fixedDistance,numPairs);

% % 输出结果
% disp('生成的点对：');
% disp(pointPairs);

%% 图像坐标
% 定义相机内参矩阵
intrinsic = [1000, 0, 960; 0, 1000, 540; 0, 0, 1]; % 示例内参

p1_pxl = zeros(numPairs,2);
p2_pxl = zeros(numPairs,2);

% 调用函数
for i = 1:numPairs
    [p1_cam, p2_cam] = worldToCamera(intrinsic, pointPairs(1:3,i), pointPairs(4:6,i));
    p1_pxl(i,:)=p1_cam(1:2)';
    p2_pxl(i,:)=p2_cam(1:2)';
end


minDistance = optimizeSpatialDistance(intrinsic, [p1_pxl p2_pxl]');


function minDistance = optimizeSpatialDistance(intrinsic, pixelPairs)
    % optimizeSpatialDistance 优化成对像素坐标对应的空间点之间的距离
    % 输入：
    % intrinsic - 相机内参矩阵 (3x3)
    % pixelPairs - 100对像素坐标 (2x100)，每列为一个点对 [x1, y1; x2, y2]
    %
    % 输出：
    % minDistance - 优化后的最小空间距离

    % 提取像素对
    numPairs = size(pixelPairs, 2);
    A_pixel = pixelPairs(1:2, :); % 像素点 A
    B_pixel = pixelPairs(3:4, :); % 像素点 B

    % 初始猜测
    initialDistance = 65; % 可以选择一个合理的初始值

    % 目标函数：最小化空间距离
    objective = @(d) calculateObjective(d, A_pixel, B_pixel, intrinsic);

    % 约束条件：d >= 0
    lb = 0; % 下界
    ub = []; % 没有上界

    % 运行 fmincon
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    [minDistance, fval] = fmincon(objective, initialDistance, [], [], [], [], lb, ub, [], options);

    % 输出优化结果
    disp('优化后的最小空间距离:');
    disp(minDistance);
end

function objValue = calculateObjective(d, A_pixel, B_pixel, intrinsic)
    % calculateObjective 计算目标函数值
    % 输入：
    % d - 当前优化的距离
    % A_pixel - 像素点 A
    % B_pixel - 像素点 B
    % intrinsic - 相机内参矩阵
    %
    % 输出：
    % objValue - 目标函数值

    % 将像素坐标转换为归一化相机坐标
    A_normalized = inv(intrinsic) * [A_pixel; ones(1, size(A_pixel, 2))];
    B_normalized = inv(intrinsic) * [B_pixel; ones(1, size(B_pixel, 2))];

    % 计算空间距离
    distances = sqrt(sum((A_normalized(1:3, :) - B_normalized(1:3, :)).^2, 1));

    % 计算目标函数值：距离与目标距离的差的平方和
    objValue = sum((distances - d).^2);
end