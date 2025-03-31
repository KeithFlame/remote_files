% % 加载双目相机参数
% load('stereoCameraParams.mat'); % 包含内外参数的文件

% 读取左图和右图
leftImage = imread('F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_6_1\origin\left/M0028.jpg');  % 左图
rightImage = imread('F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_6_1\origin\right/M0028.jpg'); % 右图

% 定义感兴趣区域（ROI），例如 [x, y, width, height]
roi = [800, 500, 400, 500]; % 根据需要调整

% 提取 ROI
leftROI = imcrop(leftImage, roi);
rightROI = imcrop(rightImage, roi);

% 转换为灰度图像
grayLeft = rgb2gray(leftROI);
grayRight = rgb2gray(rightROI);

% 边缘检测
edgesLeft = edge(grayLeft, 'Canny');
edgesRight = edge(grayRight, 'Canny');

% 检测圆
[centersLeft, radiiLeft] = imfindcircles(grayLeft, [10 50], 'ObjectPolarity', 'bright', 'Sensitivity', 0.92);
[centersRight, radiiRight] = imfindcircles(grayRight, [10 50], 'ObjectPolarity', 'bright', 'Sensitivity', 0.92);

% 检测椭圆
statsLeft = regionprops('table', edgesLeft, 'MajorAxisLength', 'MinorAxisLength', 'Orientation', 'Centroid');
statsRight = regionprops('table', edgesRight, 'MajorAxisLength', 'MinorAxisLength', 'Orientation', 'Centroid');

% 绘制结果
figure;
subplot(1, 2, 1);
imshow(leftROI);
hold on;
viscircles(centersLeft, radiiLeft, 'EdgeColor', 'b'); % 绘制检测到的圆
% 绘制椭圆
for k = 1:height(statsLeft)
    theta = linspace(0, 2*pi, 100);
    a = statsLeft.MajorAxisLength(k) / 2; % 半长轴
    b = statsLeft.MinorAxisLength(k) / 2; % 半短轴
    x0 = statsLeft.Centroid(k, 1); % 中心x
    y0 = statsLeft.Centroid(k, 2); % 中心y
    x = x0 + a * cos(theta) * cosd(statsLeft.Orientation(k)) - b * sin(theta) * sind(statsLeft.Orientation(k));
    y = y0 + a * cos(theta) * sind(statsLeft.Orientation(k)) + b * sin(theta) * cosd(statsLeft.Orientation(k));
    
    plot(x, y, 'r', 'LineWidth', 2); % 绘制椭圆
end
title('Left Image Detected Circles and Ellipses');
hold off;

subplot(1, 2, 2);
imshow(rightROI);
hold on;
viscircles(centersRight, radiiRight, 'EdgeColor', 'b'); % 绘制检测到的圆
% 绘制椭圆
for k = 1:height(statsRight)
    theta = linspace(0, 2*pi, 100);
    a = statsRight.MajorAxisLength(k) / 2; % 半长轴
    b = statsRight.MinorAxisLength(k) / 2; % 半短轴
    x0 = statsRight.Centroid(k, 1); % 中心x
    y0 = statsRight.Centroid(k, 2); % 中心y
    x = x0 + a * cos(theta) * cosd(statsRight.Orientation(k)) - b * sin(theta) * sind(statsRight.Orientation(k));
    y = y0 + a * cos(theta) * sind(statsRight.Orientation(k)) + b * sin(theta) * cosd(statsRight.Orientation(k));
    
    plot(x, y, 'r', 'LineWidth', 2); % 绘制椭圆
end
title('Right Image Detected Circles and Ellipses');
hold off;

disp('双目圆斑检测完成！');