I1 = imread('f1.jpg');

img = I1;%(150:980,500:1500,:);
% imshow(img);
% title('Original Image');

% 将图像转换为灰度图像
grayImg = rgb2gray(img);

% 使用自适应阈值方法进行二值化
binaryImg = imbinarize(grayImg, 'adaptive', 'Sensitivity', 0.6);

% 定义结构元素
se1 = strel('disk', 2); % 创建一个半径为2的圆形结构元素
se2 = strel('disk', 7); % 创建一个半径为7的圆形结构元素
se3 = strel('disk', 35); % 创建一个半径为35的圆形结构元素

% 使用开操作平滑图像
smoothedImg = imclose(binaryImg, se1);
smoothedImg = imopen(smoothedImg, se2);
smoothedImg = imclose(smoothedImg, se2);
smoothedImg = imopen(smoothedImg, se2);
smoothedImg = imclose(smoothedImg, se3);

% 交换黑白颜色
invertedBinaryImg = ~smoothedImg;

% 变换尺寸
% invertedBinaryImg = imresize(invertedBinaryImg,0.5);

% 使用Canny边缘检测算法
% edgeImg = edge(invertedBinaryImg, 'Prewitt');

imshow(invertedBinaryImg);
title('invertedBinaryImg Image');


% 使用Hough变换检测圆
[centers, radii, metric] = imfindcircles(invertedBinaryImg, [150 200],"EdgeThreshold",0.1, 'Sensitivity', 0.9); % 设置半径范围 [minRadius, maxRadius]

% 绘制检测到的圆
figure;
imshow(invertedBinaryImg);
title('Detected Circles');
viscircles(centers, radii,'EdgeColor','b'); % 绘制圆，边缘颜色为蓝色

% 显示检测到的圆的数量
numCircles = size(centers, 1);
fprintf('Number of circles detected: %d\n', numCircles);

%% 连通域
[B,L] = bwboundaries(invertedBinaryImg,'noholes');%只关注外边界
figure(6),imshow(label2rgb(L,@jet,[.5 .5 .5]))%标记矩阵L转化为RGB图像
hold on
for k = 1:length(B)
  boundary = B{k};
  plot(boundary(:,2),boundary(:,1),'w','LineWidth',2);%绘制白色边界线
end
stats = regionprops(L,'Area','Centroid');%测量图像区域的属性(实际像素数和质心)
threshold = 0.9;%阈值
% 边界循环
for k = 1:length(B)
	boundary = B{k};%获取标签“k”对应的（X，Y）边界坐标
    %计算物体的近似周长
	delta_sq = diff(boundary).^2;
	perimeter = sum(sqrt(sum(delta_sq,2)));
    
	area = stats(k).Area;%计算标签‘k’对应的面积
	metric = 4*pi*area/perimeter^2;%计算圆度
	metric_string = sprintf('%2.2f',metric);%显示结果

    %如果圆度大于阈值threshold，则用黑色圆圈绘制圆心(代表为圆)
	if metric > threshold
        centroid = stats(k).Centroid;
        plot(centroid(1),centroid(2),'ko');

    end
  
    text(boundary(1,2)-35,boundary(1,1)+13,metric_string,'Color','y',...
       'FontSize',14,'FontWeight','bold')   %在边界左侧显示圆度
end
title('指标越接近1代表越接近圆形');