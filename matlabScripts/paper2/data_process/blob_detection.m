% 加载相机的内参和外参
load('stereoParams.mat'); % 替换为实际的内参和外参文件

% 左右图像文件夹路径
leftFolderPath = 'pic_factory/t3/temp/left'; % 替换为实际的左图像文件夹路径
rightFolderPath = 'pic_factory/t3/temp/right'; % 替换为实际的右图像文件夹路径

% 加载左右图像文件夹中的图像列表
leftImageFiles = dir(fullfile(leftFolderPath, '*.jpg')); % 替换为实际的图像文件类型
rightImageFiles = dir(fullfile(rightFolderPath, '*.jpg')); % 替换为实际的图像文件类型

% 创建输出文件夹
outputFolderPath_left = 'pic_factory/t3/temp/r_left'; % 替换为实际的输出文件夹路径
mkdir(outputFolderPath_left);
outputFolderPath_right = 'pic_factory/t3/temp/r_right'; % 替换为实际的输出文件夹路径
mkdir(outputFolderPath_right);
pixel_pos=zeros(numel(leftImageFiles),4);
for i = 1:numel(leftImageFiles)
    % 加载左图像
    leftImagePath = fullfile(leftFolderPath, leftImageFiles(i).name);
    leftImage = imread(leftImagePath);
    
    % 加载右图像
    rightImagePath = fullfile(rightFolderPath, rightImageFiles(i).name);
    rightImage = imread(rightImagePath);
    
    imshow(leftImage);
    pt=LoG_Blob(rgb2gray(leftImage));
    draw(leftImage,pt,'LOG')
    % % 将图像转换为灰度图像
    % grayImage_left = rgb2gray(leftImage);
    % 
    % % 对灰度图像进行阈值化
    % threshold = 100; % 替换为适当的阈值
    % binaryImage_left = grayImage_left > threshold;
    % 
    % % 对二值图像进行连通组件分析
    % cc_left = bwconncomp(binaryImage_left);
    % stats_left = regionprops(cc_left, 'Area', 'Centroid');
    % 
    % % 根据斑点的面积进行筛选
    % minArea = 100; % 最小斑点面积
    % maxArea = 1000000; % 最大斑点面积
    % idx = find([stats_left.Area] > minArea & [stats_left.Area] < maxArea);
    % 
    % % 在原始图像上标记检测到的斑点
    % markedImage = leftImage;
    % for j = 1:numel(idx)
    %     centroid = stats_left(idx(j)).Centroid;
    %     markedImage = insertMarker(markedImage, centroid, 'color', 'red');
    % end
end


function [points]=LoG_Blob(img,num_blobs)
%功能：提取LoG斑点
%img——输入图像
%num——需要检测斑点数目
%point——检测出的斑点
img=double(img(:,:,1));
if nargin==1     %如果输入参数仅有一个（img）
    num=120;    %则将检测斑点数设置为120
else
    num=num_blobs;
end
%设定LoG参数
sigma_begin=2;
sigma_end=15;
sigma_step=1;
sigma_array=sigma_begin:sigma_step:sigma_end;
sigma_nb=numel(sigma_array);
    %n = numel(A) returns the number of elements, n, in array A
    %equivalent to prod(size(A)).
img_height=size(img,1);
img_width=size(img,2);
%计算尺度规范化高斯拉普拉斯算子
snlo=zeros(img_height,img_width,sigma_nb);
for i=1:sigma_nb
    sigma=sigma_array(i);
    snlo(:,:,i)=sigma*sigma*imfilter(img,fspecial('log',...
        floor(6*sigma+1),sigma),'replicate');
end
%搜索局部极值
snlo_dil=imdilate(snlo,ones(3,3,3));
blob_candidate_index=find(snlo==snlo_dil);
blob_candidate_value=snlo(blob_candidate_index);
[temp,index]=sort(blob_candidate_value,'descend');
blob_index=blob_candidate_index(index(1:min(num,numel(index))));
[lig,col,sca]=ind2sub([img_height,img_width,sigma_nb],blob_index);
points=[lig,col,3*reshape(sigma_array(sca),[size(lig,1),1])];
end

function draw(img,pt,str)
%功能：在图像中绘制特征点
%img——输入图像
%pt——特征点坐标
%str——图上显示的名称
figure('Name',str);
imshow(img);
hold on;
axis off;
switch size(pt,2)
    case 2
        s=2;
        for i=1:size(pt,1)
            rectangle('Position',[pt(i,2)-s,pt(i,1)-s,2*s,2*s],'Curvature'...
                ,[0,0],'EdgeColor','b','LineWidth',2);
        end
    case 3
        for i=1:size(pt,1)
            rectangle('Position',[pt(i,2)-pt(i,3),pt(i,1)-pt(i,3),...
                2*pt(i,3),2*pt(i,3)],'Curvature',[1,1],'EdgeColor',...
                'w','LineWidth',2);
        end
end
end