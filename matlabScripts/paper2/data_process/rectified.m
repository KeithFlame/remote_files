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
pro_err=zeros(numel(leftImageFiles),1);
for i = 1:numel(leftImageFiles)
    % 加载左图像
    leftImagePath = fullfile(leftFolderPath, leftImageFiles(i).name);
    leftImage = imread(leftImagePath);
    
    % 加载右图像
    rightImagePath = fullfile(rightFolderPath, rightImageFiles(i).name);
    rightImage = imread(rightImagePath);
    
    % 对左图像进行校正
    [J1, J2,pro_erri] = rectifyStereoImages(leftImage, rightImage, stereoParams, 'OutputView', 'valid');
    % pro_err(i)=pro_erri;
    
    % 保存校正后的图像
    if(i<10)
        ns=['M00',num2str(i-1),'.jpg'];
    elseif(i<100)
        ns=['M0',num2str(i-1),'.jpg'];
    else
        ns=['M',num2str(i-1),'.jpg'];
    end
    rectifiedLeftImagePath = fullfile(outputFolderPath_left, ns);
    rectifiedRightImagePath = fullfile(outputFolderPath_right, ns);
    imwrite(J1, rectifiedLeftImagePath);
    imwrite(J2, rectifiedRightImagePath);
end