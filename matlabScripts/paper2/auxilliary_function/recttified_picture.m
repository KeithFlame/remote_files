% 加载内外参
load('cameraParameters.mat'); % 假设你已经有内外参存储在这个文件中

% 设置左图和右图的文件夹路径
leftFolder = 'G:\pic\trial\date0114\tem2p\left\';
rightFolder = 'G:\pic\trial\date0114\tem2p\right\';
outputLeftFolder = 'F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_6_1\origin\left\';
outputRightFolder = 'F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_6_1\origin\right\';

% 创建输出文件夹（如果不存在）
if ~exist(outputLeftFolder, 'dir')
    mkdir(outputLeftFolder);
end
if ~exist(outputRightFolder, 'dir')
    mkdir(outputRightFolder);
end

% 获取文件夹中的所有图像文件
leftImages = dir(fullfile(leftFolder, '*.jpg')); % 根据实际格式修改
rightImages = dir(fullfile(rightFolder, '*.jpg')); % 根据实际格式修改

% 校正图像
for i = 1:length(leftImages)
    % 读取左图像
    leftImage = imread(fullfile(leftFolder, leftImages(i).name));
    % 读取右图像
    rightImage = imread(fullfile(rightFolder, rightImages(i).name));
    
    % 校正图像
    [correctedLeft, correctedRight] = rectifyStereoImages(leftImage, rightImage, cameraParameters);

    % 保存校正后的图像
    imwrite(correctedLeft, fullfile(outputLeftFolder, leftImages(i).name));
    imwrite(correctedRight, fullfile(outputRightFolder, rightImages(i).name));
end

disp('双目图像校正完成！');