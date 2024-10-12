% 指定文件夹路径
folderPath = 'F:\code_git\matlabScripts\paper2\data_process\pic_factory\t2f\main_pic\';

% 获取文件夹下的所有图片文件
imageFiles = dir(fullfile(folderPath, '*.jpg')); % 修改为您需要的图片格式

% 新的文件名前缀
% newPrefix = 'M';
serial=287;
% serial = serial+numel(imageFiles);
% 遍历文件列表，修改文件名
for i = 1:142
    newPrefix = setNumber(serial+i-1);
    oldName = fullfile(folderPath, imageFiles(i+serial).name);
    [~, oldFileName, oldExt] = fileparts(oldName);
    newFileName = [newPrefix, oldExt];
    newName = fullfile(folderPath, newFileName);
    movefile(oldName, newName);
end

function name = setNumber(num)

    if(num<10)
        name = ['M00',num2str(num)];
    elseif(num<100)
        name = ['M0',num2str(num)];
    else
        name = ['M',num2str(num)];
    end
end