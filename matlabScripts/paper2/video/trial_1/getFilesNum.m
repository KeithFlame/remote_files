function num = getFilesNum(file_name,file_type)
% a function to acquire the numer of file type in a file.
% 
% Author: Keith W.
% Ver.: 1.0
% Date: 18.02.2024

folder = file_name;  % 指定文件夹路径
fileType = file_type;   % 指定文件类型

% 使用dir函数获取文件夹下的所有符合文件类型的文件信息
fileList = dir(fullfile(folder, fileType));

% 获取文件数量
num = numel(fileList);
end