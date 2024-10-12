% This MATLAB script is to record a video as an experiment for the RAL
% article.
% 
% Author: Keith W.
% Ver.: 1.0
% Date: 18.02.2024


clear;clc;close;
date = 421;
file_path1 = ['./date0',num2str(date)];
pic_num1 = getFilesNum([file_path1, '/done2/'],'*.jpg');
[cur_pose1,tar_pose1] = getData(file_path1);

date = 422;
file_path2 = ['./date0',num2str(date)];
pic_num2 = getFilesNum([file_path2, '/done2/'],'*.jpg');
[cur_pose2,tar_pose2] = getData(file_path2);

date = 512;
file_path3 = ['./date0',num2str(date)];
pic_num3 = getFilesNum([file_path3, '/done2/'],'*.jpg');
[cur_pose3,tar_pose3] = getData(file_path3);

% 设置保存对象
aviobj = VideoWriter([file_path1,'/video/1.avi']);   
aviobj.FrameRate = 8;   % 画图的帧率，越大表示越快
open(aviobj);   % 打开AVI文件
assert(size(cur_pose1,3)==pic_num1);
for i = 1:pic_num1

end

close(aviobj);