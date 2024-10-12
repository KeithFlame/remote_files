function [cur_pose,tar_pose] = getData(file_path)
% acquire the current pose and target pose.
% 
% Author: Keith W.
% Ver.: 1.0
% Date: 18.02.2024

    cur_pose7 = load([file_path, '/data/data_cur.log']);
    tar_pose7 = load([file_path, '/data/data_tar.log']);
    
    cur_pose7 = cur_pose7(:,1:7);
    tar_pose7 = tar_pose7(end,1:7);
    num = size(cur_pose7,1);
    cur_pose = zeros(4,4,num);
    tar_pose = getTransformationFromVec7(tar_pose7);
    for i = 1:num
        T = getTransformationFromVec7(cur_pose7(i,:));
        cur_pose(:,:,i) = T;
    end

end

