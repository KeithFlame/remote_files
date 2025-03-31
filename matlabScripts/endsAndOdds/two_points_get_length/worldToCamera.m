function [p1_cam, p2_cam] = worldToCamera(intrinsic, point1, point2)
    % worldToCamera 将空间点转换到相机坐标系
    % 输入：
    % intrinsic - 相机内参矩阵 (3x3)
    % point1 - 空间点 1 (3x1)
    % point2 - 空间点 2 (3x1)
    %
    % 输出：
    % p1_cam - 空间点1在相机坐标系下的坐标 (3x1)
    % p2_cam - 空间点2在相机坐标系下的坐标 (3x1)

    % 确保输入点是列向量
    if size(point1, 2) ~= 1
        point1 = point1(:);
    end
    if size(point2, 2) ~= 1
        point2 = point2(:);
    end

    % % 将空间点转换为齐次坐标
    % point1_homogeneous = [point1; 1]; % (4x1)
    % point2_homogeneous = [point2; 1]; % (4x1)
    % 
    % % 计算相机坐标系下的坐标
    % p1_cam_homogeneous = intrinsic * point1_homogeneous; % (3x1)
    % p2_cam_homogeneous = intrinsic * point2_homogeneous; % (3x1)
    % 
    % % 转换为非齐次坐标
    % p1_cam = p1_cam_homogeneous(1:3) / p1_cam_homogeneous(3); % (3x1)
    % p2_cam = p2_cam_homogeneous(1:3) / p2_cam_homogeneous(3); % (3x1)
    u1 = intrinsic(1,3)-intrinsic(1,1)*point1(1)/point1(3);
    v1 = intrinsic(2,3)-intrinsic(2,2)*point1(2)/point1(3);
    p1_cam = [u1 v1];

    u2 = intrinsic(1,3)-intrinsic(1,1)*point2(1)/point2(3);
    v2 = intrinsic(2,3)-intrinsic(2,2)*point2(2)/point2(3);
    p2_cam = [u2 v2];
    
end