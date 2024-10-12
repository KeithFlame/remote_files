function rotm = RotmAxisZ(xita)
% 输出绕Z轴旋转xita角的旋转矩阵
    rotm = [cos(xita) -sin(xita) 0;
            sin(xita) cos(xita) 0;
            0         0          1;];
end

