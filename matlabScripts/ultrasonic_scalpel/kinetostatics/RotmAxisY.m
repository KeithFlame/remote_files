function rotm = RotmAxisY(xita)
% 输出绕Y轴旋转xita角的旋转矩阵
    rotm = [cos(xita) 0 sin(xita);
            0         1 0;
            -sin(xita)     0  cos(xita)];
end

