function dis= point_to_line_distance(p1,p2,v)

    xa = p2(1);  % 直线上的已知点 A 的 x 坐标
    ya = p2(2);  % 直线上的已知点 A 的 y 坐标
    za = p2(3);  % 直线上的已知点 A 的 z 坐标
    xv = v(1);  % 直线的方向向量 V 的 x 分量
    yv = v(2);  % 直线的方向向量 V 的 y 分量
    zv = v(3);  % 直线的方向向量 V 的 z 分量
    xp = p1(1);  % 待计算距离的点 P 的 x 坐标
    yp = p1(2);  % 待计算距离的点 P 的 y 坐标
    zp = p1(3);  % 待计算距离的点 P 的 z 坐标
    
    % 计算从点 A 到点 P 的向量 AP
    AP = [xp - xa, yp - ya, zp - za];
    V = [xv, yv, zv];
    dis = norm(cross(AP, V)) / norm(V);

end