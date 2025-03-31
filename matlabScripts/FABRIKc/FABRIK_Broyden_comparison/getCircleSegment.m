function arcPoints3D = getCircleSegment(startPoint,endPoint,thirdPoint,bendAngle)
    
    a = startPoint;
    b = endPoint;
    c = thirdPoint;
    angle = bendAngle;
    % 计算平面的法向量
    AB = b - a; % 向量 AB
    AC = c - a; % 向量 AC
    normal = cross(AB, AC); % 计算法向量
    normal = normal / norm(normal); % 单位法向量
    
    % 计算中点 m
    m = (a + b) / 2; % AB 的中点
    
    % 计算 ad 和 bd 的长度
    r = norm(AB) / (2 * sin(angle / 2)); % 根据夹角计算 ad 和 bd 的长度
    
    % 确保 d 在垂直平分线上
    direction = (b - a) / norm(b - a); % 从 a 到 b 的单位向量
    perpendicular = cross(normal, direction); % 垂直于 AB 的方向
    perpendicular = perpendicular / norm(perpendicular); % 单位化
    if dot(perpendicular,c-a)>0
        perpendicular = - perpendicular;
    end
    
    % 计算点 d 的位置
    d = m + r * perpendicular * cos(angle / 2); % 点 d 在垂直平分线上的位置
    
    % 计算垂直平分线与圆的交点
    intersection = d; % 初始交点为 d
    
    % 生成圆弧的点
    theta = linspace(-angle / 2, angle / 2, 100); % 从交点向两侧绘制圆弧的角度范围
    arcPoints = r * [cos(theta); sin(theta); zeros(1,100)]; % 圆弧的点在平面内
    if dot(perpendicular,d-a)>0
        perpendicular = - perpendicular;
    end
    y_ = cross(normal,perpendicular);
    y_ = y_/norm(y_);
    Rot = [perpendicular' y_' normal'];
    % 将圆弧点转换到三维空间
    arcPoints3D = zeros(3, length(arcPoints));
    for i = 1:length(arcPoints)
        % 计算每个圆弧点的位置
        arcPoints3D(:, i) = Rot*arcPoints(:,i)+d'; 
    end

end