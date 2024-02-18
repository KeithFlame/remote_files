function [direction, intersection_point] = getCrossLine(n1,p1,n2,p2,y2)
    % 定义平面参数
    % 定义平面1参数
    n1=n1/norm(n1);
    n2=n2/norm(n2);
    a1 = n1(1); % 平面1的法向量的x分量
    b1 = n1(2); % 平面1的法向量的y分量
    c1 = n1(3); % 平面1的法向量的z分量
    
    % 定义平面2参数
    a2 = n2(1); % 平面2的法向量的x分量
    b2 = n2(2); % 平面2的法向量的y分量
    c2 = n2(3); % 平面2的法向量的z分量


    % 计算交线
    direction = cross([a1, b1, c1]', [a2, b2, c2]');

    % 直线参数
%     V = y2; % 直线的方向向量
    
    % 平面参数
    A = a1; % 平面的法向量的y分量
    B= b1; % 平面的法向量的z分量
    C = c1; % 平面的常数项
    D = -dot(n1,p1);
    
    % 计算 t 值
    t = (-D - A*p2(1) - B*p2(2) - C*p2(3)) / (A*y2(1) + B*y2(2) + C*y2(3));
    
    % 计算交点坐标
    intersection_point = p2 + t * y2;    
%     % 输出结果
%     disp('交线的方向向量：');
%     disp(direction);
%     
%     disp('交点坐标：');
%     disp(intersection_point);
end