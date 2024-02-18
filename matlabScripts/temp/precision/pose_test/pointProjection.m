function P_proj = pointProjection(plane,P)
% 点到平面的投影
% 点的坐标

% 平面的法向量

% 平面上一点的坐标
n=plane(1:3)/norm(plane(1:3));
if(abs(n(3))>0)
    p=[1 1 -(n(1)+n(2)+plane(4))/n(3)]';
end
% 计算点到平面的距离
d = dot(n, (P - p));

% 计算投影点的坐标
P_proj = P - d * n;
end