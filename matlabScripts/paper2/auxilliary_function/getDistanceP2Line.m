function distance = getDistanceP2Line(P,A,B)

% 计算直线方向向量
AB = B - A;

% 计算点到直线的投影
AP = P - A;
t = dot(AP, AB) / dot(AB, AB);

% 限制投影在直线段内
t = max(0, min(1, t));

% 计算最近点
closestPoint = A + t * AB;

% 计算距离
distance = norm(P - closestPoint);

% 输出结果
% fprintf('点到直线的最小距离为: %.4f\n', distance);

end