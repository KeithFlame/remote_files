function dist = minDistanceToSegment(point, segmentPoint1, segmentPoint2)
    % 计算点到线段的最小距离
    % 线段的方向向量
    segmentDir = segmentPoint2 - segmentPoint1;
    
    % 线段的长度的平方
    segmentLengthSq = dot(segmentDir, segmentDir);
    
    % 特殊情况：线段长度为0
    if segmentLengthSq == 0
        dist = norm(point - segmentPoint1);
        return;
    end
    
    % 计算投影参数t
    t = dot(point - segmentPoint1, segmentDir) / segmentLengthSq;
    
    % 限制 t 的范围到 [0, 1]
    t = max(0, min(1, t));
    
    % 找到线段上最近的点
    closestPoint = segmentPoint1 + t * segmentDir;
    
    % 计算距离
    dist = norm(point - closestPoint);
end