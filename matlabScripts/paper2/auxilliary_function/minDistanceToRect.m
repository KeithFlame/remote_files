function min_dist = minDistanceToRect(point, Rect)
P1 = Rect(:,1);P2 = Rect(:,2);P3 = Rect(:,3);P4 = Rect(:,4);
dist1 = minDistanceToSegment(point, P1, P2);
dist2 = minDistanceToSegment(point, P2, P3);
dist3 = minDistanceToSegment(point, P3, P4);
dist4 = minDistanceToSegment(point, P4, P1);
min_dist = min([dist1 dist2 dist3 dist4]);
end