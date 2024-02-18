function [d]=DistanceBetweenPointAndLine(p0,p1,p2)
%distance between p0 and line p1p2
d = norm(cross(p2-p1,p0-p1))/norm(p2-p1);
end