function p0=lineXplane(n,d,t,p)
% n 平面法向量
% d 平面方程的D
% t 直线方向向量
% p 直线上一点
if(max(size(d))==3)
    P=d;
elseif(abs(n(3))>0)
    P = [1 1 -(n(1)+n(2)+d)/n(3)]';
end

n = n/norm(n);
t = t/norm(t);
dt = dot(n, (P - p));
dt = dt/dot(n,t);
p0 = p + dt * t;
end
