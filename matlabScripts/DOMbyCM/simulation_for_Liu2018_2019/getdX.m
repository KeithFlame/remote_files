function dx = getdX(x1,x2)
%GETDX 此处显示有关此函数的摘要
%   此处显示详细说明
%x1 current
%x2 target
    dp = x1(1:3)-x2(1:3);
    r1 = eul2rotm(x1(4:6)');
    r2 = eul2rotm(x2(4:6)');
    dr = rotm2eul(r2'*r1)*180/pi;
    dx = [dp;dr'];
end

