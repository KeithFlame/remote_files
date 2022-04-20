% this is validation for x=cos(x)


x=getAngle_simplfy;
norm(x)









function x=getAngle(a,b)
% this is a function to get ax + b = sin(x)
% input 1: the gain coefficient a
% input 2: the parameter b
%
% Author Keith. W
% Ver. 1.0
% Date 04.17.2022

end

function x=getAngle_simplfy
% this is a function to get x = sin(x)
    y0=1e-6;
    y1 = RK4(@Sophi_Int,y0,0.01,0,1);
%     y2=1/pi*(y1(end)*1i/2+1);
    y2=exp(y1(end))*pi/2;
    x=y2;
end

function [y] = RK4(f,y0,h,s0,s1)
%RK4 四阶龙格库塔积分
% f  函数
% y0:状态初值
% h:积分步长

% y:输出的状态矩阵，每一列对应一个s
% s0-s1:积分区间
s=s0:h:s1;
y=zeros(length(y0),length(s));
y(:,1)=y0;
for n = 1:length(s)-1
    %积分过程
    k1 = feval(f,s(n),y(:,n));
    k2 = feval(f,s(n)+h/2,y(:,n)+k1*h/2);
    k3 = feval(f,s(n)+h/2,y(:,n)+k2*h/2);
    k4 = feval(f,s(n)+h,y(:,n)+k3*h);
    y(:,n+1) = y(:,n) + (k1+2*k2+2*k3+k4)/6*h;
end
end
function y = Sophi_Int(~,t)
    x1=log((sqrt(1-t^2)+1)/t);
    x2=x1*(pi*t+2)*t;
    x3=x1^2*t^2-pi*t-1;
    y=atan(x2/x3)/t/pi;
%     y=log(-1i*exp(1i*(sin(t)-t)))/log(10);
end
