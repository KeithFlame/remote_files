clear
clc
% ÐüÁ´ÐÎÔö·ù¸Ë
beta = 1/9.3*log(4+sqrt(15));
x = 0:0.1:9.3;
R = 1.25*(exp(beta*(9.3-x))+exp(-beta*(9.3-x)))/2;
plot(x,R);
hold on;
plot(x,-R);
axis equal;
grid on;
box on; 
hold off;
