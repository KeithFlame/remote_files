% this is a function to optimize the new tool's structural paramters

ga=0;La=200;L1=100;Lr=10;L2=20.4;k11=5;k12=5;k21=1.4;k22=1.4;rp=11;Lg=15;
x=[ga La L1 Lr L2 k11 k12 k21 k22 rp]';
block_size = size(x,1);
% x_upper=[1 6 1.08 1.2 2.2 8 8 4 4 1.4 ]';
% x_lower=[-1 0 0.92 0.8 1.8 1 1 0.01 0.01 1.0]';

x_upper=[10 600 108 12 22 8 8 4 4 14 ]';
x_lower=[-10 0 92 8 18 1 1 0.01 0.01 10]';
xg = rand(block_size,1).*(x_upper-x_lower)+x_lower;
xg=[-5.5654  254.1144   93.5236    8.0000   18.0001    1.0000    8.0000    0.5277    4.0000 12.1610]';
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
[xh,yh,exitflag] = fmincon('costFunc_newTool',xg,[],[],[],[],x_lower,x_upper,[],options); %,options
