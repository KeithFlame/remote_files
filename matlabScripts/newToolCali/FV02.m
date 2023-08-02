
L1 = 100;Lr=10;L2=20;Lg=15;Lstem = 400;
K1 = 5;K2 = 0.6;zeta = 0.1;gamma1 = 0;
x = [L1 Lr L2 Lg Lstem K1 K2 zeta gamma1]';
block_size = size(x,1);
xg = rand(block_size,1).*(x_upper-x_lower)+x_lower;
% xg=[-0.3832  212.8971  103.7528    8.1215   21.7866    1.0044    7.9892    0.6950    3.9868 11.7400]';
options = optimoptions('fmincon','Algorithm','interior-point','Display','iter');
[xh,yh,exitflag] = fmincon('costFunc_newTool',xg,[],[],[],[],x_lower,x_upper,[],options); %,options
