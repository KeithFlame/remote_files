x1=[-28.7300   66.8106  105.0004 93.3045    2.9628   30.4659 15]';
x2=[-134.7    48.7    107.0   96.5108   29.2577   55.8210 15]';
x1=[0 0 1 0 0 1 15]';
%   106.7716  -20.6499   77.0575  -21.0193  -82.6907   16.4559         0
% 103.6344  -28.2861   86.8156  -22.8248  -83.8900   14.4504         0
% 103.5760  -27.8230   86.5810  -22.8234  -83.8897   14.4517         0
x1 = [rand(6,1)*100; 5];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
A=[0 0 0 0 0 0 -1];
% x1=[144.8680   41.5290   90.0341   -7.7308  -91.7939   42.5359         0]';
[x,fmin,flag]=fmincon('cost_trocar_coord',x1,[],[],A,0,[],[],[],options);
T = fromX2T(x(1:6)),