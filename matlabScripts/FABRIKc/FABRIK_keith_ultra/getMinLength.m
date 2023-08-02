x=[0 0]';
A=[-1 0
    1 0
    0 -1
    0 1];
b=[-pi/6 pi*5/6 -20 30]';
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x,fmin,flag]=fmincon('cost_FABRIK',x,A,b,[],[],[],[],[],options);