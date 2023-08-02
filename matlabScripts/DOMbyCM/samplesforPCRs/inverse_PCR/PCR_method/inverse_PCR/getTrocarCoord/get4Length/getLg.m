x = [22.8135   21.2425   24.4250   25.7816]';
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x,fmin,flag]=fmincon('cost_4_Length',x,[],[],[],[],[],[],[],options);
