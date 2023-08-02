x = [23.9179   21.1698   27.8803   20.3787]';
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x,fmin,flag]=fmincon('cost_4_Length',x,[],[],[],[],[],[],[],options);
