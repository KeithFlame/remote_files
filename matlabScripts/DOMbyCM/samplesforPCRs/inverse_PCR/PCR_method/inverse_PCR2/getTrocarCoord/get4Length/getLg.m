x = [20.1044   26.4268   28.5811   20.6790]';
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x,fmin,flag]=fmincon('cost_4_Length',x,[],[],[],[],[],[],[],options);
