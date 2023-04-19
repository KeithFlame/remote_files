x1=[-28.7300   66.8106  105.0004 93.3045    2.9628   30.4659]';
x2=[-134.7    48.7    107.0   96.5108   29.2577   55.8210]';

options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x,fmin,flag]=fmincon('cost_trocar_coord',x1,[],[],[],[],[],[],[],options);