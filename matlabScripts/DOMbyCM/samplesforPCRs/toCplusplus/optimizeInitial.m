
x=[-0.0182   -0.0087    0.0127   -0.0390]';
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
[x,y,flag_k]=fmincon('costInitial',x,[],[],[],[],[],[],[],options);