
% % % vol1 = vol(1,:);
% % % vol2 = vol(2,:);
% % % vol3 = vol(3,:);


% % F = 2:0.1:20;
a=1;b=-0.5;omiga1=1;
% % u0=3.285;
% % u = u0*omiga1./(omiga1+a*F.^b);

% % % figure;
% % % plot(F,u);

options = optimoptions("fmincon","Display","iter","Algorithm","interior-point","PlotFcn","optimplotfval");
x=[a b omiga1]';
[res,yh,exitFlag_keith] = fmincon("costFunction_1",x,[],[],[],[],[],[],[],options);