
% % % vol1 = vol(1,:);
% % % vol2 = vol(2,:);
% % % vol3 = vol(3,:);


F = 20:10:2000;
a=-58.2;b=-387.5;omiga1=60.2;
u0=1266.5;
u = u0*omiga1./(omiga1+a*F.^b);

figure;
plot(F,u);

% options = optimoptions("fmincon","Display","iter","Algorithm","interior-point","PlotFcn","optimplotfval");
% x=[a b omiga1 u0]';
% [res,yh,exitFlag_keith] = fmincon("costFunction_1",x,[],[],[],[],[],[],[],options);

a3 = 1813;b3 = 0.0006228;c3 = 0.03152;
a2 = 1716;b2 = 0.0007142;c2 = 0.0208;
a1 = 1928;b1 = 0.0006089;c1 = 0.02673;
figure;hold on; grid on;
title("标定结果")
xlabel("Force (g)")
ylabel("voltage (mV)");
plot(F0,vol1,"bo");
plot(F0,vol2,"bo");
plot(F0,vol3,"bo");
a = mean([a1 a2 a3]);
b = mean([b1 b2 b3]);
c = mean([c1 c2 c3]);
u = a*sin(b.*F + c);
plot(F,u);


u = [1320 1380 1355 1267 1331 1388 1366 1320 1286];
F = (asin(u/a)-c)./b/1000*9.8