t=load("c=0.01.mat");
dis1=t.dis_block'*1000;
t=load("c=0.05.mat");
dis2=t.dis_block'*1000;
t=load("c=0.1.mat");
dis3=t.dis_block'*1000;
t=load("c=0.2.mat");
dis4=t.dis_block'*1000;
t=load("c=0.5.mat");
dis5=t.dis_block'*1000;
t=load("c=1.mat");
dis6=t.dis_block'*1000;

theta1=0:5:90;

values1 =[[theta1(1) dis1(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis1(1) dis1(1,:) dis1(end)]]) [theta1(end) dis1(end)]'];
values2 =[[theta1(1) dis2(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis2(1) dis2(1,:) dis2(end)]]) [theta1(end) dis2(end)]'];
values3 =[[theta1(1) dis3(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis3(1) dis3(1,:) dis3(end)]]) [theta1(end) dis3(end)]'];
values4 =[[theta1(1) dis4(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis4(1) dis4(1,:) dis4(end)]]) [theta1(end) dis4(end)]'];
values5 =[[theta1(1) dis5(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis5(1) dis5(1,:) dis5(end)]]) [theta1(end) dis5(end)]'];
values6 =[[theta1(1) dis6(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis6(1) dis6(1,:) dis6(end)]]) [theta1(end) dis6(end)]'];
figure;
hold on;grid on;
% title(name);
ax = gca;
ax.FontName = 'Times New Roman';
plot(values1(1,:),values1(2,:));
plot(values2(1,:),values2(2,:));
plot(values3(1,:),values3(2,:));
plot(values4(1,:),values4(2,:));
plot(values5(1,:),values5(2,:));
plot(values6(1,:),values6(2,:));
xlabel("1^s^t segment bending angle (°)",'FontSize',16);
ylabel("tip position difference (mm)",'FontSize',16);

legend("c=0.01 mm", "c=0.05 mm", "c=0.1 mm", "c=0.2 mm", "c=0.5 mm", "c=1 mm",'FontSize',12);




t=load("c=0.01_theta=90.mat");
dis1=t.dis_block'*1000;
t=load("c=0.05_theta=90.mat");
dis2=t.dis_block'*1000;
t=load("c=0.1_theta=90.mat");
dis3=t.dis_block'*1000;
t=load("c=0.2_theta=90.mat");
dis4=t.dis_block'*1000;
t=load("c=0.5_theta=90.mat");
dis5=t.dis_block'*1000;
t=load("c=1_theta=90.mat");
dis6=t.dis_block'*1000;

theta1=(0:5:90)+63;

values1 =[[theta1(1) dis1(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis1(1) dis1(1,:) dis1(end)]]) [theta1(end) dis1(end)]'];
values2 =[[theta1(1) dis2(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis2(1) dis2(1,:) dis2(end)]]) [theta1(end) dis2(end)]'];
values3 =[[theta1(1) dis3(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis3(1) dis3(1,:) dis3(end)]]) [theta1(end) dis3(end)]'];
values4 =[[theta1(1) dis4(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis4(1) dis4(1,:) dis4(end)]]) [theta1(end) dis4(end)]'];
values5 =[[theta1(1) dis5(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis5(1) dis5(1,:) dis5(end)]]) [theta1(end) dis5(end)]'];
values6 =[[theta1(1) dis6(1)]' spcrv([[theta1(1) theta1(1,:) theta1(end)];[dis6(1) dis6(1,:) dis6(end)]]) [theta1(end) dis6(end)]'];

figure;
hold on;grid on;
% title(name);
ax = gca;
ax.FontName = 'Times New Roman';
plot(values1(1,:),values1(2,:));
plot(values2(1,:),values2(2,:));
plot(values3(1,:),values3(2,:));
plot(values4(1,:),values4(2,:));
plot(values5(1,:),values5(2,:));
plot(values6(1,:),values6(2,:));
xlabel("feeding extension length (mm)",'FontSize',16);
ylabel("tip position difference (mm)",'FontSize',16);

legend("c=0.01 mm", "c=0.05 mm", "c=0.1 mm", "c=0.2 mm", "c=0.5 mm", "c=1 mm",'FontSize',12);