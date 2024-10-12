data=load('doudong1.log');

figure; hold on;
for i = 2:6

    plot(data(:,i),'-');
end
legend("\phi","\theta_1","\delta_1","\theta_2","\delta_2");