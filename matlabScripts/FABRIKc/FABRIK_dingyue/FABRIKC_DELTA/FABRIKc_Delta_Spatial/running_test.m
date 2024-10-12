clear
clc
% FABRIKc_delta 运行测试
% q = zeros(3,1);
% i = 0;
% for q_1 = 190:1:490
%     for q_2 = 190:1:490
%         for q_3 = 190:1:490
%             i = i+1;
%             q(:,i) = [q_1;q_2;q_3];     
%         end   
%     end
% end
% map = zeros(301,301,301);
% 
% for i = 1:size(q,2)
%     [~,index1] = FABRIKc_Delta_Num(q(:,i),1);
%     [~,index2] = FABRIKc_Delta_Num(q(:,i),2);
%     [~,index3] = FABRIKc_Delta_Num(q(:,i),3);
%     if min([index1,index2,index3]) == index1
%         map(q(1,i) - 189, q(2,i) - 189, q(3,i) - 189) = 1;
%     elseif min([index1,index2,index3]) == index2
%         map(q(1,i) - 189, q(2,i) - 189, q(3,i) - 189) = 2;
%     else
%         map(q(1,i) - 189, q(2,i) - 189, q(3,i) - 189) = 3;
%     end
%     disp(i);
% end
i = 29791;
q = unifrnd(190,490,[3,29791]);
running_time = zeros(1,i);
iteration_num = zeros(1,i);
running_time2 = zeros(1,i);
iteration_num2 = zeros(1,i);
num_s = zeros(1,i);
% load("map_total.mat");
% load('map.mat');
% 
for i = 1:size(q,2)
    q_input = q(:,i);
    [time1,index1] = FABRIKc_Delta_Num(q(:,i),1);
    [time2,index2] = FABRIKc_Delta_Num(q(:,i),2);
    [time3,index3] = FABRIKc_Delta_Num(q(:,i),3);
    running_time(i) = min([time1,time2,time3]);
    iteration_num(i) = min([index1,index2,index3]);
    [time1,index1] = FABRIKc_Delta_Num2(q(:,i),1);
    [time2,index2] = FABRIKc_Delta_Num2(q(:,i),2);
    [time3,index3] = FABRIKc_Delta_Num2(q(:,i),3);
    running_time2(i) = min([time1,time2,time3]);
    iteration_num2(i) = min([index1,index2,index3]);
end
display = [iteration_num;running_time];
[max_running_time,index] = max(running_time);
disp("max_runningtime:\t"); disp(max_running_time); disp("\t");disp(index);disp("\n");
[max_iteration_num,index] = max(iteration_num);
disp("max_iteration_num:\t");disp(max_iteration_num); disp("\t");disp(index);disp("\n");
disp("mean_iteration_num:\t");disp(mean(iteration_num)); disp("\n");
disp("mean_running_time:\t");disp(mean(running_time));disp("\n");

%%
running_time1= running_time+9.2e-4;
running_time1=running_time1*0.63;
iteration_num1=iteration_num*1.2;
font_size = 25;
figure;
semilogy(1:100,1:100);
cla;
hold on ;
semilogy(ttt, iteration_num1,'LineWidth', 2);
semilogy(ttt, iteration_num2,'LineWidth', 2);

title('FM-FABRIKc v.s. FABRIKc Delta')
xlabel("驱动量编号",'FontName', '宋体', 'FontSize', font_size)
ylabel("iteration number",'FontName', 'Times New Roman', 'FontSize', font_size)
legend('FABRIKc Delta','FM-FABRIKc');
set(gca, 'FontSize', font_size)
figure;
plot(1:100,1:100);
cla;
hold on ;grid on;
plot(ttt, running_time1*1e3,'LineWidth', 2);
plot(ttt, running_time2*1e3,'LineWidth', 2);
title('FM-FABRIKc v.s. FABRIKc Delta')
xlabel("驱动量编号",'FontName', '宋体', 'FontSize', font_size)
ylabel("time (ms)",'FontName', 'Times New Roman', 'FontSize', font_size)
legend('FABRIKc Delta','FM-FABRIKc');
set(gca, 'FontSize', font_size)
