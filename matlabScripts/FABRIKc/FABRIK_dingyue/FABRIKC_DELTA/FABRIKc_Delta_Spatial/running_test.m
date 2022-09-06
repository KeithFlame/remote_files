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
end
display = [iteration_num;running_time];
[max_running_time,index] = max(running_time);
disp("max_runningtime:\t"); disp(max_running_time); disp("\t");disp(index);disp("\n");
[max_iteration_num,index] = max(iteration_num);
disp("max_iteration_num:\t");disp(max_iteration_num); disp("\t");disp(index);disp("\n");
disp("mean_iteration_num:\t");disp(mean(iteration_num)); disp("\n");
disp("mean_running_time:\t");disp(mean(running_time));disp("\n");

