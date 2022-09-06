clear
clc
% FABRIKc_delta 运行测试
q = zeros(2,1);
i = 0;
for q_1 = 100:10:300
    for q_2 = 100:10:300
        i = i+1;
        q(:,i) = [q_1;q_2];   
    end
end

running_time = zeros(1,i);
iteration_num = zeros(1,i);

for i = 1:size(q,2)
    q_input = q(:,i);
    [time,index] = FABRIKc_Delta(q_input);
    running_time(i) = time;
    iteration_num(i) = index;
end
display = [iteration_num;running_time];
[max_running_time,index] = max(running_time);
disp("max_runningtime:\t"); disp(max_running_time); disp("\t");disp(index);disp("\n");
[max_iteration_num,index] = max(iteration_num);
disp("max_iteration_num:\t");disp(max_iteration_num); disp("\t");disp(index);disp("\n");
disp("mean_iteration_num:\t");disp(mean(iteration_num)); disp("\n");
disp("mean_running_time:\t");disp(mean(running_time));disp("\n");