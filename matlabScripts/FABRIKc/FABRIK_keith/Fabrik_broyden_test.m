% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 08.30.2022
%% figure
is_plot = 0;
%% motion unit declarations
P = [0 0 0  0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0 0
    0 100 200 300 400 500 600 700 800 900 1000];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + P(:,2))/2,P(:,2),0);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3),3);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4),3);
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5),3);
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6),3);
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7),3);
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8),0);
joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9),2);
% joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,(P(:,8) + P(:,9))/2+[50 0 0]',2);
joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10),1);
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
    joint_unit4
    joint_unit5
    joint_unit6
    joint_unit7
%     joint_unit8
%     joint_unit9
    ];
block_size = max(size(joint_units));
for i = 2:block_size
    ju = joint_units(i);
    ju.is_plot = is_plot;
    joint_units(i) = ju.plotUnit;
end
%% residual
SL = [P(3,4)-P(3,3) P(3,5)-P(3,4) P(3,6)-P(3,5) P(3,7)-P(3,6)];
err_p = 1e-3;
errp = 10;
err_dis =  errp;
broyden_number = 1;
last_errp = 1000;
J=[eye(3) eye(3) eye(3)]';
last_end_position = joint_units(block_size-1).end_position;
last_joint_position = [joint_units(2).joint_position; ...
    joint_units(3).joint_position; joint_units(5).joint_position];
%% target
% tar_set = load('target.log');
% iter_block = zeros(50000,1);
% for ipsi = 1:1

% p=tar_set(ipsi,7:9)'; t=tar_set(ipsi,10:12)';
joint_units(end).end_position = 20*t+p; % [300 200 500]';
joint_units(end).joint_position = 10*t+p; %[270 200 460]';
joint_units(end).origin_position = p; %[270 200 410]';
joint_units(end) = joint_units(end).refreshUnit;

%% executions
iter = 0;
errp = 10*(rand()+0.5);
hd=[];
while(errp>err_p)
    for i = (block_size - 1) :-1: 2
        ju = single_forward(joint_units,i,hd,is_plot);
        joint_units(i) = ju;
    end
    for i = 2 : (block_size - 1)
        ju = single_backward(joint_units,i,hd,is_plot);
        joint_units(i) = ju;
    end
    iter = iter + 1;
    current_joint_position = [joint_units(2).joint_position; joint_units(3).joint_position; joint_units(5).joint_position];
    current_end_position = joint_units(block_size-1).end_position;
    dp = current_joint_position-last_joint_position;
    dx = current_end_position-last_end_position;
    J=BadBroydenJacobian(dp,dx,J);
    if((iter/5)>broyden_number)
        dp = J*(joint_units(end).origin_position-joint_units(block_size-1).end_position);
        joint_units(2).joint_position = joint_units(2).joint_position+dp(1:3);
        joint_units(3).joint_position = joint_units(3).joint_position+dp(4:6);
        joint_units(5).joint_position = joint_units(5).joint_position+dp(7:9);
        broyden_number = broyden_number + 1;
    end
    last_joint_position = current_joint_position;
    last_end_position = current_end_position;
    errp = norm(joint_units(block_size-1).end_position - joint_units(end).origin_position);
    if(iter>300)
        iter = 10000000;
        break;
    end
end
% iter_block(ipsi)=iter;
% end