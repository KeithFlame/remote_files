% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 08.30.2022
%% figure
figure;
hold on;grid on;axis equal;
xlabel("x");ylabel("y");zlabel("z");
title("fanrik");
view([0 0]);
is_plot = 0;
%% motion unit declarations
P = [0 0 0 0 0 0 0 0 0 0
    0 0 0 0 0 0 0 0 0 0
    0 100 200 300 400 500 600 700 800 900];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + P(:,2))/2,P(:,2),0);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3),1);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4),3);
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5),3);
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6),2);
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7),3);
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8),3);
joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9),1);
joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10),0);
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
    joint_unit4
    joint_unit5
    joint_unit6
    joint_unit7
    joint_unit8
    joint_unit9];
block_size = max(size(joint_units));
for i = 1:block_size
    ju = joint_units(i);
    ju.is_plot = is_plot;
    joint_units(i) = ju.plotUnit;
end
%% residual
err_p = 1e-3;
errp = 10;

%% target
joint_units(1).end_position = [30 0 90]';
joint_units(1).joint_position = [0 0 50]';
joint_units(end).end_position = [400 250 600]';
joint_units(end).joint_position = [370 250 560]';
joint_units(end).origin_position = [370 250 510]';
joint_units(end) = joint_units(end).refreshUnit;

joint_units(1) = joint_units(1).refreshUnit;
if(is_plot)
    joint_units(1).updateUnit;
    joint_units(end).updateUnit;
end
%% preparation
ee1 = joint_units(end).e1;
joint_position = joint_units(end).origin_position + ...
    ee1 * joint_units(end-1).l2;
if(is_plot)
P=[joint_units(end).origin_position joint_position joint_units(end-1).end_position];
a1 = plot3(P(1,:),P(2,:),P(3,:),'Color',[0 1 0 ],LineStyle='-.');
a2 = plot3(joint_position(1),joint_position(2),joint_position(3),...
    'Color',[1 0 0 ],'Marker','o',MarkerSize=10);
else
    a1 = [];
    a2 = [];
end
hd = [a1 a2];
for i = 2 : (block_size - 1)
    ju = single_backward(joint_units,i,hd,is_plot);
    joint_units(i) = ju;
end
%% executions
tic;
for iww = 1:10000
    iter = 0;
    joint_units(end).end_position = [400 250 600]';
    joint_units(end).joint_position = [370 250 560]';
    joint_units(end).origin_position = [370 100 510]' +rand(3,1)*10-5;
    joint_units(end) = joint_units(end).refreshUnit;
    errp = 10;
    while(errp>err_p)
        for i = (block_size - 1) :-1: 2
            ju = single_forward(joint_units,i,hd,is_plot);
            joint_units(i) = ju;
        end
        for i = 2 : (block_size - 1)
            ju = single_backward(joint_units,i,hd,is_plot);
            joint_units(i) = ju;
        end
        errp = norm(joint_units(8).end_position - joint_units(9).origin_position);
        iter = iter +1;
    end
end
toc;