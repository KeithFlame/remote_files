% fabrik pro算法
%
% Author Keith W.
% Ver. 1.0
% Date 11.10.2022
%% figure
figure;
hold on;grid on;axis equal;
xlabel("x");ylabel("y");zlabel("z");
title("fabrik 2chain");
set(gca, 'FontSize', 18);
set(gca,'FontName','Times New Roman');
view([0 0]);
is_plot = 1;

is_write = 0;
if(is_write)
    dt = 1/4;
end
%% motion unit declarations
P = [0 0  0 0 0 0 0 0 0 0 0
    0 50 0 0 0 0 0 0 0 0 0
    0 90 200 300 400 500 600 700 800 900 1000];
e1 = [1 0 0]';e2 = [0 1 0]';e3 = [0 0 1]';
t = 0;
joint_unit1 = MotionUnit(P(:,1), [P([1 2],1); P(3,2)],P(:,2), P(:,1) + e1 + t * rand(3, 1), P(:,2) + e3 + t * rand(3, 1), 7);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3), P(:,2) + e1 + t * rand(3, 1), P(:,3) + e2 + t * rand(3, 1), 4);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4), P(:,3) + e1 + t * rand(3, 1), P(:,4) + e2 + t * rand(3, 1), 7);
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5), P(:,4) + e1 + t * rand(3, 1), P(:,5) + e2 + t * rand(3, 1), 7);
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6), P(:,5) + e1 + t * rand(3, 1), P(:,6) + e2 + t * rand(3, 1), 7);
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7), P(:,6) + e1 + t * rand(3, 1), P(:,7) + e2 + t * rand(3, 1), 4);
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8), P(:,7) + e1 + t * rand(3, 1), P(:,8) + e2 + t * rand(3, 1), 7);
joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9), P(:,8) + e1 + t * rand(3, 1), P(:,9) + e2 + t * rand(3, 1), 7);
joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10), P(:,9) + e1 + t * rand(3, 1), P(:,10) + e2 + t * rand(3, 1), 5);
joint_unit10 = MotionUnit(P(:,10), (P(:,10) + P(:,11))/2,P(:,11), P(:,10) + e1 + t * rand(3, 1), P(:,11) + e2 + t * rand(3, 1), 5);
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
    joint_unit4
    joint_unit5
    joint_unit6
    joint_unit7
    joint_unit8
%     joint_unit9
%     joint_unit10
    ];
block_size = max(size(joint_units));
for i = 1:block_size
    ju = joint_units(i);
    ju.is_plot = is_plot;
    joint_units(i) = ju.plotUnit;
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','LoopCount',Inf,'DelayTime',dt);
end
%% residual
err_p = 1e-3;
errp = 10;
err_dis =  errp;
%% target
tem = joint_units(1).end_direction_position - joint_units(1).end_position;
tem = [0 1 0;-1 0 0;0 0 0]*tem;
joint_units(1).joint_position = joint_units(1).end_position - 1e-4 * tem/norm(tem); 

joint_units(end).origin_position = [-22.1582 145.8272 373.4736]'; %[270 200 410]';
joint_units(end).origin_direction_position = [-15.1093 140.0971 369.2928]'; %[270 200 410]';
tem = joint_units(end).origin_direction_position - joint_units(end).origin_position;
tem = [0 1 0;-1 0 0;0 0 0]*tem;
joint_units(end).joint_position = joint_units(end).origin_position + 1e-4 * tem/norm(tem); 
if(is_plot)
    joint_units(1).updateUnit;
    joint_units(end).updateUnit;
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
end
%% preparation
ee1 = joint_units(end).e1;
joint_position = joint_units(end).origin_position + ...
    ee1 * joint_units(end-1).l2;
if(is_plot)
P=[joint_units(end).origin_position joint_position joint_units(end-1).end_position];
a1 = plot3(P(1,:),P(2,:),P(3,:),'Color',[0 1 0 ],LineStyle='-.');
a2 = plot3(joint_position(1),joint_position(2),joint_position(3),...
    'Color',[1 0 0 ],'Marker','o',MarkerSize=20);
else
    a1 = [];
    a2 = [];
end
hd = [a1 a2];
for i = 2 : (block_size - 1)
    ju = single_backward_pro(joint_units,i,hd,is_plot);
    joint_units(i) = ju;
    if(is_write)
        [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
        imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
    end
end
%% executions
tic;
iter = 0;
while(errp>err_p)
    for i = (block_size - 1) :-1: 2
        ju = single_forward_pro(joint_units,i,hd,is_plot);
        joint_units(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end
    for i = 2 : (block_size - 1)
        ju = single_backward_pro(joint_units,i,hd,is_plot);
        joint_units(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end
    errp = norm(joint_units(block_size-1).end_position - joint_units(end).origin_position);
    iter = iter +1
    err_dis = [err_dis errp];
end
toc;
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end