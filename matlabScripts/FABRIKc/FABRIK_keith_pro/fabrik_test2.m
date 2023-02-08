% fabrik pro算法
%
% Author Keith W.
% Ver. 1.0
% Date 11.10.2022
%% figure
figure;
hold on;
% grid on;
axis equal;
xlabel("x");ylabel("y");zlabel("z");
% set( gca, 'XTick', [], 'YTick', [],'ZTick', [] );
title("fabrik 2chain");
set(gca, 'FontSize', 18);
set(gca,'FontName','Times New Roman');
view([-54 15]);
axis([-100 100 -50 150 80 520]);
is_plot = 1;

is_write = 0;
if(is_write)
    dt = 1/4;
end
%% motion unit declarations
P = [0 0   0   0   0   0   0   0   0   0   0
     0 0   0   0   0   0   0   0   0   0   0
     0 100 200 300 400 500 600 700 800 900 1000];
e1 = [1 0 0]';e2 = [0 1 0]';e3 = [0 0 1]';
t = 0;
joint_units_type = [5 4 1 2 3 5 5 5 5 6 5];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + 9999*P(:,2))/10000,P(:,2), P(:,1) + e1, P(:,2) + e1, joint_units_type(1));
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3), P(:,2) + e1, P(:,3) + e2, joint_units_type(2),'c');
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4), P(:,3) + e2, P(:,4) - e1, joint_units_type(3),'r');
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5), P(:,4) - e1, P(:,5) - e2, joint_units_type(4),'g');
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6), P(:,5) - e2, P(:,6) + e1, joint_units_type(5),'b');
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7), P(:,6) + e1, P(:,7) + e2, joint_units_type(6));
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8), P(:,7) + e1, P(:,8) + e2, joint_units_type(7));
joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9), P(:,8) + e1, P(:,9) + e2, joint_units_type(8));
joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10), P(:,9) + e1, P(:,10) + e2, joint_units_type(9));
joint_unit10 = MotionUnit(P(:,10), (999*P(:,10) + P(:,11))/1000,P(:,11), P(:,10) + e1, P(:,11) + e2, joint_units_type(10),'m');
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
    joint_unit4
    joint_unit5
%     joint_unit6
%     joint_unit7
%     joint_unit8
%     joint_unit9
    joint_unit10
    ];
block_size = max(size(joint_units));
for i = 2:block_size
    ju = joint_units(i);
    ju.is_plot = is_plot;
    joint_units(i) = ju.plotUnit;
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','LoopCount',Inf,'DelayTime',dt);
end
%% residual
err_p = 1e-2;
err_a = 1e-2;
errp = 10;
erra = 10;
err_dis =  errp;
err_ang =  erra;
%% target

joint_units(end).end_direction_position = [10 200 400]';
joint_units(end).end_position = [00 200 400]';
joint_units(end).joint_position = [00 100.1 400]';
joint_units(end).origin_position = [00 100 400]'; %[270 200 410]';
joint_units(end).origin_direction_position = [0 100 410]'; %[270 200 410]';
joint_units(end) = joint_units(end).refreshUnit;
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
P=[joint_units(end).joint_position joint_position joint_units(end-2).joint_position];
a1 = plot3(P(1,:),P(2,:),P(3,:),'Color',[0 1 0 ],LineStyle='-.');
a2 = plot3(joint_position(1),joint_position(2),joint_position(3),...
    'Color',[1 0 0 ],'Marker','o',MarkerSize=20);
else
    a1 = [];
    a2 = [];
end
hd = [a1 a2];
% for i = 2 : (block_size - 1)
%     ju = single_backward_pro(joint_units,i,hd,is_plot);
%     joint_units(i) = ju;
%     if(is_write)
%         [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
%         imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
%     end
% end
%% executions
tic;
iter = 0;
while(errp>err_p || erra>err_a)
    for i = (block_size - 1) :-1: 2
        ju = single_forward_pro(joint_units,i,hd,is_plot,errp);
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
    erra = acos(joint_units(block_size-1).de2'*joint_units(block_size).de1);
    iter = iter +1;
    fprintf('当前为第 %d 次迭代，位置误差为 %6.4f。\n',iter,errp);
    err_dis = [err_dis errp];
    err_ang = [err_ang erra];
end
toc;
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end