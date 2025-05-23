% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 08.30.2022
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
    0 0 0 0 0 0 0 0 0 0 0
    0 10 20 30 400 500 600 700 800 900 1000];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + P(:,2))/2,P(:,2),0);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3),3);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4),3);
% joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5),3);
% joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6),3);
% joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7),3);
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8),0);
% joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9),2);
% % joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,(P(:,8) + P(:,9))/2+[50 0 0]',2);
% joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10),1);
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
%     joint_unit4
%     joint_unit5
%     joint_unit6
    joint_unit7
%     joint_unit8
%     joint_unit9
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
joint_units(1).end_position = [16.3651  -25.7480  124.7638]';
joint_units(1).joint_position = -[0.0529   -0.9426    0.3296]'+[16.3651  -25.7480  124.7638]';
joint_units(1).origin_position = -2*[0.0529   -0.9426    0.3296]'+[16.3651  -25.7480  124.7638]';
joint_units(end).end_position = 2*[0.1214    0.9818   -0.1458]'+[11.1823   -5.0772  122.6402]'; % [300 200 500]';
joint_units(end).joint_position = [0.1214    0.9818   -0.1458]'+[11.1823   -5.0772  122.6402]'; %[270 200 460]';
joint_units(end).origin_position = [11.1823   -5.0772  122.6402]'; %[270 200 410]';
joint_units(end) = joint_units(end).refreshUnit;

joint_units(1) = joint_units(1).refreshUnit;
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
    ju = single_backward(joint_units,i,hd,is_plot);
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
        ju = single_forward(joint_units,i,hd,is_plot);
        joint_units(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end
    for i = 2 : (block_size - 1)
        ju = single_backward(joint_units,i,hd,is_plot);
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