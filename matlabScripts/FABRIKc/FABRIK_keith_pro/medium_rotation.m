% fabrik pro算法
%
% Author Keith W.
% Ver. 1.0
% Date 12.12.2022
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
    dt = 1/16;
end
%% motion unit declarations
P = [0   0   0   0   0   0   0   0   0   0
     0   0   0   0   0   0   0   0   0   0
     0 100 200 300 400 401 500 600 700 800];
e1 = [1 0 0]';e2 = [0 1 0]';e3 = [0 0 1]';
t = 0;
joint_units_type = [5 5 5 2 4 2 5 5 5];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + 9999*P(:,2))/10000,P(:,2), P(:,1) + e1, P(:,2) + e2, joint_units_type(1));
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3), P(:,2) + e1, P(:,3) + e2, joint_units_type(2));
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4), P(:,3) + e1, P(:,4) + e2, joint_units_type(3));
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5), P(:,4) + e1, P(:,5) + e2, joint_units_type(4));
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6), P(:,5)+ e1, P(:,6) + e2, joint_units_type(5));
joint_unit5d = MotionUnit(P(:,6), (P(:,5) + P(:,6))/2,P(:,5), P(:,6)+ e2, P(:,5) + e1, joint_units_type(5));
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7), P(:,6) + e1, P(:,7) + e2, joint_units_type(6));
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8), P(:,7) + e1, P(:,8) + e2, joint_units_type(7));
joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9), P(:,8) + e1, P(:,9) + e2, joint_units_type(8));
joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10), P(:,9) + e1, P(:,10) + e2, joint_units_type(9));
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
    ];
joint_units2 = [joint_unit9
    joint_unit8
    joint_unit7
    joint_unit6
    joint_unit5d
    ];
block_size = max(size(joint_units));
for i = 1:block_size
    ju = joint_units(i);
    ju.is_plot = is_plot;
    joint_units(i) = ju.plotUnit;

    ju = joint_units2(i);
    ju.is_plot = is_plot;
    joint_units2(i) = ju.plotUnit;
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','LoopCount',Inf,'DelayTime',dt);
end
%% residual
err_p = 1e-2;
errp = 10;
err_dis =  errp;
%% target

joint_units2(1).origin_direction_position = [300 300 410]';
joint_units2(1).origin_position = [300 300 400]';
joint_units2(1).joint_position = [300 250 400]';
joint_units2(1).end_position = [300 200 400]'; %[270 200 410]';
joint_units2(1).end_direction_position = [310 200 400]'; %[270 200 410]';
joint_units2(1) = joint_units2(1).refreshUnit;
if(is_plot)
    joint_units2(1).updateUnit;
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
        ju = single_forward_pro(joint_units,i,hd,is_plot, errp);
        joint_units(i) = ju;

        ju = single_forward_pro(joint_units2,i,hd,is_plot, errp);
        joint_units2(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end
    for i = 2 : (block_size - 1)
        ju = single_backward_pro(joint_units,i,hd,is_plot);
        joint_units(i) = ju;

        ju = single_backward_pro(joint_units2,i,hd,is_plot);
        joint_units2(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end

    e0 = (joint_units2(end - 1).joint_position - joint_units(end - 1).joint_position)/2;
    e0 = e0/norm(e0);
    joint_units(end).joint_position = (joint_units2(end - 1).joint_position + joint_units(end - 1).joint_position)/2;
    joint_units(end).origin_position = joint_units(end).joint_position - e0 * joint_units(end).l1;
    joint_units(end).end_position = joint_units(end).joint_position + e0 * joint_units(end).l1;


    joint_units(end).origin_direction_position = joint_units(end).origin_position +...
        calcNormLine(- e0,joint_units(4).de2);
    
    
    joint_units(end).end_direction_position = joint_units(end).end_position + ...
        calcNormLine(e0,joint_units2(4).de2);
    joint_units(end) = joint_units(end).updateUnit;

    joint_units2(end).origin_position = joint_units(end).end_position;
    joint_units2(end).origin_direction_position = joint_units(end).end_direction_position;
    joint_units2(end).joint_position = joint_units(end).joint_position;
    joint_units2(end).end_position = joint_units(end).origin_position;
    joint_units2(end).end_direction_position = joint_units(end).origin_direction_position;
    joint_units2(end) = joint_units2(end).updateUnit; 

    errp = norm(joint_units(4).end_position - joint_units(5).origin_position) + ...
        norm(joint_units2(4).end_position - joint_units2(5).origin_position);
    iter = iter +1;
    fprintf('当前为第 %d 次迭代，位置误差为 %6.4f。\n',iter,errp);
    err_dis = [err_dis errp];
    if(errp<0.5042)
        break;
    end
end
toc;
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end

function p0 = calcNormLine(p1,p2)
    p1 = p1/norm(p1);
    p2 = p2 / norm(p2);
    cross_p = cross(p1, p2);
    if(sum(abs(cross_p))<1e-6)
        p0 = p2;
    else
        cross_p = cross_p / norm(cross_p);
        p0 = cross(cross_p, p1);
        p0 = p0 / norm(p0);
    end
end