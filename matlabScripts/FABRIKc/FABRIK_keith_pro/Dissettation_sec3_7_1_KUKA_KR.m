% fabrik pro算法
%
% Author Keith W.
% Ver. 1.0
% Date 12.05.2022
%% figure
figure;
hold on;grid on;axis equal;
xlabel("x (mm)");ylabel("y (mm)");zlabel("z (mm)");
title("FM-FABRIKc");
set(gca, 'FontSize', 20);
set(gca,'FontName','Times New Roman');
view([-120 20]);
axis([-50 200 -50 200 -10 310])
is_plot = 1;

is_write = 0;
if(is_write)
    dt = 1/4;
end
%% motion unit declarations
P = [ 0  0 0   0   0   0   0    0  
      0  0 0   0   0   0   0    0   
     -4 -2 0 100 200 300 400  410 ];
e1 = [1 0 0]';e2 = [0 1 0]';e3 = [0 0 1]';
t = 0;
joint_units_type = [5 5 4 5 0 5 0 6 5];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + P(:,2))/2,P(:,2), e2,  e2, 0);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3), e2, e2, 4,[0 0 0]);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4), e2, -e2, 6,[1 0 0]);
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5), e2, e2, 5,[0 1 0]);
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6), e2, e2, 6,[0 0 1]);
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7), e2, e2, 0,[1 1 1]);
% joint_unit7 = MotionUnit(P(:,7), [P(1,8);P(2,8);450],P(:,8), P(:,7) + e1, P(:,8) + e1, joint_units_type(7));
% joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9), P(:,8) + e1, P(:,9) + e1, joint_units_type(8));
% joint_unit9 = MotionUnit(P(:,9), (999*P(:,9) + P(:,10))/1000,P(:,10), P(:,9) + e1, P(:,10) + e1, joint_units_type(9));
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
    joint_unit4
    joint_unit5
    joint_unit6
    % joint_unit7
    % joint_unit8
    % joint_unit9
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
err_p = 1e-2;
errp = 10;
err_dis =  errp;
%% target

joint_units(end).end_direction_position = e2;
joint_units(end).end_position = [102 100 200]';
joint_units(end).joint_position = [101 100 200]';
joint_units(end).origin_position = [100 100 200]'; %[270 200 410]';
joint_units(end).origin_direction_position = e2; %[270 200 410]';
joint_units(end) = joint_units(end).refreshUnit;
plot_orientation(joint_units(end).origin_position,[e1,-e3,e2],50,['k','k','k']);
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
err_flag = 0;
while(errp>err_p)
    for i = (block_size - 1) :-1: 2
        ju = single_forward_pro(joint_units,i,hd,is_plot, err_flag);
        joint_units(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
        err_flag = calcSum(joint_units);
    end
    for i = 2 : (block_size - 1)
        ju = single_backward_pro(joint_units,i,hd,is_plot);
        joint_units(i) = ju;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
        err_flag = calcSum(joint_units);
    end
    errp0 = norm(joint_units(block_size-1).end_position - joint_units(end).origin_position);
    iter = iter +1;
    fprintf('当前为第 %d 次迭代，位置误差为 %6.6f；相比上次的减少量为%6.6f。\n',iter,errp0,errp-errp0);
    errp = errp0;
    err_dis = [err_dis errp];

    if(iter == 20)
        break;
    end
         end
toc;
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end

function ef = calcSum(mus)
    ef = 0;
    len = size(mus,1);
    for i = 1:len
        ef = ef + mus(i).err_flag;
    end

end