% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 08.30.2022
%% figure
figure(1);
hold on;grid on;axis equal;
xlabel("x");ylabel("y");zlabel("z");
title("fabrik 2chain");
set(gca, 'FontSize', 18);
set(gca,'FontName','Times New Roman');
view([0 0]);
is_plot = 0;

is_write = 0;
if(is_write)
    dt = 1/64;
end
%% motion unit declarations
P = [0 0 0  0 0 0 0 0
    0 0 0 0 0 0 0 0
    -10 0 10 110 120 140 155 175];
% P = [0 0 0  63.6620 73.6620 93.6620 108.6620 128.6620
%      0 0 0  0 0 0 0 0
%     -10 0 10 73.6620 73.6620 73.6620 73.6620 73.6620];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + P(:,2))/2,P(:,2),0);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3),1);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4),3);
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5),0);
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6),3);
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7),0);
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8),0);
% joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,P(:,9),2);
% % joint_unit8 = MotionUnit(P(:,8), (P(:,8) + P(:,9))/2,(P(:,8) + P(:,9))/2+[50 0 0]',2);
% joint_unit9 = MotionUnit(P(:,9), (P(:,9) + P(:,10))/2,P(:,10),1);
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
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','LoopCount',Inf,'DelayTime',dt);
end
%% residual
err_p = 1e-3;
errp = 10;
err_dis =  errp;
broyden_number = 1;
J=[eye(3) eye(3) eye(3)]';
last_end_position = joint_units(block_size-1).end_position;
last_joint_position = [joint_units(2).joint_position; ...
    joint_units(3).joint_position; joint_units(5).joint_position];
%% target
% joint_units(1).end_position = [16.3651  -25.7480  124.7638]';
% joint_units(1).joint_position = -[0.0529   -0.9426    0.3296]'+[16.3651  -25.7480  124.7638]';
% joint_units(1).origin_position = -2*[0.0529   -0.9426    0.3296]'+[16.3651  -25.7480  124.7638]';
p=[53.9140  -82.6077  103.8377]'; t=[0.4386   -0.1682   -0.8828]';
% p=[-47.8304 72.7436 95.8108]'; t=[-0.3209 -0.3002 -0.8983]';

% p=[0 0 145]'; t=[0 0 1]';
joint_units(end).end_position = 20*t+p; % [300 200 500]';
joint_units(end).joint_position = 10*t+p; %[270 200 460]';
joint_units(end).origin_position = p; %[270 200 410]';
joint_units(end) = joint_units(end).refreshUnit;

joint_units(1) = joint_units(1).refreshUnit;
if(is_plot)
%     joint_units(1).updateUnit;
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
    iter = iter + 1;
    err_dis = [err_dis errp];
    if(iter>10)
        current_joint_position = [joint_units(2).joint_position; joint_units(3).joint_position; joint_units(5).joint_position];
        current_end_position = joint_units(block_size-1).end_position;
        dp = current_joint_position-last_joint_position;
        dx = current_end_position-last_end_position;
        
        J=BadBroydenJacobian(dp,dx,J);
        if(iter>15)
            dp = J*(joint_units(end).origin_position-joint_units(block_size-1).end_position);
            joint_units(2).joint_position = joint_units(2).joint_position+dp(1:3);
            joint_units(3).joint_position = joint_units(3).joint_position+dp(4:6);
            joint_units(5).joint_position = joint_units(5).joint_position+dp(7:9);
            broyden_number = broyden_number + 1;
        end
        last_joint_position = current_joint_position;
        last_end_position = current_end_position;
    end

end
toc;
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end