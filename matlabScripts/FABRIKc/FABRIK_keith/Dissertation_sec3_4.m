% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 08.22.2024
%% figure
figure;
hold on;
% grid on;
axis equal;
font_size = 20;
xlabel("x (mm)",'FontName', 'Times New Roman', 'FontSize', font_size);
ylabel("y (mm)",'FontName', 'Times New Roman', 'FontSize', font_size);
zlabel("z (mm)",'FontName', 'Times New Roman', 'FontSize', font_size);
title("FM-FABRIKc",'FontName', 'Times New Roman', 'FontSize', font_size);
set(gca, 'FontSize', font_size)
view([0 0]);
axis([-50 350 -10 10 -100 250])
is_plot = 0;
is_broyden = 1;
% setFont()
%% motion unit declarations
P = [0 0 0  0 0 0 0 0 0 0 
    0 0 0 0 0 0 0 0 0 0 
    -2 0 100 200 300 400 500 502 600 602];
joint_unit1 = MotionUnit(P(:,1), (P(:,1) + P(:,2))/2,P(:,2),0);
joint_unit2 = MotionUnit(P(:,2), (P(:,2) + P(:,3))/2,P(:,3),3,[1 0 0]);
joint_unit3 = MotionUnit(P(:,3), (P(:,3) + P(:,4))/2,P(:,4),3,[0 1 0]);
joint_unit4 = MotionUnit(P(:,4), (P(:,4) + P(:,5))/2,P(:,5),3,[0 0 1]);
joint_unit5 = MotionUnit(P(:,5), (P(:,5) + P(:,6))/2,P(:,6),3,[0 1 0]);
joint_unit6 = MotionUnit(P(:,6), (P(:,6) + P(:,7))/2,P(:,7),3,[1 0 0]);
joint_unit7 = MotionUnit(P(:,7), (P(:,7) + P(:,8))/2,P(:,8),0);
joint_units = [
    joint_unit1
    joint_unit2
    joint_unit3
    joint_unit4
    joint_unit5
    joint_unit6
    joint_unit7
    ];
block_size = max(size(joint_units));
for i = 1:block_size
    ju = joint_units(i);
    ju.is_plot = is_plot;
    joint_units(i) = ju.plotUnit;
end
%% residual
% SL = [P(3,4)-P(3,3) P(3,5)-P(3,4) P(3,6)-P(3,5) P(3,7)-P(3,6)];
err_p = 1e-3;
% err_dis =  errp;
broyden_number = 1;
last_errp = 1000;
JBro=[eye(3) eye(3) eye(3) eye(3) eye(3)]';
[m1,n1]=size(JBro);
p_mu = zeros(m1,1);

for i = 2:block_size-1
    p_mu((i-2)*3+1:(i-1)*3)=joint_units(i).joint_position;
end
%% target
joint_units(1).origin_position=[-2 0 0]';
joint_units(1).joint_position = [-1 0 0]';
joint_units(1).end_position = [0 0 0]';
joint_units(1) = joint_units(1).refreshUnit;

joint_units(end).origin_position = [500 0 0]'; % [300 200 500]';
joint_units(end).joint_position = [501 0 0]'; %[270 200 460]';
joint_units(end).end_position = [502 0 0]'; %[270 200 410]';
joint_units(end) = joint_units(end).refreshUnit;
if(is_plot)
    joint_units(1).updateUnit;
    joint_units(end).updateUnit;
end
p_tar = joint_units(end).origin_position;
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
iter = 0;
errp = 10;
err_f=errp;
is_first_broyden = 0;
% hd=[];
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
    if(iter>10 && is_broyden == 1)
        current_joint_position = [joint_units(2).joint_position; ...
            joint_units(3).joint_position; joint_units(4).joint_position; ...
            joint_units(5).joint_position;joint_units(6).joint_position;];
        current_end_position = joint_units(block_size-1).end_position;
        
        if(is_first_broyden == 0)
            is_first_broyden = 1;
        else
            dp = current_joint_position-last_joint_position;
            dx = current_end_position-last_end_position;
            JBro=BadBroydenJacobian(dp,dx,JBro);
        end
        if(iter>15)
            dp = JBro*(joint_units(end).origin_position-joint_units(block_size-1).end_position);
            joint_units(2).joint_position = joint_units(2).joint_position+dp(1:3);
            joint_units(3).joint_position = joint_units(3).joint_position+dp(4:6);
            joint_units(4).joint_position = joint_units(4).joint_position+dp(7:9);
            joint_units(5).joint_position = joint_units(5).joint_position+dp(10:12);
            joint_units(6).joint_position = joint_units(6).joint_position+dp(13:15);
            % broyden_number = broyden_number + 1;
        end
        last_joint_position = current_joint_position;
        last_end_position = current_end_position;
    end
    errp = norm(joint_units(block_size-1).end_position - joint_units(end).origin_position);
    err_f = [err_f errp];
    % if(iter>300)
    %     % iter = 10000000;
    %     break;
    % end
end
err_f(1)=[];
% iter_block(ipsi)=iter;
% end


%%
font_size = 20;
x1=1:85348;
x2=1:25;
tt=[36.6933246580994	24.1668839377973	18.2647297605099	14.7598071204763	12.4181333003158	10.7354279018800	9.46439114533038	8.46864853190993	7.66650974523668	7.00592084806540	6.45207800145406	5.98079568449906	5.57472628042145	5.22109269687511	4.91026993937949	4.63486567262351	1.18826046696164	0.521066019366835	0.188030706075438	0.0733664012461678	0.0278785857587470	0.0107680115887228	0.00419794799616044	0.00169837950700464	0.000755645830540777];
% figure;
loglog(1:100,100:-1:1)
cla;

hold on;
grid on;
loglog(x1, err_f,'LineWidth', 2);
loglog(x2, tt,'LineWidth', 2);
xlabel("iteration",'FontName', 'Times New Roman', 'FontSize', font_size)
ylabel("error (mm)",'FontName', 'Times New Roman', 'FontSize', font_size)
legend('FABRIKc Delta','FM-FABRIKc');
set(gca, 'FontSize', font_size)
