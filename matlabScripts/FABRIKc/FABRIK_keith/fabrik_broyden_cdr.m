% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 09.02.2022

%% figure
figure;
hold on;grid on;axis equal;
xlabel("x");ylabel("y");zlabel("z");
title("fabrik Delta");
set(gca, 'FontSize', 18);
set(gca,'FontName','Times New Roman');
view([0 0]);
is_plot = 0;
is_write = 0;
if(is_write)
    dt = 1/256;
end
ti = zeros(999,2);
%% delta 机器人
for ipsi = 1:999
    q1 = mod(ipsi,10)*33+190;
    temq1=floor(ipsi/10);
    q2 = mod(temq1,10)*33+190;
    temq2=floor(temq1/10);
    q3 = mod(temq2,10)*33+190;
    q = [q1 q2 q3];%输入
[p0, p1, p2] = setQ(q);
p3 = p2 - [0 0 30]';
p4 = p3 - [0 0 30]';
p5 = p4 - [0 0 190]';
p6 = p5 - [0 0 190]';
p7 = p6 - [0 0 30]';
p8 = p7 - [0 0 30]';
p9 = p8 - [0 0 10]';
beta = [ 0 120/180*pi 240/180*pi];
p10 = p9-[cos(beta(1)) cos(beta(2)) cos(beta(3));
    sin(beta(1)) sin(beta(2)) sin(beta(3));
    0 0 0]*20;
% chain1
joint_unit1 = MotionUnit(p0(:,1), p1(:,1), p2(:,1), 1,[0 0 0]);
joint_unit2 = MotionUnit(p2(:,1), p3(:,1), p4(:,1), 3,[1 0 0 ]);
joint_unit3 = MotionUnit(p4(:,1), p5(:,1), p6(:,1), 0, [0.6 0.6 0.6]);
joint_unit4 = MotionUnit(p6(:,1), p7(:,1), p8(:,1), 3,[1 0 0 ]);
joint_unit5 = MotionUnit(p8(:,1), p9(:,1), p10(:,1), 0,[0 0 0]);
chain_1=[joint_unit1 joint_unit2 joint_unit3 joint_unit4 joint_unit5];
joint_unit1 = MotionUnit(p10(:,2), p9(:,2), p8(:,2), 1,[0 0 0]);
joint_unit2 = MotionUnit(p8(:,2), p7(:,2), p6(:,2), 3, [0 1 0]);
joint_unit3 = MotionUnit(p6(:,2), p5(:,2), p4(:,2), 0, [0.6 0.6 0.6]);
joint_unit4 = MotionUnit(p4(:,2), p3(:,2), p2(:,2), 3, [0 1 0]);
joint_unit5 = MotionUnit(p2(:,2), p1(:,2), p0(:,2), 0,[0 0 0]);
chain_2=[joint_unit1 joint_unit2 joint_unit3 joint_unit4 joint_unit5];
joint_unit1 = MotionUnit(p10(:,3), p9(:,3), p8(:,3), 1,[0 0 0]);
joint_unit2 = MotionUnit(p8(:,3), p7(:,3), p6(:,3), 3, [0 0 1]);
joint_unit3 = MotionUnit(p6(:,3), p5(:,3), p4(:,3), 0, [0.6 0.6 0.6]);
joint_unit4 = MotionUnit(p4(:,3), p3(:,3), p2(:,3), 3, [0 0 1]);
joint_unit5 = MotionUnit(p2(:,3), p1(:,3), p0(:,3), 0,[0 0 0]);
chain_3=[joint_unit1 joint_unit2 joint_unit3 joint_unit4 joint_unit5];

chain = [chain_1; chain_2; chain_3];
[chain_num, chain_element] = size(chain);
for j = 1:chain_num
    for i = 1:chain_element
        ju = chain(j,i);
        ju.is_plot = is_plot;
        chain(j,i) = ju.plotUnit;
    end
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','LoopCount',Inf,'DelayTime',dt);
end
%% residual
err_p = 1e-3;
errp = 10;
err_dis = errp;
broyden_number = 1;
J=[eye(6) eye(6) eye(6)]';
last_joint_position = [chain(1,2).joint_position; chain(1,4).joint_position; ...
        chain(2,2).joint_position; chain(2,4).joint_position; ...
        chain(3,2).joint_position; chain(3,4).joint_position;];
last_end_position = [chain(2,end-1).end_position; chain(3,end-1).end_position];
%% target
% q = [190 300 490];
% [p0, p1, p2] = setQ(q);
chain(1,1).origin_position =p0(:,1);chain(1,1).joint_position =p1(:,1);
chain(1,1).end_position =p2(:,1);
chain(2,end).origin_position =p2(:,2);chain(2,end).joint_position =p1(:,2);
chain(2,end).end_position =p0(:,2);
chain(3,end).origin_position =p2(:,3);chain(3,end).joint_position =p1(:,3);
chain(3,end).end_position =p0(:,3);

chain(1,1) = chain(1,1).refreshUnit;
chain(2,end) = chain(2,end).refreshUnit;
chain(3,end) = chain(3,end).refreshUnit;
if(is_plot)
    chain(1,1).updateUnit;
    if(is_write)
        [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
        imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
    end
    chain(2,end).updateUnit;
    if(is_write)
        [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
        imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
    end
    chain(3,end).updateUnit;
    if(is_write)
        [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
        imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
    end
    chain = setForward(chain);
    if(is_write)
        [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
        imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
    end
end

%% preparation
ee1 = chain(1,1).e2;
joint_position = chain(1,1).origin_position + ...
    ee1 * chain(1,2).l1;
if(is_plot)
    P=[chain(1,1).origin_position joint_position chain(1,2).end_position];
    a1 = plot3(P(1,:),P(2,:),P(3,:),'Color',[0 1 0 ],LineStyle='-.');
    a2 = plot3(joint_position(1),joint_position(2),joint_position(3),...
        'Color',[1 0 0 ],'Marker','o',MarkerSize=20);
else
    a1 = [];
    a2 = [];
end
hd = [a1 a2];
for j = 1:chain_num
        if(j == 1)
            for i = 2:(chain_element-2)
                ju = single_backward(chain(j,:),i,hd,is_plot);
                chain(j,i) = ju;
                    if(is_write)
                        [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                        imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
                    end
            end
            chain(j,chain_element-1).origin_position = chain(j,2).end_position + (chain(j,3).l1 + chain(j,3).l2)...
                    *chain(j,3).e2;
            chain(j,chain_element-1).joint_position = chain(j,2).e2*chain(j,2).l2...
                +chain(j,chain_element-1).origin_position;
            chain(j,chain_element-1).end_position = -chain(j,2).e1*chain(j,2).l1...
                +chain(j,chain_element-1).joint_position;
            chain(j,chain_element-1).updateUnit;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
            chain = setBackward(chain);
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
            continue;
        end
        
        for i = 2:(chain_element-2)
            ju = single_backward(chain(j,:),i,hd,is_plot);
            chain(j,i) = ju;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
        end
        chain(j,chain_element-1).origin_position = chain(j,2).end_position + (chain(j,3).l1 + chain(j,3).l2)...
                *chain(j,3).e2;
        chain(j,chain_element-1).joint_position = chain(j,2).e2*chain(j,2).l2...
            +chain(j,chain_element-1).origin_position;
        chain(j,chain_element-1).end_position = -chain(j,2).e1*chain(j,2).l1...
            +chain(j,chain_element-1).joint_position;
        chain(j,chain_element-1).updateUnit;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
end

%% executions
tic;
iter = 0;
while(errp>err_p)
    for j = chain_num:-1:1
        if(j>1)
            for i = (chain_element-1):-1:3
                ju = single_forward(chain(j,:),i,hd,is_plot);
                chain(j,i) = ju;
                if(is_write)
                    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
                end
            end
            chain(j,2).end_position = chain(j,3).origin_position;
            chain(j,2).joint_position = chain(j,4).e1*chain(j,4).l1...
                +chain(j,2).end_position;
            chain(j,2).origin_position = -chain(j,4).e2*chain(j,4).l2...
                +chain(j,2).joint_position;
            chain(j,2).updateUnit;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
            continue;
        end
        chain = setForward(chain);
        for i = (chain_element-1):-1:3
            ju = single_forward(chain(j,:),i,hd,is_plot);
            chain(j,i) = ju;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
        end
        chain(j,2).end_position = chain(j,3).origin_position;
        chain(j,2).joint_position = chain(j,4).e1*chain(j,4).l1...
            +chain(j,2).end_position;
        chain(j,2).origin_position = -chain(j,4).e2*chain(j,4).l2...
            +chain(j,2).joint_position;
        chain(j,2).updateUnit;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end
    
    for j = 1:chain_num
        if(j == 1)
            for i = 2:(chain_element-2)
                ju = single_backward(chain(j,:),i,hd,is_plot);
                chain(j,i) = ju;
                if(is_write)
                    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
                end
            end
            chain(j,chain_element-1).origin_position = chain(j,2).end_position + (chain(j,3).l1 + chain(j,3).l2)...
                    *chain(j,3).e2;
            chain(j,chain_element-1).joint_position = chain(j,2).e2*chain(j,2).l2...
                +chain(j,chain_element-1).origin_position;
            chain(j,chain_element-1).end_position = -chain(j,2).e1*chain(j,2).l1...
                +chain(j,chain_element-1).joint_position;
            chain(j,chain_element-1).updateUnit;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
            chain = setBackward(chain);
            continue;
        end
        
        for i = 2:(chain_element-2)
            ju = single_backward(chain(j,:),i,hd,is_plot);
            chain(j,i) = ju;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
        end
        chain(j,chain_element-1).origin_position = chain(j,2).end_position + (chain(j,3).l1 + chain(j,3).l2)...
                *chain(j,3).e2;
        chain(j,chain_element-1).joint_position = chain(j,2).e2*chain(j,2).l2...
            +chain(j,chain_element-1).origin_position;
        chain(j,chain_element-1).end_position = -chain(j,2).e1*chain(j,2).l1...
            +chain(j,chain_element-1).joint_position;
        chain(j,chain_element-1).updateUnit;
        if(is_write)
            [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
            imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
        end
    end
    errp = max([norm(chain(2,end-1).end_position - chain(2,end).origin_position), ...
        norm(chain(3,end-1).end_position - chain(3,end).origin_position)]);
    iter = iter +1;
%     err_dis = [err_dis errp];
    if(iter>10)
        current_joint_position = [chain(1,2).joint_position; chain(1,4).joint_position; ...
            chain(2,2).joint_position; chain(2,4).joint_position; ...
            chain(3,2).joint_position; chain(3,4).joint_position;];
        current_end_position = [chain(2,end-1).end_position; chain(3,end-1).end_position];
        dp = current_joint_position-last_joint_position;
        dx = current_end_position-last_end_position;
        J=BadBroydenJacobian(dp,dx,J);
        if(iter>15)
            dx_ = -[chain(2,end-1).end_position - chain(2,end).origin_position; ...
                chain(3,end-1).end_position - chain(3,end).origin_position];
            dp = J*dx_;
            chain(1,2).joint_position = chain(1,2).joint_position+dp(1:3);
            chain(1,4).joint_position = chain(1,4).joint_position+dp(4:6);
            chain(2,2).joint_position = chain(2,2).joint_position+dp(7:9);
            chain(2,4).joint_position = chain(2,4).joint_position+dp(10:12);
            chain(3,2).joint_position = chain(3,2).joint_position+dp(13:15);
            chain(3,4).joint_position = chain(3,4).joint_position+dp(16:18);
            broyden_number = broyden_number + 1;
        end
        last_joint_position = current_joint_position;
        last_end_position = current_end_position;
    end
end
TT=toc;
ti(ipsi,:)=[TT iter];
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
end
end
%% auxillary
function [p0, p1, p2] = setQ(q)
    q_max = 490;
    q_min = 190;
    for i = 1:3
        if(q(i)>q_max)
            q(i) = q_max;
        elseif(q(i)<q_min)
            q(i) = q_min;
        end
        alpha = 70/180*pi;
        beta = [ 0 120/180*pi 240/180*pi];
        p0 = zeros(3,3);
%         p0 = [cos(alpha)*cos(beta(1))*q_min cos(alpha)*cos(beta(2))*q_min cos(alpha)*cos(beta(3))*q_min;
%             cos(alpha)*sin(beta(1))*q_min cos(alpha)*sin(beta(2))*q_min cos(alpha)*sin(beta(3))*q_min;
%             sin(alpha)*q_min sin(alpha)*q_min sin(alpha)*q_min];

        p1 = [cos(alpha)*cos(beta(1))*q(1) cos(alpha)*cos(beta(2))*q(2) cos(alpha)*cos(beta(3))*q(3);
            cos(alpha)*sin(beta(1))*q(1) cos(alpha)*sin(beta(2))*q(2) cos(alpha)*sin(beta(3))*q(3);
            sin(alpha)*q(1) sin(alpha)*q(2) sin(alpha)*q(3)];
        p2 = p1 - [0 0 10]'; 
    end
end
function chain_after = setForward(chain_before)
    t2 = chain_before(2,2).origin_position;
    t3 = chain_before(3,2).origin_position;
    p = (t2 + t3)/2;
    tt1 = -chain_before(2,1).e1'*chain_before(1,end).e2;
    p0 = sin(acos(tt1)) * chain_before(2,1).l1...
        *(chain_before(2,1).e1+tt1*chain_before(1,end).e2)/...
        norm(chain_before(2,1).e1+tt1*chain_before(1,end).e2);
    

    chain_before(3,1).end_position = p + p0;
    chain_before(3,1).joint_position = p + p0 - chain_before(3,1).e2 * chain_before(3,1).l2;
    chain_before(3,1).origin_position = chain_before(3,1).joint_position + chain_before(3,1).e1 * chain_before(3,1).l1;
    chain_before(2,1).end_position = p - p0;
    chain_before(2,1).joint_position = p - p0 - chain_before(2,1).e2 * chain_before(2,1).l2;
    chain_before(2,1).origin_position = chain_before(2,1).joint_position + chain_before(2,1).e1 * chain_before(2,1).l1;
    
    chain_before(2,1) = chain_before(2,1).updateUnit;
    chain_before(3,1) = chain_before(3,1).updateUnit;
    

    chain_before(1,end).end_position = chain_before(2,1).origin_position;
    chain_before(1,end).joint_position = chain_before(1,end).end_position - chain_before(1,end).e2...
         *chain_before(1,end).l2;
    chain_before(1,end).origin_position = chain_before(1,end).joint_position + chain_before(1,end).e1...
        *chain_before(1,end).l1;
    chain_before(1,end) = chain_before(1,end).updateUnit;

    chain_after = chain_before;

end
function chain_after = setBackward(chain_before)
    t1 = chain_before(1,end-1).end_position;
    p = t1;
    chain_before(1,end).origin_position = p;
    chain_before(1,end).joint_position = p - chain_before(1,end).l1...
         *chain_before(1,end).e1;
    chain_before(1,end).end_position = chain_before(1,end).joint_position + chain_before(1,end).l2...
         *chain_before(1,end).e2;
    chain_before(1,end) = chain_before(1,end).updateUnit;

    chain_before(2,1).origin_position = chain_before(1,end).end_position;
    chain_before(2,1).joint_position = chain_before(2,1).origin_position - chain_before(2,1).l1...
         *chain_before(2,1).e1;
    chain_before(2,1).end_position = chain_before(2,1).joint_position + chain_before(2,1).l2...
         *chain_before(2,1).e2;
    chain_before(2,1) = chain_before(2,1).updateUnit;

    chain_before(3,1).origin_position = chain_before(1,end).end_position;
    chain_before(3,1).joint_position = chain_before(3,1).origin_position - chain_before(3,1).l1...
         *chain_before(3,1).e1;
    chain_before(3,1).end_position = chain_before(3,1).joint_position + chain_before(3,1).l2...
         *chain_before(3,1).e2;
    chain_before(3,1) = chain_before(3,1).updateUnit;
    chain_after = chain_before;

end