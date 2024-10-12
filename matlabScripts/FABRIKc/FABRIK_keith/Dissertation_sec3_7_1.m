% fabrik 算法
%
% Author Keith W.
% Ver. 1.0
% Date 09.04.2022

%% figure
figure;
hold on;
% grid on;
axis equal;
xlabel("x (mm)");ylabel("y (mm)");zlabel("z (mm)");
view([-40 15]);
title("FM-FABRIKc");
set(gca, 'FontSize', 20);
set(gca,'FontName','Times New Roman');

view([0 90]);
axis([-400 400 -400 400])
is_plot = 1;
is_write = 0;
if(is_write)
    dt = 1/32;
end
%% Tendon-Actuated Planar Parallel Continuum Robot
q = [15 25 35]*pi/180;%输入
[p0, p1, p2] = setQ(q);
p3 = p2 - [0 10 0]';
p4 = p3 - [0 82 0]';
p5 = p4 - [0 82 0]';
p6 = p5 - [0 30 0]';
p7 = p6 - [0 3 0]';
p8 = p7 - [0 3 0]';
% chain1
joint_unit1 = MotionUnit(p0(:,1), p1(:,1), p2(:,1), 3,[1,1,1]);
joint_unit2 = MotionUnit(p2(:,1), p3(:,1), p4(:,1), 2,[1,0,0]);
joint_unit3 = MotionUnit(p4(:,1), p5(:,1), p6(:,1), 2,[0,1,0]);
joint_unit4 = MotionUnit(p6(:,1), p7(:,1), p8(:,1), 0,[0,0,1]);
chain_1=[joint_unit1 joint_unit2 joint_unit3 joint_unit4];
% chain2
joint_unit1 = MotionUnit(p8(:,2), p7(:,2), p6(:,2), 0,[0,0,1]);
joint_unit2 = MotionUnit(p6(:,2), p5(:,2), p4(:,2), 2,[0,1,0]);
joint_unit3 = MotionUnit(p4(:,2), p3(:,2), p2(:,2), 2,[1,0,0]);
joint_unit4 = MotionUnit(p2(:,2), p1(:,2), p0(:,2), 3,[1,1,1]);
chain_2=[joint_unit1 joint_unit2 joint_unit3 joint_unit4];
% chain3
joint_unit1 = MotionUnit(p8(:,3), p7(:,3), p6(:,3), 0,[0,0,1]);
joint_unit2 = MotionUnit(p6(:,3), p5(:,3), p4(:,3), 2,[0,1,0]);
joint_unit3 = MotionUnit(p4(:,3), p3(:,3), p2(:,3), 2,[1,0,0]);
joint_unit4 = MotionUnit(p2(:,3), p1(:,3), p0(:,3), 3,[1,1,1]);
chain_3=[joint_unit1 joint_unit2 joint_unit3 joint_unit4];

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
err_p = 5e-1;
errp = 10;
err_dis = errp;
%% target
q = [50 50 50] * pi / 180;
[p0, p1, p2] = setQ(q);
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
            for i = 2:(chain_element-1)
                ju = single_backward(chain(j,:),i,hd,is_plot);
                chain(j,i) = ju;
                if(is_write)
                    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
                end
            end
            chain = setBackward(chain);
            continue;
        end
        
        for i = 2:(chain_element-1)
            ju = single_backward(chain(j,:),i,hd,is_plot);
            chain(j,i) = ju;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
        end
end
%% executions
tic;
iter = 0;
while(errp>err_p)
    for j = chain_num:-1:1
        if(j>1)
            for i = (chain_element-1):-1:2
                ju = single_forward(chain(j,:),i,hd,is_plot);
                chain(j,i) = ju;
                if(is_write)
                    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
                end
            end
            continue;
        end
        chain = setForward(chain);
        for i = (chain_element-1):-1:2
            ju = single_forward(chain(j,:),i,hd,is_plot);
            chain(j,i) = ju;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
        end 
    end
    
    for j = 1:chain_num
        if(j == 1)
            for i = 2:(chain_element-1)
                ju = single_backward(chain(j,:),i,hd,is_plot);
                chain(j,i) = ju;
                if(is_write)
                    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
                end
            end
            chain = setBackward(chain);
            continue;
        end
        
        for i = 2:(chain_element-1)
            ju = single_backward(chain(j,:),i,hd,is_plot);
            chain(j,i) = ju;
            if(is_write)
                [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
                imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
            end
        end
    end
    errp = max([norm(chain(2,end-1).end_position - chain(2,end).origin_position), ...
        norm(chain(3,end-1).end_position - chain(3,end).origin_position)]);
    iter = iter +1;
    err_dis = [err_dis errp];
end
toc;
if(is_plot)
    set(hd(1),'LineStyle','none');
    set(hd(2),'Marker','none');
end
if(is_write)
    [I,map] = rgb2ind(frame2im(getframe(gcf)),128);
    imwrite(I,map,'test.gif','WriteMode','append','DelayTime',dt);
end
%% Auxillary function
function [p0, p1, p2] = setQ(q)
    alpha = [-120 -240 0] *pi/180;
    r_base = 285;
    lc = 125;
    q(find(abs(q)<1e-6)) =1e-6;
    p = @(theta)[lc/theta*(1-cos(theta)) lc/theta*sin(theta) 0]';
    l1 = @(theta)lc/theta*tan(theta/2);
    r = @(delta)[cos(delta) -sin(delta) 0;sin(delta) cos(delta) 0; 0 0 1];

    p0 = [r_base*sin(alpha(1)) r_base*sin(alpha(2)) 0;
        r_base*cos(alpha(1)) r_base*cos(alpha(2)) r_base;
        0 0 0];
    p2 = [r(alpha(3))*p(q(1)) r(alpha(2))*p(q(2)) r(alpha(1))*p(q(3))] +p0;
    e1 = [0 1 0]';
    p1 = [l1(q(1))*r(alpha(3))*e1 l1(q(2))*r(alpha(2))*e1 l1(q(3))*r(alpha(1))*e1] +p0;
end

function chain_after = setForward(chain_before)
    t2 = chain_before(2,2).origin_position;
    t3 = chain_before(3,2).origin_position;
    p = (t2 + t3)/2;
    beta = 120 *pi/180;
    lc = chain_before(3,1).l1 + chain_before(3,1).l2;
    e1 = (t3-t2)/norm(t3-t2);
    e2 = [e1(2) -e1(1) 0]';
    tt1 = sin(beta) * lc;
    p0 = tt1 * e1;
    p1 = cos(beta) * lc *e2;
    

    chain_before(3,1).end_position = p + p0;
    chain_before(3,1).origin_position = p + p1;
    chain_before(3,1).joint_position = (chain_before(3,1).end_position + chain_before(3,1).origin_position)/2;
    chain_before(2,1).end_position = p - p0;
    chain_before(2,1).origin_position = p + p1;
    chain_before(2,1).joint_position = (chain_before(2,1).end_position + chain_before(2,1).origin_position)/2;
    
    chain_before(2,1) = chain_before(2,1).updateUnit;
    chain_before(3,1) = chain_before(3,1).updateUnit;
    

    chain_before(1,end).end_position = chain_before(2,1).origin_position;
    chain_before(1,end).joint_position = chain_before(1,end).end_position - e2...
         *chain_before(1,end).l2;
    chain_before(1,end).origin_position = chain_before(1,end).joint_position - e2...
        *chain_before(1,end).l1;
    chain_before(1,end) = chain_before(1,end).updateUnit;

    chain_after = chain_before;

end

function chain_after = setBackward(chain_before)
    t1 = chain_before(1,end-1).end_position;
    p = t1;
    alpha = 120 / 180 * pi;
    lc = chain_before(2,1).l1 + chain_before(2,1).l2;
    e1 = chain_before(1,end-1).e2;
    beta = alpha - pi;
    ee2 = [cos(beta)*e1(1)-sin(beta)*e1(2) sin(beta)*e1(1)+cos(beta)*e1(2) 0]';
    beta = pi - alpha;
    ee3 = [cos(beta)*e1(1)-sin(beta)*e1(2) sin(beta)*e1(1)+cos(beta)*e1(2) 0]';
    chain_before(1,end).origin_position = p;
    chain_before(1,end).joint_position = p + chain_before(1,end).l1...
         *chain_before(1,end-1).e2;
    chain_before(1,end).end_position = chain_before(1,end).joint_position + chain_before(1,end).l2...
         *chain_before(1,end-1).e2;
    chain_before(1,end) = chain_before(1,end).updateUnit;

    chain_before(2,1).origin_position = chain_before(1,end).end_position;
    chain_before(2,1).end_position = chain_before(2,1).origin_position + ee2 * lc;
    chain_before(2,1).joint_position = (chain_before(2,1).origin_position + chain_before(2,1).end_position) / 2;
    chain_before(2,1) = chain_before(2,1).updateUnit;

    chain_before(3,1).origin_position = chain_before(1,end).end_position;
    chain_before(3,1).end_position = chain_before(3,1).origin_position + ee3 * lc;
    chain_before(3,1).joint_position = (chain_before(3,1).origin_position + chain_before(3,1).end_position) / 2;
    chain_before(3,1) = chain_before(3,1).updateUnit;
    chain_after = chain_before;

end