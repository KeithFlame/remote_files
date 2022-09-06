function plot_delta_planar(q_1,xita_1,delta_1,q_2,xita_2,delta_2)
%画delta机器人_平面版
%% 结构参数
global alpha L_continuum L_rigid L_moving_platform;
% 导轨单位向量
origin_point = [0;0];
linear_guide_1 = [-cos(alpha);sin(alpha)];  
linear_guide_2 = [cos(alpha);sin(alpha)];
line_1 = [origin_point 300*linear_guide_1];
line_2 = [origin_point 300*linear_guide_2];

% 画导轨1和导轨2
plot(line_1(1,:),line_1(2,:),'.-','linewidth',3,'color','red');
hold on;
plot(line_2(1,:),line_1(2,:),'.-','linewidth',3,'color','green');

%% 画连续体1
P_1_proximal_base = q_1 * linear_guide_1;
if xita_1 == 0
    continuum_1_proximal = [P_1_proximal_base P_1_proximal_base-[0;L_continuum]];
    rigid_1 = [P_1_proximal_base-[0;L_continuum] P_1_proximal_base-[0;L_continuum+L_rigid]];
    continuum_1_distal = [P_1_proximal_base-[0;L_continuum+L_rigid] P_1_proximal_base-[0;2*L_continuum+L_rigid]];
    plot(continuum_1_proximal(1,:),continuum_1_proximal(2,:),'linewidth',3,'color','red');
    plot(continuum_1_distal(1,:),continuum_1_distal(2,:),'linewidth',3,'color','red');
    plot(rigid_1(1,:),rigid_1(2,:),'linewidth',3,'color','black');
else
    R_1 = L_continuum/xita_1;
    continuum_1_proximal = zeros(2,11);
    continuum_1_proximal(:,1) = P_1_proximal_base;
    continuum_1_distal = zeros(2,11);
    if delta_1 == 1
        for i = 1:10
            xita = i/10*xita_1;
            temp = R_1*[1-cos(xita);-sin(xita)];
            continuum_1_proximal(:,i+1) = continuum_1_proximal(:,1) + temp;     
        end
        rigid_1 = [continuum_1_proximal(:,end) continuum_1_proximal(:,end)+L_rigid*[sin(xita_1);-cos(xita_1)]];      
        continuum_1_distal(:,1) = rigid_1(:,2);
        for i = 1:10
            xita = (10-i)/10*xita_1;
            temp = R_1*[cos(xita)-cos(xita_1);-sin(xita_1)+sin(xita)];
            continuum_1_distal(:,i+1) = rigid_1(:,2) + temp;     
        end
    else
        for i = 1:10
            xita = i/10*xita_1;
            temp = R_1*[-1+cos(xita);-sin(xita)];
            continuum_1_proximal(:,i+1) = continuum_1_proximal(:,1) + temp;     
        end
        rigid_1 = [continuum_1_proximal(:,end) continuum_1_proximal(:,end)+L_rigid*[-sin(xita_1);-cos(xita_1)]];
        continuum_1_distal(:,1) = rigid_1(:,2);
        for i = 1:10
            xita = (10-i)/10*xita_1;
            temp = R_1*[-cos(xita)+cos(xita_1);-sin(xita_1)+sin(xita)];
            continuum_1_distal(:,i+1) = rigid_1(:,2) + temp;     
        end
    end
    plot(continuum_1_proximal(1,:),continuum_1_proximal(2,:),'linewidth',3,'color','red');
    plot(continuum_1_distal(1,:),continuum_1_distal(2,:),'linewidth',3,'color','red');
    plot(rigid_1(1,:),rigid_1(2,:),'linewidth',3,'color','black');
end

%% 画连续体2
P_2_proximal_base = q_2 * linear_guide_2;
if xita_2 == 0
    continuum_2_proximal = [P_2_proximal_base P_2_proximal_base-[0;L_continuum]];
    rigid_2 = [P_2_proximal_base-[0;L_continuum] P_2_proximal_base-[0;L_continuum+L_rigid]];
    continuum_2_distal = [P_2_proximal_base-[0;L_continuum+L_rigid] P_2_proximal_base-[0;2*L_continuum+L_rigid]];
    plot(continuum_2_proximal(1,:),continuum_2_proximal(2,:),'linewidth',3,'color','green');
    plot(continuum_2_distal(1,:),continuum_2_distal(2,:),'linewidth',3,'color','green');
    plot(rigid_2(1,:),rigid_2(2,:),'linewidth',3,'color','black');
else
    R_2 = L_continuum/xita_2;
    continuum_2_proximal = zeros(2,11);
    continuum_2_proximal(:,1) = P_2_proximal_base;
    continuum_2_distal = zeros(2,11);
    if delta_2 == 1
        for i = 1:10
            xita = i/10*xita_2;
            temp = R_2*[1-cos(xita);-sin(xita)];
            continuum_2_proximal(:,i+1) = continuum_2_proximal(:,1) + temp;     
        end
        rigid_2 = [continuum_2_proximal(:,end) continuum_2_proximal(:,end)+L_rigid*[sin(xita_2);-cos(xita_2)]];      
        continuum_2_distal(:,1) = rigid_2(:,2);
        for i = 1:10
            xita = (10-i)/10*xita_2;
            temp = R_2*[cos(xita)-cos(xita_2);-sin(xita_2)+sin(xita)];
            continuum_2_distal(:,i+1) = rigid_2(:,2) + temp;     
        end
    else
        for i = 1:10
            xita = i/10*xita_2;
            temp = R_2*[-1+cos(xita);-sin(xita)];
            continuum_2_proximal(:,i+1) = continuum_2_proximal(:,1) + temp;     
        end
        rigid_2 = [continuum_2_proximal(:,end) continuum_2_proximal(:,end)+L_rigid*[-sin(xita_2);-cos(xita_2)]];
        continuum_2_distal(:,1) = rigid_2(:,2);
        for i = 1:10
            xita = (10-i)/10*xita_2;
            temp = R_2*[-cos(xita)+cos(xita_2);-sin(xita_2)+sin(xita)];
            continuum_2_distal(:,i+1) = rigid_2(:,2) + temp;     
        end
    end
    plot(continuum_2_proximal(1,:),continuum_2_proximal(2,:),'linewidth',3,'color','green');
    plot(continuum_2_distal(1,:),continuum_2_distal(2,:),'linewidth',3,'color','green');
    plot(rigid_2(1,:),rigid_2(2,:),'linewidth',3,'color','black');
end
%% 画动平台
moving_platform = [continuum_1_distal(:,end) continuum_2_distal(:,end)];
plot(moving_platform(1,:),moving_platform(2,:),'linewidth',3,'color','blue');
grid on;
box on;
axis equal;
hold off;

end

