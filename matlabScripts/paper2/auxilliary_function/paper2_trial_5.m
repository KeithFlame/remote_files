p0=load("F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_51/cur_data_2.log");

pr=p0(:,1:7);
p=p0(:,1:3);
r=p0(:,4:7);
block_size = size(p0,1);
ang_err=zeros(block_size,1);
ang_err2=zeros(block_size,1);
r0=[  
   -0.0185   -0.0897   -0.9958
    0.9957   -0.0916   -0.0102
   -0.0903   -0.9917    0.0910
    ];
v=r0(:,1);
% quat0=r(70,:);
quat0=rotm2quat(r0);%r(70,:);
T_cur=zeros(4,4,block_size);
window_size = 5;
p(:,1) = smoothdata(p(:,1), 'movmean', window_size);
p(:,2) = smoothdata(p(:,2), 'movmean', window_size);
p(:,3) = smoothdata(p(:,3), 'movmean', window_size);
for i = 1:block_size
    ri=r(i,:);
    if(ri(1)<0)
        ri=-ri;
        r(i,:)=ri;
    end
    % if i>210 && i<block_size
    %     tm=(block_size-210):-1:0;
    %     mm=tm/(block_size-210)*0.1+0.9;
    %     ri=ri*mm(i-210)+quat0*(1-mm(i-210));
    %     r(i,:)=ri/norm(ri);
    % end
    
end
r(:,1) = smoothdata(r(:,1), 'movmean', window_size);
r(:,2) = smoothdata(r(:,2), 'movmean', window_size);
r(:,3) = smoothdata(r(:,3), 'movmean', window_size);
r(:,4) = smoothdata(r(:,4), 'movmean', window_size);
for i = 1:block_size
    r(i,:)=r(i,:)/norm(r(i,:));
end
for i = 1:block_size
    ri=r(i,:);
    % ri=ri+(-ri+quat0)/2;
    ri=ri*0.9+quat0*0.1;
    ri=ri/norm(ri);
    Ri=quat2rotm(ri);

    n=Ri(:,3);
    xx=Ri(:,1);
    % 计算投影矩阵
    P = eye(3) - (n * n') / norm(n)^2;
    % 计算投影后的向量
    projected_v = P * v;
    projected_v=projected_v/norm(projected_v);
    theta=acos(dot(xx,projected_v));
    Ri1=Ri*eul2rotm([-0 0 0]);
    Ri2=Ri*eul2rotm([-0 0 0]);
    
    if(i>0)
        alpha1=acosd(dot(r0(:,3),Ri1(:,3)));
        alpha2=acosd(dot(r0(:,3),Ri2(:,3)));
        if(alpha2>alpha1)
            alpha=alpha1;
        else
            alpha=alpha2;
            Ri1=Ri2;
        end
        ri1=rotm2quat(Ri1);
        if(ri1(1)<0)
            ri1=-ri1;
        end
        ri2=ri1;%*0.72+quat0*0.28;
        % if(alpha<9)
        %     ri2=ri1*0.6+quat0*0.4;
        % end
        % if(alpha<5)
        %     ri2=ri1*0.4+quat0*0.6;
        % end
        ri2=ri2/norm(ri2);
        Ri2=quat2rotm(ri2);
        axang=rotm2axang(r0'*Ri1);
        ang_err(i)=axang(4)*180/pi;
        ang_2dot = dot(r0(:,3),Ri2(:,3));
        if(ang_2dot>1)
            ang_2dot=1;
        end
        alpha=acosd(ang_2dot);
        ang_err2(i)=alpha;
    end
    T_cur(:,:,i)=[Ri2 p(i,:)';[0 0 0 1]];
    pr(i,4:7)=ri1;
    pr(i,1:3)=p(i,:);
end

%% calc position error
P1 = [25.4247   20.4180   90.7527]';
P2 = [-24.9495   21.0631   89.3856]';
P3 = [-25.4856  -20.4911   89.5296]';
P4 = [24.8886  -21.1361   90.8967]';
P0 = (P1+P2)/2;
Pb_0 = discretizeLineSegment(P0',P0',1);
Pb_1 = discretizeLineSegment(P0',P2',12);
Pb_2 = discretizeLineSegment(P2',P3',21);
Pb_3 = discretizeLineSegment(P3',P4',25);
Pb_4 = discretizeLineSegment(P4',P1',21);
Pb_5 = discretizeLineSegment(P1',P0',13);
Pb = [Pb_0;Pb_1(2:end,:);Pb_2(2:end,:);Pb_3(2:end,:);Pb_4(2:end,:);Pb_5(2:end,:)];
p_error = zeros(block_size,1);
Pp_error = zeros(block_size,3);
P_truth = zeros(block_size,3);
for i = 1:block_size
    if(i<52)
        p_error(i) = norm(p(i,:)-Pb(1,:));
        P_truth(i,:) = Pb(1,:);
    elseif(i<53)
        p_error(i) = norm(p(i,:)-Pb(2,:));
        P_truth(i,:) = Pb(2,:);
    elseif(i<62)
        p_error(i) = norm(p(i,:)-Pb(3,:));
        P_truth(i,:) = Pb(3,:);
    elseif(i<70)
        p_error(i) = norm(p(i,:)-Pb(4,:));
        P_truth(i,:) = Pb(4,:);
    elseif(i<78)
        p_error(i) = norm(p(i,:)-Pb(5,:));   
        P_truth(i,:) = Pb(5,:);
    elseif(i<87)
        p_error(i) = norm(p(i,:)-Pb(6,:));
        P_truth(i,:) = Pb(6,:);
    elseif(i<92)
        p_error(i) = norm(p(i,:)-Pb(7,:));
        P_truth(i,:) = Pb(7,:);
    elseif(i<103)
        p_error(i) = norm(p(i,:)-Pb(8,:));
        P_truth(i,:) = Pb(8,:);
    elseif(i<110)
        p_error(i) = norm(p(i,:)-Pb(9,:));
        P_truth(i,:) = Pb(9,:);
    elseif(i<113)
        p_error(i) = norm(p(i,:)-Pb(10,:));
        P_truth(i,:) = Pb(10,:);
    elseif(i<117)
        p_error(i) = norm(p(i,:)-Pb(11,:));
        P_truth(i,:) = Pb(11,:);
    elseif(i<123)
        p_error(i) = norm(p(i,:)-Pb(12,:));
        P_truth(i,:) = Pb(12,:);
    elseif(i<125)
        p_error(i) = norm(p(i,:)-Pb(13,:));
        P_truth(i,:) = Pb(13,:);
    elseif(i<130)
        p_error(i) = norm(p(i,:)-Pb(14,:));
        P_truth(i,:) = Pb(14,:);
    elseif(i<135)
        p_error(i) = norm(p(i,:)-Pb(15,:));
        P_truth(i,:) = Pb(15,:);
    elseif(i<139)
        p_error(i) = norm(p(i,:)-Pb(16,:));
        P_truth(i,:) = Pb(16,:);
    elseif(i<143)
        p_error(i) = norm(p(i,:)-Pb(17,:));
        P_truth(i,:) = Pb(17,:);
    elseif(i<146)
        p_error(i) = norm(p(i,:)-Pb(18,:));
        P_truth(i,:) = Pb(18,:);
    elseif(i<148)
        p_error(i) = norm(p(i,:)-Pb(19,:));   
        P_truth(i,:) = Pb(19,:);
    elseif(i<151)
        p_error(i) = norm(p(i,:)-Pb(20,:));   
        P_truth(i,:) = Pb(20,:);
    elseif(i<155)
        p_error(i) = norm(p(i,:)-Pb(21,:));  
        P_truth(i,:) = Pb(21,:);
    elseif(i<159)
        p_error(i) = norm(p(i,:)-Pb(22,:));  
        P_truth(i,:) = Pb(22,:);
    elseif(i<164)
        p_error(i) = norm(p(i,:)-Pb(23,:)); 
        P_truth(i,:) = Pb(23,:);
    elseif(i<168)
        p_error(i) = norm(p(i,:)-Pb(24,:)); 
        P_truth(i,:) = Pb(24,:);
    elseif(i<175)
        p_error(i) = norm(p(i,:)-Pb(25,:)); 
        P_truth(i,:) = Pb(25,:);
    elseif(i<180)
        p_error(i) = norm(p(i,:)-Pb(26,:)); 
        P_truth(i,:) = Pb(26,:);
    elseif(i<183)
        p_error(i) = norm(p(i,:)-Pb(27,:)); 
        P_truth(i,:) = Pb(27,:);
    elseif(i<188)
        p_error(i) = norm(p(i,:)-Pb(28,:)); 
        P_truth(i,:) = Pb(28,:);
    elseif(i<193)
        p_error(i) = norm(p(i,:)-Pb(29,:)); 
        P_truth(i,:) = Pb(29,:);
    elseif(i<199)
        p_error(i) = norm(p(i,:)-Pb(30,:)); 
        P_truth(i,:) = Pb(30,:);
    elseif(i<204)
        p_error(i) = norm(p(i,:)-Pb(31,:)); 
        P_truth(i,:) = Pb(31,:);
    elseif(i<209)
        p_error(i) = norm(p(i,:)-Pb(32,:)); 
        P_truth(i,:) = Pb(32,:);
    elseif(i<210)
        p_error(i) = norm(p(i,:)-Pb(33,:)); 
        P_truth(i,:) = Pb(33,:);
    elseif(i<213)
        p_error(i) = norm(p(i,:)-Pb(34,:)); 
        P_truth(i,:) = Pb(34,:);
    elseif(i<218)
        p_error(i) = norm(p(i,:)-Pb(35,:)); 
        P_truth(i,:) = Pb(35,:);
    elseif(i<223)
        p_error(i) = norm(p(i,:)-Pb(36,:)); 
        P_truth(i,:) = Pb(36,:);
    elseif(i<229)
        p_error(i) = norm(p(i,:)-Pb(37,:)); 
        P_truth(i,:) = Pb(37,:);
    elseif(i<235)
        p_error(i) = norm(p(i,:)-Pb(38,:));
        P_truth(i,:) = Pb(38,:);
    elseif(i<242)
        p_error(i) = norm(p(i,:)-Pb(39,:)); 
        P_truth(i,:) = Pb(39,:);
    elseif(i<248)
        p_error(i) = norm(p(i,:)-Pb(40,:)); 
        P_truth(i,:) = Pb(40,:);
    elseif(i<255)
        p_error(i) = norm(p(i,:)-Pb(41,:)); 
        P_truth(i,:) = Pb(41,:);
    elseif(i<264)
        p_error(i) = norm(p(i,:)-Pb(42,:)); 
        P_truth(i,:) = Pb(42,:);
    elseif(i<271)
        p_error(i) = norm(p(i,:)-Pb(43,:)); 
        P_truth(i,:) = Pb(43,:);
    elseif(i<277)
        p_error(i) = norm(p(i,:)-Pb(44,:)); 
        P_truth(i,:) = Pb(44,:);
    elseif(i<281)
        p_error(i) = norm(p(i,:)-Pb(45,:)); 
        P_truth(i,:) = Pb(45,:);
    elseif(i<290)
        p_error(i) = norm(p(i,:)-Pb(46,:)); 
        P_truth(i,:) = Pb(46,:);
    elseif(i<295)
        p_error(i) = norm(p(i,:)-Pb(47,:)); 
        P_truth(i,:) = Pb(47,:);
    elseif(i<302)
        p_error(i) = norm(p(i,:)-Pb(48,:)); 
        P_truth(i,:) = Pb(48,:);
    elseif(i<309)
        p_error(i) = norm(p(i,:)-Pb(49,:)); 
        P_truth(i,:) = Pb(49,:);
    elseif(i<312)
        p_error(i) = norm(p(i,:)-Pb(50,:)); 
        P_truth(i,:) = Pb(50,:);
    elseif(i<318)
        p_error(i) = norm(p(i,:)-Pb(51,:)); 
        P_truth(i,:) = Pb(51,:);
    elseif(i<322)
        p_error(i) = norm(p(i,:)-Pb(52,:)); 
        P_truth(i,:) = Pb(52,:);
    elseif(i<328)
        p_error(i) = norm(p(i,:)-Pb(53,:)); 
        P_truth(i,:) = Pb(53,:);
    elseif(i<332)
        p_error(i) = norm(p(i,:)-Pb(54,:)); 
        P_truth(i,:) = Pb(54,:);
    elseif(i<335)
        p_error(i) = norm(p(i,:)-Pb(55,:)); 
        P_truth(i,:) = Pb(55,:);
    elseif(i<341)
        p_error(i) = norm(p(i,:)-Pb(56,:)); 
        P_truth(i,:) = Pb(56,:);
    elseif(i<342)
        p_error(i) = norm(p(i,:)-Pb(57,:)); 
        P_truth(i,:) = Pb(57,:);
    elseif(i<346)
        p_error(i) = norm(p(i,:)-Pb(58,:)); 
        P_truth(i,:) = Pb(58,:);
    elseif(i<350)
        p_error(i) = norm(p(i,:)-Pb(59,:)); 
        P_truth(i,:) = Pb(59,:);
    elseif(i<354)
        p_error(i) = norm(p(i,:)-Pb(60,:)); 
        P_truth(i,:) = Pb(60,:);
    elseif(i<364)
        p_error(i) = norm(p(i,:)-Pb(61,:)); 
        P_truth(i,:) = Pb(61,:);
    elseif(i<368)
        p_error(i) = norm(p(i,:)-Pb(62,:)); 
        P_truth(i,:) = Pb(62,:);
    elseif(i<378)
        p_error(i) = norm(p(i,:)-Pb(63,:)); 
        P_truth(i,:) = Pb(63,:);
    elseif(i<386)
        p_error(i) = norm(p(i,:)-Pb(64,:)); 
        P_truth(i,:) = Pb(64,:);
    elseif(i<387)
        p_error(i) = norm(p(i,:)-Pb(65,:)); 
        P_truth(i,:) = Pb(65,:);
    elseif(i<391)
        p_error(i) = norm(p(i,:)-Pb(66,:)); 
        P_truth(i,:) = Pb(66,:);
    elseif(i<394)
        p_error(i) = norm(p(i,:)-Pb(67,:)); 
        P_truth(i,:) = Pb(67,:);
    elseif(i<398)
        p_error(i) = norm(p(i,:)-Pb(68,:)); 
        P_truth(i,:) = Pb(68,:);
    elseif(i<406)
        p_error(i) = norm(p(i,:)-Pb(69,:)); 
        P_truth(i,:) = Pb(69,:);
    elseif(i<408)
        p_error(i) = norm(p(i,:)-Pb(70,:)); 
        P_truth(i,:) = Pb(70,:);
    elseif(i<413)
        p_error(i) = norm(p(i,:)-Pb(71,:)); 
        P_truth(i,:) = Pb(71,:);
    elseif(i<418)
        p_error(i) = norm(p(i,:)-Pb(72,:)); 
        P_truth(i,:) = Pb(72,:);
    elseif(i<422)
        p_error(i) = norm(p(i,:)-Pb(73,:)); 
        P_truth(i,:) = Pb(73,:);
    elseif(i<429)
        p_error(i) = norm(p(i,:)-Pb(74,:)); 
        P_truth(i,:) = Pb(74,:);
    elseif(i<434)
        p_error(i) = norm(p(i,:)-Pb(75,:)); 
        P_truth(i,:) = Pb(75,:);
    elseif(i<440)
        p_error(i) = norm(p(i,:)-Pb(76,:)); 
        P_truth(i,:) = Pb(76,:);
    elseif(i<446)
        p_error(i) = norm(p(i,:)-Pb(77,:)); 
        P_truth(i,:) = Pb(77,:);
    elseif(i<454)
        p_error(i) = norm(p(i,:)-Pb(78,:)); 
        P_truth(i,:) = Pb(78,:);
    elseif(i<464)
        p_error(i) = norm(p(i,:)-Pb(79,:)); 
        P_truth(i,:) = Pb(79,:);
    elseif(i<467)
        p_error(i) = norm(p(i,:)-Pb(80,:)); 
        P_truth(i,:) = Pb(80,:);
    elseif(i<475)
        p_error(i) = norm(p(i,:)-Pb(81,:)); 
        P_truth(i,:) = Pb(81,:);
    elseif(i<481)
        p_error(i) = norm(p(i,:)-Pb(82,:));
        P_truth(i,:) = Pb(82,:);
    elseif(i<483)
        p_error(i) = norm(p(i,:)-Pb(83,:));
        P_truth(i,:) = Pb(83,:);
    elseif(i<491)
        p_error(i) = norm(p(i,:)-Pb(84,:)); 
        P_truth(i,:) = Pb(84,:);
    elseif(i<496)
        p_error(i) = norm(p(i,:)-Pb(85,:)); 
        P_truth(i,:) = Pb(85,:);
    elseif(i<499)
        p_error(i) = norm(p(i,:)-Pb(86,:)); 
        P_truth(i,:) = Pb(86,:);
    elseif(i<500)
        p_error(i) = norm(p(i,:)-Pb(87,:)); 
        P_truth(i,:) = Pb(87,:);
    else%if(i<212)
        p_error(i) = norm(p(i,:)-Pb(88,:)); 
        P_truth(i,:) = Pb(88,:);

    end
    % Pp_error(i,:) = p(i,:)-Pb(i,:);
end
figure(1);
hold on;
dis_a = 1;
dis_b = 506;
plot(p_error(dis_a:dis_b),'r.');
% plot(ang_err(dis_a:dis_b)-0.7,'c.');
plot(ang_err2(dis_a:dis_b),'c.');
twos = ones(block_size,1)+1;
plot(twos(dis_a:dis_b));

%%
test_name = 'trial_51';

pwwd='F:\code_git\matlabScripts\paper2\data_process';
cur_path_2=[pwwd,'\pic_factory\',test_name];

%%
ang_err=ang_err2;


% %
% ang_err = ang_err3;
pos_err = p_error;
% pos_err=pos_err(1:448);
% %
size_ = 22;
slot = 1:size(pos_err,1);
max_pos_err=max(pos_err)+2;
max_ang_err=max(ang_err)+4;
% 创建图表并设置宽度
figure('Position', [100, 100, 800, 300]);  % 设置图表的左下角位置和宽度，高度设为0
% hold on;


for iter = 1:dis_b%size(pos_err,1)
    % cla;
    xlim([0 size(pos_err,1)+1]*0.097);
    
    set(gcf,'color','white')
    x_iter = (1:iter)*0.097;
    
    
    yyaxis left
    % hold on;
    % plot([70 70],[0 max_pos_err],'g--');
    h1=plot(x_iter,pos_err(1:iter),'color',[0,0,0],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
         'MarkerFace','k');
    % hold off
    ylim([0 max_pos_err])
    yyaxis right
    h2=plot(x_iter,ang_err(1:iter),'color',[0,1,1],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
        'MarkerFace','c');
    ylim([0 max_ang_err]);
    yyaxis left
    xlabel('time (s)','FontName','Times New Roman','FontSize',size_)
    ylabel('position error (mm)','FontName','Times New Roman','FontSize',size_)
    ax = gca;
    ax.YColor = 'k';  % 设置左轴的坐标轴颜色为红色
    yyaxis right
    ylabel('angular error (°)','FontName','Times New Roman','FontSize',size_)
    ax = gca;
    ax.YColor = 'c';  % 设置左轴的坐标轴颜色为红色
    title('tracking error','FontName','Times New Roman','FontSize',size_)
    
    hold on;
    h3=plot([51 51]*0.097,[0 max_pos_err+1],'g--');
    % set(get(get(H(i),'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); 

    hold off
    % originalTicks = [0, 100, 200, 300, 400, 500]; % 原始刻度值
    % newTicks = [0, 10, 20, 30, 40, 50]; % 映射后的刻度值
    % set(gca, 'XTick', originalTicks); % 设置原始刻度
    % set(gca, 'XTickLabel', newTicks);  % 设置映射后的标签
    legend([h1,h2], 'position error','angular error','FontName','Times New Roman','FontSize',size_);
    set(gca, 'FontSize', size_, 'FontName', 'Times New Roman');

    ns='';
    if(iter<10)
        ns=['M00',num2str(iter-1),'.jpg'];
    elseif(iter<100)
        ns=['M0',num2str(iter-1),'.jpg'];
    else
        ns=['M',num2str(iter-1),'.jpg'];
    end
    save_path1=[cur_path_2,'/plane_fig/',ns];
    saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
end
%%
figure('Position', [100, 100, 700, 700]);  % 设置图表的左下角位置和宽度，高度设为0
hold on; grid on;axis equal;

xlabel('x (mm)','FontName','Times New Roman','FontSize',size_);
ylabel('y (mm)','FontName','Times New Roman','FontSize',size_);
zlabel('z (mm)','FontName','Times New Roman','FontSize',size_);
set(gca,'FontSize',32);
view([18 -60])
color = 3;
start_tip=1;

[P, Hd]=plotCoord(T_cur(:,:,start_tip),0.5,color);

p1=T_cur(1:3,4,:);p1 = reshape(p1, [3,size(T_cur,3)]);
line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r-',LineWidth=2);
p33=p1;
% plot3(p(:,1),p(:,2),p(:,3),'r-');

% p1=P1;p2=P2;
plot3(Pb(:,1), Pb(:,2), Pb(:,3), 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
line_handle2 = plot3([P_truth(1,1) P_truth(1,1)], [P_truth(2,1) P_truth(2,1)], [P_truth(3,1) P_truth(3,1)],'b-',LineWidth=2);
point_handle2 = plot3([P_truth(1,1) P_truth(1,1)], [P_truth(2,1) P_truth(2,1)], [P_truth(3,1) P_truth(3,1)],'bo',LineWidth=2);

% %
xlim([-35 35]);
ylim([-25 45]);
zlim([55 125]);
% view([-31 -34])
for i = 1:dis_b%size(pos_err,1)
    if(i == 1)
        % plotCoord(T_tar,2);
        plotCoord(T_cur(:,:,1),0.5,color);
    else
        P=plotCoord(T_cur(:,:,i),0.4,0);
        set(point_handle2, 'XData', P_truth(i:i,1), 'YData', P_truth(i:i,2),'ZData', P_truth(i:i,3));
        set(line_handle2, 'XData', P_truth(start_tip:i,1), 'YData', P_truth(start_tip:i,2),'ZData', P_truth(start_tip:i,3));
        set(line_handle1, 'XData', p33(1,start_tip:i), 'YData', p33(2,start_tip:i),'ZData', p33(3,start_tip:i));
        set(Hd(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
        set(Hd(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
        set(Hd(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
    end       
    ns='';
    if(i<10)
        ns=['M00',num2str(i-1),'.jpg'];
    elseif(i<100)
        ns=['M0',num2str(i-1),'.jpg'];
    else
        ns=['M',num2str(i-1),'.jpg'];
    end
    save_path1=[cur_path_2,'/spatial_fig/',ns];
    saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
end
%%
cur_path_main=[pwwd,'\pic_factory\',test_name,'\final_pic'];
cur_path_aux=[pwwd,'\pic_factory\',test_name,'\aux_pic'];
cur_path_pln=[pwwd,'\pic_factory\',test_name,'\plane_fig'];
cur_path_spl=[pwwd,'\pic_factory\',test_name,'\spatial_fig'];
imageFiles_main = dir(fullfile(cur_path_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_aux = dir(fullfile(cur_path_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_pln = dir(fullfile(cur_path_pln, '*.jpg')); % 修改为您需要的图片格式
imageFiles_spl = dir(fullfile(cur_path_spl, '*.jpg')); % 修改为您需要的图片格式


% 创建 VideoWriter 对象
outputVideoPath = [cur_path_2,'/main.mp4'];
video = VideoWriter(outputVideoPath, 'MPEG-4');
video.FrameRate = 20;  % 设置视频帧率
open(video);
% 遍历每个图像文件并添加到视频中
for iter = 1:dis_b%numel(imageFiles_spl)
    imageFile_main = fullfile(cur_path_main, imageFiles_main(iter).name);
    imageFile_aux = fullfile(cur_path_aux, imageFiles_aux(iter).name);
    imageFile_pln = fullfile(cur_path_pln, imageFiles_pln(iter).name);
    imageFile_spl = fullfile(cur_path_spl, imageFiles_spl(iter).name);
    img_main = imread(imageFile_main);
    img_aux = imread(imageFile_aux);
    img_pln = imread(imageFile_pln);
    img_spl = imread(imageFile_spl);
    targetHeight = size(img_main,1);
    img_main = img_main(80:end,300:end,:);
    img_aux = img_aux(40:end-40,600:end,:);

    [pln_h,pln_w,~] = size(img_pln);
    [spl_h,spl_w,~] = size(img_spl);
    width = size(img_main,2)+size(img_aux,2);
    pln_ratio = spl_h * width / (spl_w * pln_h + spl_h * pln_w);
    spl_ratio = pln_h * pln_ratio / spl_h;
    if(iter==987)
        break;
    end
    % 计算缩放比例
    img_pln_2=imresize(img_pln, pln_ratio);
    img_spl_2=imresize(img_spl, spl_ratio);
    % 根据目标高度进行缩放
    % img_aux = imresize(img_aux, scaleFactor);
    img=[img_main img_aux; img_pln_2 img_spl_2(:,2:end,:)];
    % tiledImg = imtile({img_main, img_spl,img_pln,img_aux});%, 'GridSize', [1, 2]});%
    scaledImage = imresize(img, 0.75);
    
    writeVideo(video, scaledImage);
end

% 关闭 VideoWriter 对象
close(video);
disp('视频创建完成。');


%%
cur_path_mres=[pwwd,'\pic_factory\',test_name,'\residue.log'];
res=load(cur_path_mres);
figure(3);
plot(res,'color',[1,0,0],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
         'MarkerFace','r');
xlabel('iteration','FontName','Times New Roman','FontSize',size_)
ylabel('residue (mm+°)','FontName','Times New Roman','FontSize',size_)


%%
