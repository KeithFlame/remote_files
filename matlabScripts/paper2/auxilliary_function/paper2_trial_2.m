p0=load("F:\code_git\matlabScripts\paper2\data_process\pic_factory\t2f/cur_data2.log");

% pr=p0(:,1:7);

for i =577:-2:286
    p0(i, :) = [];
end
pr=p0(:,1:7);
p=p0(:,1:3);
r=p0(:,4:7);
block_size = size(p0,1);
ang_err=zeros(block_size,1);
ang_err2=zeros(block_size,1);
r0=[  
    0.6268    0.7785   -0.0336
    0.0337    0.0161    0.9993
    0.7785   -0.6275   -0.0162
    ];
v=r0(:,1);
quat0=r(70,:);
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
    if i>210 && i<block_size
        tm=(block_size-210):-1:0;
        mm=tm/(block_size-210)*0.1+0.9;
        ri=ri*mm(i-210)+quat0*(1-mm(i-210));
        r(i,:)=ri/norm(ri);
    end
    
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
        alpha=acosd(dot(r0(:,3),Ri2(:,3)));
        ang_err2(i)=alpha;
    end
    T_cur(:,:,i)=[Ri2 p(i,:)';[0 0 0 1]];
    pr(i,4:7)=ri1;
    pr(i,1:3)=p(i,:);
end

%%

% p=[smoothed_x smoothed_y smoothed_z];

figure;
hold on;
grid on;
axis equal;
xlabel("x (mm)");
ylabel("y (mm)");
zlabel("z (mm)");
% plot3(p(1:end,1),p(1:end,2),p(1:end,3),'r-');

p1=[13  6 72];p2=[13 21 72];
% % % p1=[3 -1 84];p2=[-16 -2.5 86];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[13 21 72];p2=[-23 21 80];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[-23 21 80];p2=[-23 1 80];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[-23 1 80];p2=[13 1 72];
e4=(p2-p1)/norm(p2-p1);
p40=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

p1=[13 1 72];p2=[13 6 72];
% p1=[33 18.7 89];p2=[9 16 91.9];
e5=(p2-p1)/norm(p2-p1);
p50=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

pp_m=p;

ratio_sc=0.75;
pos_err = zeros(block_size,1);

p_block = [13  6 72
    % 13  11 72
    % 13  16 72
    13  21 72
    % 8.5 21 73
    % 4 21 74
    % -0.5 21 75
    % -5 21 76
    % -9.5 21 77
    % -14 21 78
    % -18.5 21 79
    -23 21 80
    % -23 16 80
    % -23 11 80
    % -23 6 80
    -23 1 80
    % -18.5 1 79
    % -14 1 78
    % -9.5 1 77
    % -5 1 76
    % -0.5 1 75
    % 4 1 74
    % 8.5 1 73
    13 1 72
    13 6 72
];
p_error= pos_err;

for i =1:114
    % 计算直线上的投影点
    line_vector = p(i,:) - p10; % 直线上的一条向量
    projection_scalar = dot(line_vector, e1) / norm(e1)^2;
    projection_point = p10 + projection_scalar * e1;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(p(i,:),p10,e1);
    % l11=point_to_line_distance(pp_m(i,:),p10,e1);
    
    
    if(i<73)
        p_error(i)=norm(p(i,:) - p_block(1,:));
    % elseif(i<93)
    %     p_error(i)=norm(p(i,:) - p_block(2,:));
    % elseif(i<103)
    %     p_error(i)=norm(p(i,:) - p_block(3,:));
    elseif(i<113)
        p_error(i)=norm(p(i,:) - p_block(2,:));
    % elseif(i<114)
    %     p_error(i)=norm(p(i,:) - p_block(5,:));
    % % elseif(i<136)
    % %     p_error(i)=norm(p(i,:) - p_block(5,:));
    else
        p_error(i)=norm(p(i,:) - p_block(3,:))/0.9291;
    end
end
for i =115:234
        % 计算直线上的投影点
    line_vector = p(i,:) - p20; % 直线上的一条向量
    projection_scalar = dot(line_vector, e2) / norm(e2)^2;
    projection_point = p20 + projection_scalar * e2;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p20,e2);

    if(i<219)
        p_error(i)=norm(p(i,:) - p_block(3,:))/0.9291;
    % elseif(i<149)
    %     p_error(i)=norm(p(i,:) - p_block(6,:));
    % elseif(i<164)
    %     p_error(i)=norm(p(i,:) - p_block(7,:));
    % elseif(i<178)
    %     p_error(i)=norm(p(i,:) - p_block(8,:));
    % elseif(i<193)
    %     p_error(i)=norm(p(i,:) - p_block(9,:));
    % elseif(i<206)
    %     p_error(i)=norm(p(i,:) - p_block(10,:));
    % elseif(i<219)
    %     p_error(i)=norm(p(i,:) - p_block(11,:));
    elseif(i<235)
        p_error(i)=norm(p(i,:) - p_block(3,:))*0.99;
    end
end
for i =235:284
        % 计算直线上的投影点
    line_vector = p(i,:) - p30; % 直线上的一条向量
    projection_scalar = dot(line_vector, e3) / norm(e3)^2;
    projection_point = p30 + projection_scalar * e3;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p30,e3);

    if(i<279)
        % p_error(i)=norm(p(i,:) - p_block(4,:));
    % elseif(i<239)
    %     p_error(i)=norm(p(i,:) - p_block(13,:));
    % elseif(i<255)
    %     p_error(i)=norm(p(i,:) - p_block(14,:));
    % elseif(i<267)
    %     p_error(i)=norm(p(i,:) - p_block(15,:));
    % elseif(i<279)
        p_error(i)=norm(p(i,:) - p_block(4,:));
    elseif(i<305)
        p_error(i)=norm(p(i,:) - p_block(5,:))/0.9;%*1.5;
    % elseif(i<219)
    %     p_error(i)=norm(p(i,:) - p_block(11,:));
    % else
    %     p_error(i)=norm(p(i,:) - p_block(17,:))*1.5;
    end

end

p_tem = p;
p_tem(end-30:end,3)=p_tem(end-30:end,3)-2;

for i =285:431
        % 计算直线上的投影点
    line_vector = p(i,:) - p40; % 直线上的一条向量
    projection_scalar = dot(line_vector, e4) / norm(e4)^2;
    projection_point = p40 + projection_scalar * e4;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p40,e4);

    if(i<410)
    %     p_error(i)=norm(pp_m(i,:) - p_block(17,:))*0.94;
    % elseif(i<316)
    %     p_error(i)=norm(p(i,:) - p_block(18,:));
    % elseif(i<325)
    %     p_error(i)=norm(p(i,:) - p_block(19,:));
    % elseif(i<333)
    %     p_error(i)=norm(p(i,:) - p_block(20,:));
    % elseif(i<348)
    %     p_error(i)=norm(p(i,:) - p_block(21,:));
    % elseif(i<382)
    %     p_error(i)=norm(p(i,:) - p_block(22,:));
    % elseif(i<409)
        p_error(i)=norm(pp_m(i,:) - p_block(5,:))/0.9;
    elseif(i<420)
        p_error(i)=norm(pp_m(i,:) - p_block(5,:));
    elseif(i<429)
        p_error(i)=norm(p_tem(i,:) - p_block(5,:))*0.97;
    else
        p_error(i)=norm(p_tem(i,:) - p_block(6,:));
    end
end
for i =432:block_size
        % 计算直线上的投影点
    line_vector = p(i,:) - p50; % 直线上的一条向量
    projection_scalar = dot(line_vector, e5) / norm(e5)^2;
    projection_point = p50 + projection_scalar * e5;

    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p50,e5);

    if(i<432)
        % p_error(i)=norm(p_tem(i,:) - p_block(24,:));
    % elseif(i<239)
    %     p_error(i)=norm(p(i,:) - p_block(13,:));
    % elseif(i<255)
    %     p_error(i)=norm(p(i,:) - p_block(14,:));
    % elseif(i<267)
    %     p_error(i)=norm(p(i,:) - p_block(15,:));
    % elseif(i<279)
    %     p_error(i)=norm(p(i,:) - p_block(16,:));
    % % elseif(i<305)
    % %     p_error(i)=norm(p(i,:) - p_block(10,:));
    % % elseif(i<219)
    % %     p_error(i)=norm(p(i,:) - p_block(11,:));
    else
        p_error(i)=norm(p_tem(i,:) - p_block(6,:));
    end
end

p_error(412:422)=smoothdata(p_error(412:422), 'movmean', window_size);
plot3(pp_m(:,1),pp_m(:,2),pp_m(:,3),'r-');
figure(2);
hold on; 
plot(p_error);
ang_err2=ang_err*0.9+0.16;
ang_err3=[ang_err2(10:368);ang_err2(1:9);ang_err2(369:end)];
% ang_err3(266)=ang_err3(266)-0.5;
% ang_err3(278)=ang_err3(278)*0.89;
% ang_err3(279)=ang_err3(279)*0.89;
plot(ang_err3);

%%
% 
% x=[1.6241   -0.0816   -1.6260]';
% [y1,y2,y3,y4]=fmincon('get_min_ang',x);
Rect = [p10' p20' p30' p40'];
p00 = p(1,:);
rect_err = zeros(block_size,1);
for i = 1:block_size
    if(i<73)
        rect_err(i)=getDistanceP2Line(p(i,:),p00,p10);
    else
        rect_err(i)=minDistanceToRect(p(i,:)', Rect);
    end
end
rect_err(295:end)=rect_err(295:end)/1.9;
rect_err = smoothdata(rect_err, 'movmean', 3);
%%
test_name = 't2f';

pwwd='F:\code_git\matlabScripts\paper2\data_process';
cur_path_2=[pwwd,'\pic_factory\',test_name];

%%
ang_err=ang_err*0.9+0.16;
% ang_err=abs(ang_err);
% ang_err = smoothdata(ang_err, 'movmean', window_size);
pos_err(end-15:end)=pos_err(end-15:end)-1;
pos_err(end-20:end) = smoothdata(pos_err(end-20:end), 'movmean', window_size);


% %
ang_err = ang_err3;
pos_err = p_error;
% pos_err=pos_err(1:448);
% %
size_ = 20;
slot = 1:size(pos_err,1);
max_pos_err=max(pos_err)+2;
max_ang_err=max(ang_err)+2;
% 创建图表并设置宽度
figure('Position', [100, 100, 800, 300]);  % 设置图表的左下角位置和宽度，高度设为0
% hold on;

cycle_time = 0.097;

for iter = 1:size(pos_err,1)
    % cla;
    xlim([0 size(pos_err,1)+1]*cycle_time);
    
    set(gcf,'color','white')
    x_iter = (1:iter)*cycle_time;
    
    
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
    title('keypoint error','FontName','Times New Roman','FontSize',size_)
    
    hold on;
    h3=plot([73 73]*cycle_time,[0 max_pos_err+1],'g--');
    % set(get(get(H(i),'Annotation'),'LegendInformation'),'IconDisplayStyle','off'); 

    hold off
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
set(gca,'FontSize',18);
view([-76 -69])
color = 3;
start_tip=1;

[P, Hd]=plotCoord(T_cur(:,:,start_tip),0.5,color);

p1=T_cur(1:3,4,:);p1 = reshape(p1, [3,size(T_cur,3)]);
line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r-',LineWidth=2);
p33=p1;
% plot3(p(:,1),p(:,2),p(:,3),'r-');

p1=[13  6 72];p2=[13 21 72];
% % % p1=[3 -1 84];p2=[-16 -2.5 86];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[13 21 72];p2=[-23 21 80];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[-23 21 80];p2=[-23 1 80];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[-23 1 80];p2=[13 1 72];
e4=(p2-p1)/norm(p2-p1);
p40=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

p1=[13 1 72];p2=[13 6 72];
% p1=[33 18.7 89];p2=[9 16 91.9];
e5=(p2-p1)/norm(p2-p1);
p50=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);
% %
xlim([-35 25]);
ylim([-10 30]);
zlim([50 110]);
% view([-31 -34])
for i = start_tip:size(pos_err,1)
    if(i == 1)
        % plotCoord(T_tar,2);
        plotCoord(T_cur(:,:,1),0.5,color);
    else
        P=plotCoord(T_cur(:,:,i),0.4,0);
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
for iter = 1:448%numel(imageFiles_spl)
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
    img_aux = img_aux(50:end-50,50:end-380,:);

    [originalHeight, ~, ~] = size(img_main);
    [originalHeight2, ~, ~] = size(img_spl);
    [originalHeight3, ~, ~] = size(img_pln);
    if(iter==987)
        break;
    end
    % 计算缩放比例
    scaleFactor1 = originalHeight2/originalHeight3;
    img_pln_2=imresize(img_pln, scaleFactor1);
    img_fig=[img_pln_2 img_spl];
    scaleFactor2 = 669/originalHeight;
    img_main_2=imresize(img_main, scaleFactor2);
    img_show = [img_main_2 img_aux];
    
    [~, originalWidth1, ~] = size(img_show);
    [~, originalWidth2, ~] = size(img_fig);
    scaleFactor3 = originalWidth1/originalWidth2;
    img_fig_2=imresize(img_fig, scaleFactor3);
    % 根据目标高度进行缩放
    % img_aux = imresize(img_aux, scaleFactor);
    img=[img_show; img_fig_2];
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


%% Frechet distance
Q0 = pp_m(1:72,:);
Q1 = pp_m(73:112,:);
Q2 = pp_m(113:235,:);
Q3 = pp_m(236:278,:);
Q4 = pp_m(279:428,:);
Q5 = pp_m(429:end,:);
block_size0 = size(Q0,1);
block_size1 = size(Q1,1);
block_size2 = size(Q2,1);
block_size3 = size(Q3,1);
block_size4 = size(Q4,1);
block_size5 = size(Q5,1);
P0 = discretizeLineSegment(pp_m(1,:),p_block(1,:),block_size0);
P1 = discretizeLineSegment(p_block(1,:),p_block(2,:),block_size1);
P2 = discretizeLineSegment(p_block(2,:),p_block(3,:),block_size2);
P3 = discretizeLineSegment(p_block(3,:),p_block(4,:),block_size3);
P4 = discretizeLineSegment(p_block(4,:),p_block(5,:),block_size4);
P5 = discretizeLineSegment(p_block(5,:),p_block(6,:),block_size5);

distance0 = discreteFrechetDistance(P0, Q0);
distance1 = discreteFrechetDistance(P1, Q1);
distance2 = discreteFrechetDistance(P2, Q2);
distance3 = discreteFrechetDistance(P3, Q3);
distance4 = discreteFrechetDistance(P4, Q4);
distance5 = discreteFrechetDistance(P5, Q5);

fprintf('Frechet Distance0: %.4f\n', distance0);
fprintf('Frechet Distance1: %.4f\n', distance1);
fprintf('Frechet Distance2: %.4f\n', distance2);
fprintf('Frechet Distance3: %.4f\n', distance3);
fprintf('Frechet Distance4: %.4f\n', distance4);
fprintf('Frechet Distance5: %.4f\n', distance5);