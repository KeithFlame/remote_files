p0=load("F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_6_3/cur_data_2.log");

intersect = 392;
p=p0(:,1:3);
r=p0(:,4:7);
block_size = size(p0,1);
window_size = 3;
quat0 = [0.7454   -0.1985   -0.6089    0.1847];
p0 = [5 -10 60];
r0 = quat2rotm(quat0/norm(quat0));
p1 = p(1:intersect,:);p2 = p(intersect+1:end,:);
p1(:,1) = smoothdata(p1(:,1), 'movmean', window_size);
p1(:,2) = smoothdata(p1(:,2), 'movmean', window_size);
p1(:,3) = smoothdata(p1(:,3), 'movmean', window_size);
p2(:,1) = smoothdata(p2(:,1), 'movmean', window_size);
p2(:,2) = smoothdata(p2(:,2), 'movmean', window_size);
p2(:,3) = smoothdata(p2(:,3), 'movmean', window_size);
for i = 1:block_size
    ri=r(i,:);
    if(ri(1)<0)
        ri=-ri;
        r(i,:)=ri;
    end   
end
r1 = r(1:intersect,:);r2 = r(intersect+1:end,:);
r1(:,1) = smoothdata(r1(:,1), 'movmean', window_size);
r1(:,2) = smoothdata(r1(:,2), 'movmean', window_size);
r1(:,3) = smoothdata(r1(:,3), 'movmean', window_size);
r1(:,4) = smoothdata(r1(:,4), 'movmean', window_size);
r2(:,1) = smoothdata(r2(:,1), 'movmean', window_size);
r2(:,2) = smoothdata(r2(:,2), 'movmean', window_size);
r2(:,3) = smoothdata(r2(:,3), 'movmean', window_size);
r2(:,4) = smoothdata(r2(:,4), 'movmean', window_size);
block_size1 = size(r1,1);
block_size2 = size(r2,1);
ang_err1=zeros(block_size1,1);
pos_err1=zeros(block_size1,1);
ang_err2=zeros(block_size2,1);
pos_err2=zeros(block_size2,1);
pr1=zeros(block_size1,1);
pr2=zeros(block_size2,1);
T_cur1=zeros(4,4,block_size1);
T_cur2=zeros(4,4,block_size2);
trajectory_length1 = 0;
trajectory_length2 = 0;

for i = 1:block_size1
    r1(i,:)=r1(i,:)/norm(r1(i,:));
    if(i>1)
        trajectory_length1 = trajectory_length1 + norm(p1(i,:)-p1(i-1,:));
    end
end
for i = 1:block_size2
    r2(i,:)=r2(i,:)/norm(r2(i,:));
    if(i>1)
        trajectory_length2 = trajectory_length2 + norm(p2(i,:)-p2(i-1,:));
    end
end
for i = 1:block_size1
    ri=r1(i,:);
    ri=ri/norm(ri);
    Ri=quat2rotm(ri);
    if(i>0)
        ri1=rotm2quat(Ri);
        if(ri1(1)<0)
            ri1=-ri1;
        end
        ri2=ri1;%*0.72+quat0*0.28;
        ri2=ri2/norm(ri2);
        Ri2=quat2rotm(ri2);
        axang=rotm2axang(r0'*Ri);
        dot_ang_err = dot(r0(:,3),Ri(:,3));
        if(dot_ang_err>1)
            dot_ang_err = 1;
        end
        ang_err1(i)=acosd(dot_ang_err);
        % ang_err1(i)=axang(4)*180/pi;
    end
    T_cur1(:,:,i)=[Ri2 p1(i,:)';[0 0 0 1]];
    pr1(i,4:7)=ri1;
    pr1(i,1:3)=p1(i,:);
    pos_err1(i) = norm(p0-p1(i,:));
end

for i = 1:block_size2
    ri=r2(i,:);
    ri=ri/norm(ri);
    Ri=quat2rotm(ri);
    if(i>0)
        ri1=rotm2quat(Ri);
        if(ri1(1)<0)
            ri1=-ri1;
        end
        ri2=ri1;%*0.72+quat0*0.28;
        ri2=ri2/norm(ri2);
        Ri2=quat2rotm(ri2);
        axang=rotm2axang(r0'*Ri);
        dot_ang_err = dot(r0(:,3),Ri(:,3));
        if(dot_ang_err>1)
            dot_ang_err = 1;
        end
        ang_err2(i)=acosd(dot_ang_err);
        % ang_err2(i)=axang(4)*180/pi;
    end
    T_cur2(:,:,i)=[Ri2 p2(i,:)';[0 0 0 1]];
    pr2(i,4:7)=ri1;
    pr2(i,1:3)=p2(i,:);
    pos_err2(i) = norm(p0-p2(i,:));
end

%% calc position error
ang_err1 = ang_err1-1.4;
pos_err1 = pos_err1-0.2;
ang_err2 = ang_err2-0.5;
pos_err2 = pos_err2-0.1;
figure(1);
hold on;
plot(pos_err1,'r.');
% plot(ang_err(dis_a:dis_b)-0.7,'c.');
plot(ang_err1,'c.');
plot(pos_err2,'r.');
% plot(ang_err(dis_a:dis_b)-0.7,'c.');
plot(ang_err2,'c.');

%%
test_name = 'trial_6_3';

pwwd='F:\code_git\matlabScripts\paper2\data_process';
cur_path_2=[pwwd,'\pic_factory\',test_name];

%%
dis_b = max(block_size2*3,block_size1);
pos_err2 = reshape([pos_err2 pos_err2 pos_err2]', [block_size2*3,1]);
ang_err2 = reshape([ang_err2 ang_err2 ang_err2]', [block_size2*3,1]);
pos_err2 = smoothdata(pos_err2, 'movmean', 3);
ang_err2 = smoothdata(ang_err2, 'movmean', 3);

bs2 = size(pos_err2,1);
size_ = 22;
slot = 1:size(pos_err1,1);
max_pos_err=max(pos_err1)+10;
max_ang_err=max(ang_err1)+4;
% 创建图表并设置宽度
figure('Position', [100, 100, 850, 200]);  % 设置图表的左下角位置和宽度，高度设为0
% hold on;
% bs2 = 1; % block_size2 计数
cycle_time = 0.097/3;
for iter = dis_b:dis_b%size(pos_err,1)
    clf;
    xlim([0 size(pos_err1,1)+1]*cycle_time);
    
    set(gcf,'color','white')
    x_iter = (1:iter)*cycle_time;
    
    
    yyaxis left
    hold on;
    % plot([70 70],[0 max_pos_err],'g--');
    h11=plot(x_iter,pos_err1(1:iter),'color',[0,0,0],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
         'MarkerFace','k','DisplayName', 'position error');

        
    if(iter>bs2)
        h12=plot((1:bs2)*cycle_time,pos_err2,'color',[0,0,0],'LineStyle','-','LineWidth',1,...
             'DisplayName', 'position error');  
    else
        h12=plot(x_iter,pos_err2(1:iter),'color',[0,0,0],'LineStyle','-','LineWidth',1,...
             'DisplayName', 'position error');        
    end
    hold off
    ylim([0 max_pos_err*1.3])
    yyaxis right
    hold on
    h21=plot(x_iter,ang_err1(1:iter),'color',[0,1,1],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
        'MarkerFace','c','DisplayName', 'angular error');
    if(iter>bs2)
        h22=plot((1:bs2)*cycle_time,ang_err2,'color',[0,1,1],'LineStyle','-','LineWidth',1,...
            'DisplayName', 'angular error');  
    else
        h22=plot(x_iter,ang_err2(1:iter),'color',[0,1,1],'LineStyle','-','LineWidth',1,...
            'DisplayName', 'angular error');   
    end
    ylim([0 max_ang_err*1.02]);
    yyaxis left
    xlabel('time (s)','FontName','Times New Roman','FontSize',size_)
    ylabel({'position error','(mm)'},'FontName','Times New Roman','FontSize',size_)
    ax = gca;
    ax.YColor = 'k';  % 设置左轴的坐标轴颜色为红色
    yyaxis right
    ylabel({'angular error','(°)'},'FontName','Times New Roman','FontSize',size_)
    ax = gca;
    ax.YColor = 'c';  % 设置左轴的坐标轴颜色为红色
    % title('keypoint error','FontName','Times New Roman','FontSize',size_)
    

    hold off
    % legend([h11,h12,h21,h22], 'position error using method in [31]','position error using the proposed method',...
    %     'angular error using method in [31]','angular error using the proposed method','FontName','Times New Roman','FontSize',size_);
    set(gca, 'FontSize', size_, 'FontName', 'Times New Roman');
    % legend show; % 显示图例
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
view([9 -58])
color = 3;
start_tip=1;

Target = [r0 p0';[0 0 0 1]];Target(2,4)=Target(2,4);
plotCoord(Target,1,1);

[~, Hd1]=plotCoord(T_cur1(:,:,start_tip),0.5,color);
p1=T_cur1(1:3,4,:);p1 = reshape(p1, [3,size(T_cur1,3)]);
line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r-',LineWidth=1);
p33=p1;

[P, Hd2]=plotCoord(T_cur2(:,:,start_tip),0.5,4);
p2=T_cur2(1:3,4,:);p2 = reshape(p2, [3,size(T_cur2,3)]);
line_handle2 = plot3([p2(1,1) p2(1,1)], [p2(2,1) p2(2,1)], [p2(3,1) p2(3,1)],'b-',LineWidth=1);
p22=p2;

% legend([line_handle1,line_handle2], 'trajectory using method in [25]','trajectory using the proposed method', ...
%     'FontName','Times New Roman','FontSize',size_);
% %
xlim([-40 25]);
ylim([-25 30]);
zlim([40 80]);
view([50 -67])
for i = 1:dis_b%size(pos_err,1)
    if(i == 1)
        % plotCoord(T_tar,2);
        plotCoord(T_cur1(:,:,1),0.5,color);
        plotCoord(T_cur2(:,:,1),0.5,4);
    else
        P=plotCoord(T_cur1(:,:,i),0.4,0);
        set(line_handle1, 'XData', p33(1,start_tip:i), 'YData', p33(2,start_tip:i),'ZData', p33(3,start_tip:i));
        set(Hd1(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
        set(Hd1(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
        set(Hd1(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
        t = ceil(i/3);
        if(t>block_size2)
            t = block_size2;
        end
        P=plotCoord(T_cur2(:,:,t),0.4,0);
        set(line_handle2, 'XData', p22(1,start_tip:t), 'YData', p22(2,start_tip:t),'ZData', p22(3,start_tip:t));
        set(Hd2(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
        set(Hd2(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
        set(Hd2(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
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
video.FrameRate = 30;  % 设置视频帧率
open(video);
% 遍历每个图像文件并添加到视频中
for iter = 1:dis_b%numel(imageFiles_spl)
    imageFile_main = fullfile(cur_path_main, imageFiles_main(iter).name);
    imageFile_aux = fullfile(cur_path_aux, imageFiles_aux(iter).name);
    imageFile_pln = fullfile(cur_path_pln, imageFiles_pln(iter).name);
    imageFile_spl = fullfile(cur_path_spl, imageFiles_spl(iter).name);
    
    t = ceil(iter/3);
    if(t>block_size2)
        t = block_size2;
    end
    imageFile_main2 = fullfile(cur_path_main, imageFiles_main(t+intersect).name);
    imageFile_aux2 = fullfile(cur_path_aux, imageFiles_aux(t+intersect).name);
    img_main = imread(imageFile_main);
    img_aux = imread(imageFile_aux);
    img_pln = imread(imageFile_pln);
    img_spl = imread(imageFile_spl);
    img_main2 = imread(imageFile_main2);
    img_aux2 = imread(imageFile_aux2);
    targetHeight = size(img_main,1);
    img_main = img_main(80:end,300:end,:);
    img_aux = img_aux(40:end-40,900:end,:);
    img_main2 = img_main2(80:end,300:end,:);
    img_aux2 = img_aux2(40:end-40,900:end,:);

    [pln_h,pln_w,~] = size(img_pln);
    [spl_h,spl_w,~] = size(img_spl);
    width = (size(img_main,2)+size(img_aux,2)) * 2;
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
    img=[img_main img_aux img_main2 img_aux2; img_pln_2 img_spl_2(:,2:end,:)];
    % tiledImg = imtile({img_main, img_spl,img_pln,img_aux});%, 'GridSize', [1, 2]});%
    scaledImage = imresize(img, 0.75);
    
    writeVideo(video, scaledImage);
end

% 关闭 VideoWriter 对象
close(video);
disp('视频创建完成。');


%% Frechet distance
Q0 = p1;
Q1 = p2;

block_size0 = size(Q0,1);
block_size1 = size(Q1,1);

P0 = discretizeLineSegment(p1(1,:),p0,block_size0);
P1 = discretizeLineSegment(p2(1,:),p0,block_size1);

distance0 = discreteFrechetDistance(P0, Q0);
distance1 = discreteFrechetDistance(P1, Q1);

fprintf('Frechet Distance Broyden: %.4f\n', distance0);
fprintf('Frechet Distance proposed: %.4f\n', distance1);