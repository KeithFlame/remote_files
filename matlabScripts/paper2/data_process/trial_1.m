data_path_1 = [pwd,'/pic_factory/test_0331_1'];
data_path_2 = [pwd,'/pic_factory/test_0331_2'];
data_path_3 = [pwd,'/pic_factory/test_0331_3'];

save_path_pln = [pwd,'/pic_factory/trial_1/plane_fig_pos'];

[T_tar_1,T_cur_1,pos_err_1,ang_err_1]=data_todo(data_path_1);
[T_tar_2,T_cur_2,pos_err_2,ang_err_2]=data_todo(data_path_2);
[T_tar_3,T_cur_3,pos_err_3,ang_err_3]=data_todo(data_path_3);
size_1 = size(pos_err_1,1);
size_2 = size(pos_err_2,1);
size_3 = size(pos_err_3,1);
max_size = max([size_1,size_2,size_3]);
size_ = 20;
max_pos_err=max([pos_err_1;pos_err_2;pos_err_3])+2;
max_ang_err=max([ang_err_1;ang_err_2;ang_err_3])+2;

%% position error
% 创建图表并设置宽度
figure('Position', [100, 100, 800, 300]);  % 设置图表的左下角位置和宽度，高度设为0
hold on;
xlim([0 max_size+1])
ylim([0 max_pos_err+1])
for iter = 1:max_size
    x_iter = 1:iter;
    if(iter<size_1+1)
        plot(x_iter,pos_err_1(1:iter),'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','r');
    else
        plot(1:size_1,pos_err_1,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','r');
    end

    if(iter<size_2+1)
        plot(x_iter,pos_err_2(1:iter),'color',[0,1,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','g');
    else
        plot(1:size_2,pos_err_2,'color',[0,1,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','g');
    end
    
    if(iter<size_3+1)
        plot(x_iter,pos_err_3(1:iter),'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','b');
    else
        plot(1:size_3,pos_err_3,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','b');
    end
    
    set(gcf,'color','white')
    title('position error','FontName','Times New Roman','FontSize',size_)
    legend('trial 1','trial 2','trial 3','FontName','Times New Roman','FontSize',size_);
    xlabel('iteration','FontName','Times New Roman','FontSize',size_)
    ylabel('position error (mm)','FontName','Times New Roman','FontSize',size_)
    set(gca, 'FontSize', size_); % 设置刻度值的字体大小为12
    ns='';
    if(iter<10)
        ns=['M00',num2str(iter-1),'.jpg'];
    elseif(iter<100)
        ns=['M0',num2str(iter-1),'.jpg'];
    else
        ns=['M',num2str(iter-1),'.jpg'];
    end
    save_path1=[save_path_pln,'/',ns];
    saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
    cla;
end

%% angular error
save_path_pln2 = [pwd,'/pic_factory/trial_1/plane_fig_ang'];
figure('Position', [100, 100, 800, 300]);  % 设置图表的左下角位置和宽度，高度设为0
hold on;
xlim([0 max_size+1])
ylim([0 max_ang_err+1])
for iter = 1:max_size
    x_iter = 1:iter;
    if(iter<size_1+1)
        plot(x_iter,ang_err_1(1:iter),'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','r');
    else
        plot(1:size_1,ang_err_1,'color',[1,0,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','r');
    end

    if(iter<size_2+1)
        plot(x_iter,ang_err_2(1:iter),'color',[0,1,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','g');
    else
        plot(1:size_2,ang_err_2,'color',[0,1,0],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','g');
    end
    
    if(iter<size_3+1)
        plot(x_iter,ang_err_3(1:iter),'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','b');
    else
        plot(1:size_3,ang_err_3,'color',[0,0,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
         'MarkerFace','b');
    end
    
    set(gcf,'color','white')
    title('angular error','FontName','Times New Roman','FontSize',size_)
    legend('trial 1','trial 2','trial 3','FontName','Times New Roman','FontSize',size_);
    xlabel('iteration','FontName','Times New Roman','FontSize',size_)
    ylabel('angular error (°)','FontName','Times New Roman','FontSize',size_)
    set(gca, 'FontSize', size_); % 设置刻度值的字体大小为12
    ns='';
    if(iter<10)
        ns=['M00',num2str(iter-1),'.jpg'];
    elseif(iter<100)
        ns=['M0',num2str(iter-1),'.jpg'];
    else
        ns=['M',num2str(iter-1),'.jpg'];
    end
    save_path1=[save_path_pln2,'/',ns];
    saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
    cla;
end

%% spatial pose
save_path_pln3 = [pwd,'/pic_factory/trial_1/spatial_fig'];
figure('Position', [100, 100, 700, 700]);
hold on; grid on;axis equal;
xlabel('x (mm)','FontName','Times New Roman','FontSize',size_);
ylabel('y (mm)','FontName','Times New Roman','FontSize',size_);
zlabel('z (mm)','FontName','Times New Roman','FontSize',size_);
set(gca,'FontSize',18);
color = 3;
[~, Hd1]=plotCoord(T_cur_1(:,:,1),0.8,2);
[~, Hd2]=plotCoord(T_cur_2(:,:,1),0.8,3);
[~, Hd3]=plotCoord(T_cur_3(:,:,1),0.8,4);
plotCoord(T_tar_1,1.2,2);
plotCoord(T_tar_2,1.2,3);
plotCoord(T_tar_3,1.2,4);
view([132 21])


p1=T_cur_1(1:3,4,:);p1 = reshape(p1, [3,size_1]);p1(1,:) = smoothdata(p1(1,:), 'movmean', 3);
p1(2,:) = smoothdata(p1(2,:), 'movmean', 3);p1(3,:) = smoothdata(p1(3,:), 'movmean', 3);
p2=T_cur_2(1:3,4,:);p2 = reshape(p2, [3,size_2]);p2(1,:) = smoothdata(p2(1,:), 'movmean', 3);
p2(2,:) = smoothdata(p2(2,:), 'movmean', 3);p2(3,:) = smoothdata(p2(3,:), 'movmean', 3);
p3=T_cur_3(1:3,4,:);p3 = reshape(p3, [3,size_3]);p3(1,:) = smoothdata(p3(1,:), 'movmean', 3);
p3(2,:) = smoothdata(p3(2,:), 'movmean', 3);p3(3,:) = smoothdata(p3(3,:), 'movmean', 3);
line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r-');
line_handle2 = plot3([p2(1,1) p2(1,1)], [p2(2,1) p2(2,1)], [p2(3,1) p2(3,1)],'g-');
line_handle3 = plot3([p3(1,1) p3(1,1)], [p3(2,1) p3(2,1)], [p3(3,1) p3(3,1)],'b-');

for i = 1:max_size
        if(i == 1)
            plotCoord(T_cur_1(:,:,1),0.8,2);
        else
            if(i<size_1+1)
                P=plotCoord(T_cur_1(:,:,i),0.3,0);
                set(line_handle1, 'XData', p1(1,1:i), 'YData', p1(2,1:i),'ZData', p1(3,1:i));
                set(Hd1(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
                set(Hd1(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
                set(Hd1(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
            end
        end
        if(i == 1)
            plotCoord(T_cur_2(:,:,1),0.8,3);
        else
            if(i<size_2+1)
                P=plotCoord(T_cur_2(:,:,i),0.3,0);
                set(line_handle2, 'XData', p2(1,1:i), 'YData', p2(2,1:i),'ZData', p2(3,1:i));
                set(Hd2(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
                set(Hd2(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
                set(Hd2(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
            end
        end
        if(i == 1)
            plotCoord(T_cur_3(:,:,1),0.8,4);
        else
            if(i<size_3+1)
                P=plotCoord(T_cur_3(:,:,i),0.3,0);
                set(line_handle3, 'XData', p3(1,1:i), 'YData', p3(2,1:i),'ZData', p3(3,1:i));
                set(Hd3(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
                set(Hd3(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
                set(Hd3(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
            end
        end      
    ns='';
    if(i<10)
        ns=['M00',num2str(i-1),'.jpg'];
    elseif(i<100)
        ns=['M0',num2str(i-1),'.jpg'];
    else
        ns=['M',num2str(i-1),'.jpg'];
    end
    save_path1=[save_path_pln3,'/',ns];
    saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
end

%% video

save_path_avi = [pwd,'/pic_factory/trial_1/'];

data_path_pos = [pwd,'/pic_factory/trial_1/plane_fig_pos/'];
data_path_ang = [pwd,'/pic_factory/trial_1/plane_fig_ang/'];
data_path_spl = [pwd,'/pic_factory/trial_1/spatial_fig/'];

data_path_p1_main = [pwd,'/pic_factory/test_0331_1/after_pic/'];
data_path_p1_aux = [pwd,'/pic_factory/test_0331_1/aux_pic/'];
data_path_p2_main = [pwd,'/pic_factory/test_0331_2/after_pic/'];
data_path_p2_aux = [pwd,'/pic_factory/test_0331_2/aux_pic/'];
data_path_p3_main = [pwd,'/pic_factory/test_0331_3/after_pic/'];
data_path_p3_aux = [pwd,'/pic_factory/test_0331_3/aux_pic/'];

imageFiles_p1_main = dir(fullfile(data_path_p1_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_p1_aux = dir(fullfile(data_path_p1_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_p2_main = dir(fullfile(data_path_p2_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_p2_aux = dir(fullfile(data_path_p2_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_p3_main = dir(fullfile(data_path_p3_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_p3_aux = dir(fullfile(data_path_p3_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_pos = dir(fullfile(data_path_pos, '*.jpg')); % 修改为您需要的图片格式
imageFiles_ang = dir(fullfile(data_path_ang, '*.jpg')); % 修改为您需要的图片格式
imageFiles_spl = dir(fullfile(data_path_spl, '*.jpg')); % 修改为您需要的图片格式


% 创建 VideoWriter 对象
outputVideoPath = [save_path_avi,'/main.mp4'];
video = VideoWriter(outputVideoPath, 'MPEG-4');
video.FrameRate = 21;  % 设置视频帧率
open(video);
size0=numel(imageFiles_pos);
size1=numel(imageFiles_p1_main);
size2=numel(imageFiles_p2_main);
size3=numel(imageFiles_p3_main);

% 遍历每个图像文件并添加到视频中
for iter = 1:size0
    if(iter<size1+1)
        imageFile_p1_main = fullfile(data_path_p1_main, imageFiles_p1_main(iter).name);
        imageFile_p1_aux = fullfile(data_path_p1_aux, imageFiles_p1_aux(iter).name);
    else
        imageFile_p1_main = fullfile(data_path_p1_main, imageFiles_p1_main(size1).name);
        imageFile_p1_aux = fullfile(data_path_p1_aux, imageFiles_p1_aux(size1).name);
    end
    if(iter<size2+1)
        imageFile_p2_main = fullfile(data_path_p2_main, imageFiles_p2_main(iter).name);
        imageFile_p2_aux = fullfile(data_path_p2_aux, imageFiles_p2_aux(iter).name);
    else
        imageFile_p2_main = fullfile(data_path_p2_main, imageFiles_p2_main(size2).name);
        imageFile_p2_aux = fullfile(data_path_p2_aux, imageFiles_p2_aux(size2).name);
    end
    if(iter<size3+1)
        imageFile_p3_main = fullfile(data_path_p3_main, imageFiles_p3_main(iter).name);
        imageFile_p3_aux = fullfile(data_path_p3_aux, imageFiles_p3_aux(iter).name);
    else
        imageFile_p3_main = fullfile(data_path_p3_main, imageFiles_p3_main(size3).name);
        imageFile_p3_aux = fullfile(data_path_p3_aux, imageFiles_p3_aux(size3).name);
    end
    imageFile_pos = fullfile(data_path_pos, imageFiles_pos(iter).name);
    imageFile_ang = fullfile(data_path_ang, imageFiles_ang(iter).name);
    imageFile_spl = fullfile(data_path_spl, imageFiles_spl(iter).name);

    img_p1_main = imread(imageFile_p1_main);
    img_p1_aux = imread(imageFile_p1_aux);
    img_p2_main = imread(imageFile_p2_main);
    img_p2_aux = imread(imageFile_p2_aux);
    img_p3_main = imread(imageFile_p3_main);
    img_p3_aux = imread(imageFile_p3_aux);
    img_pos = imread(imageFile_pos);
    img_ang = imread(imageFile_ang);
    img_spl = imread(imageFile_spl);

    img_p1_main_c = img_p1_main(72:end,200:1920,:);
    img_p1_aux_c = img_p1_aux(300:end,300:2000,:);
    img_p2_main_c = img_p2_main(72:end,200:1920,:);
    img_p2_aux_c = img_p2_aux(300:end,300:2000,:);
    img_p3_main_c = img_p3_main(72:end,200:1920,:);
    img_p3_aux_c = img_p3_aux(300:end,300:2000,:);
    % img_spl_c = img_spl(:,31:end-31,:);
    
    targetHeight_ma=600;
    targetHeight_spl=1094;
    targetHeight_fig = 1800;
    [originalHeight1, ~, ~] = size(img_p1_main_c);
    [originalHeight2, ~, ~] = size(img_p1_aux_c);
    [~, originalHeight3, ~] = size(img_ang);

    % 计算缩放比例
    scaleFactor_main = targetHeight_ma / originalHeight1;
    scaleFactor_aux = targetHeight_ma / originalHeight2;
    scaleFactor_spl = targetHeight_spl / originalHeight3;

    % 根据目标高度进行缩放
    img_p1_main_c1 = imresize(img_p1_main_c, scaleFactor_main);
    img_p1_aux_c1 = imresize(img_p1_aux_c, scaleFactor_aux);
    img_p2_main_c1 = imresize(img_p2_main_c, scaleFactor_main);
    img_p2_aux_c1 = imresize(img_p2_aux_c, scaleFactor_aux);
    img_p3_main_c1 = imresize(img_p3_main_c, scaleFactor_main);
    img_p3_aux_c1 = imresize(img_p3_aux_c, scaleFactor_aux);
    % img_spl_c1 = imresize(img_spl_c, scaleFactor_spl);
    
    img_fig_pln=[img_pos;img_ang];
    img_fig_pln_c1 = imresize(img_fig_pln, scaleFactor_spl);
    
    img_fig = [img_spl;img_fig_pln_c1];
    [originalHeight4, ~, ~] = size(img_fig);
    scaleFactor_fig = targetHeight_fig / originalHeight4;
    img_fig_c1 = imresize(img_fig, scaleFactor_fig);

    % 创建空彩色图像
    row_concatenated_image = [img_p1_aux_c1 img_p1_main_c1 ; img_p2_aux_c1 img_p2_main_c1 ;
       img_p3_aux_c1 img_p3_main_c1 ];
    img = [row_concatenated_image img_fig_c1];
        % img_spl_c1(:,1:500,:) img_pos img_ang];
    img = imresize(img, 0.5);

    writeVideo(video, img);
end

close(video);
disp('视频创建完成。');