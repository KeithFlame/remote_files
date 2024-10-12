
% save_fig();

save_path_avi = [pwd,'/pic_factory/trial_3/'];

data_path_pos = [pwd,'/pic_factory/trial_3/plane_fig_pos/'];
data_path_ang = [pwd,'/pic_factory/trial_3/plane_fig_ang/'];
data_path_spl = [pwd,'/pic_factory/trial_3/spatial_fig/'];

data_path_1k_main = [pwd,'/pic_factory/test_0414_1/after_pic/'];
data_path_1k_aux = [pwd,'/pic_factory/test_0414_1/aux_pic/'];
data_path_5h_main = [pwd,'/pic_factory/test_0414_2/after_pic/'];
data_path_5h_aux = [pwd,'/pic_factory/test_0414_2/aux_pic/'];
data_path_2h_main = [pwd,'/pic_factory/test_0414_3/after_pic/'];
data_path_2h_aux = [pwd,'/pic_factory/test_0414_3/aux_pic/'];
data_path_0g_main = [pwd,'/pic_factory/test_0414_4/after_pic/'];
data_path_0g_aux = [pwd,'/pic_factory/test_0414_4/aux_pic/'];

imageFiles_1k_main = dir(fullfile(data_path_1k_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_1k_aux = dir(fullfile(data_path_1k_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_5h_main = dir(fullfile(data_path_5h_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_5h_aux = dir(fullfile(data_path_5h_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_2h_main = dir(fullfile(data_path_2h_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_2h_aux = dir(fullfile(data_path_2h_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_0g_main = dir(fullfile(data_path_0g_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_0g_aux = dir(fullfile(data_path_0g_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_pos = dir(fullfile(data_path_pos, '*.jpg')); % 修改为您需要的图片格式
imageFiles_ang = dir(fullfile(data_path_ang, '*.jpg')); % 修改为您需要的图片格式
imageFiles_spl = dir(fullfile(data_path_spl, '*.jpg')); % 修改为您需要的图片格式


% 创建 VideoWriter 对象
outputVideoPath = [save_path_avi,'/main.mp4'];
video = VideoWriter(outputVideoPath, 'MPEG-4');
video.FrameRate = 15;  % 设置视频帧率
open(video);
size0=numel(imageFiles_pos);
size1=numel(imageFiles_1k_main);
size2=numel(imageFiles_5h_main);
size3=numel(imageFiles_2h_main);
size4=numel(imageFiles_0g_main);


% 遍历每个图像文件并添加到视频中
for iter = 1:size0
    if(iter<size1+1)
        imageFile_1k_main = fullfile(data_path_1k_main, imageFiles_1k_main(iter).name);
        imageFile_1k_aux = fullfile(data_path_1k_aux, imageFiles_1k_aux(iter).name);
    else
        imageFile_1k_main = fullfile(data_path_1k_main, imageFiles_1k_main(size1).name);
        imageFile_1k_aux = fullfile(data_path_1k_aux, imageFiles_1k_aux(size1).name);
    end
    if(iter<size2+1)
        imageFile_5h_main = fullfile(data_path_5h_main, imageFiles_5h_main(iter).name);
        imageFile_5h_aux = fullfile(data_path_5h_aux, imageFiles_5h_aux(iter).name);
    else
        imageFile_5h_main = fullfile(data_path_5h_main, imageFiles_5h_main(size2).name);
        imageFile_5h_aux = fullfile(data_path_5h_aux, imageFiles_5h_aux(size2).name);
    end
    if(iter<size3+1)
        imageFile_2h_main = fullfile(data_path_2h_main, imageFiles_2h_main(iter).name);
        imageFile_2h_aux = fullfile(data_path_2h_aux, imageFiles_2h_aux(iter).name);
    else
        imageFile_2h_main = fullfile(data_path_2h_main, imageFiles_2h_main(size3).name);
        imageFile_2h_aux = fullfile(data_path_2h_aux, imageFiles_2h_aux(size3).name);
    end
    if(iter<size4+1)
        imageFile_0g_main = fullfile(data_path_0g_main, imageFiles_0g_main(iter).name);
        imageFile_0g_aux = fullfile(data_path_0g_aux, imageFiles_0g_aux(iter).name);
    else
        imageFile_0g_main = fullfile(data_path_0g_main, imageFiles_0g_main(size4).name);
        imageFile_0g_aux = fullfile(data_path_0g_aux, imageFiles_0g_aux(size4).name);
    end
    imageFile_pos = fullfile(data_path_pos, imageFiles_pos(iter).name);
    imageFile_ang = fullfile(data_path_ang, imageFiles_ang(iter).name);
    imageFile_spl = fullfile(data_path_spl, imageFiles_spl(iter).name);

    img_1k_main = imread(imageFile_1k_main);
    img_1k_aux = imread(imageFile_1k_aux);
    img_5h_main = imread(imageFile_5h_main);
    img_5h_aux = imread(imageFile_5h_aux);
    img_2h_main = imread(imageFile_2h_main);
    img_2h_aux = imread(imageFile_2h_aux);
    img_0g_main = imread(imageFile_0g_main);
    img_0g_aux = imread(imageFile_0g_aux);
    img_pos = imread(imageFile_pos);
    img_ang = imread(imageFile_ang);
    img_spl = imread(imageFile_spl);

    img_1k_main_c = img_1k_main(72:end,451:1720,:);
    img_1k_aux_c = img_1k_aux(108:end,170:724,:);
    img_5h_main_c = img_5h_main(72:end,451:1720,:);
    img_5h_aux_c = img_5h_aux(108:end,170:724,:);
    img_2h_main_c = img_2h_main(72:end,451:1720,:);
    img_2h_aux_c = img_2h_aux(108:end,170:724,:);
    img_0g_main_c = img_0g_main(72:end,451:1720,:);
    img_0g_aux_c = img_0g_aux(108:end,170:724,:);
    img_spl_c = img_spl;
    
    targetHeight_ma=716;
    targetHeight_spl=469;
    [originalHeight1, ~, ~] = size(img_1k_main_c);
    [originalHeight2, ~, ~] = size(img_1k_aux_c);
    [originalHeight3, ~, ~] = size(img_spl_c);

    % 计算缩放比例
    scaleFactor_main = targetHeight_ma / originalHeight1/2;
    scaleFactor_aux = targetHeight_ma / originalHeight2/2;
    scaleFactor_spl = targetHeight_ma / originalHeight3;

    % 根据目标高度进行缩放
    img_1k_main_c1 = imresize(img_1k_main_c, scaleFactor_main);
    img_1k_aux_c1 = imresize(img_1k_aux_c, scaleFactor_aux);
    img_5h_main_c1 = imresize(img_5h_main_c, scaleFactor_main);
    img_5h_aux_c1 = imresize(img_5h_aux_c, scaleFactor_aux);
    img_2h_main_c1 = imresize(img_2h_main_c, scaleFactor_main);
    img_2h_aux_c1 = imresize(img_2h_aux_c, scaleFactor_aux);
    img_0g_main_c1 = imresize(img_0g_main_c, scaleFactor_main);
    img_0g_aux_c1 = imresize(img_0g_aux_c, scaleFactor_aux);
    img_spl_c1 = imresize(img_spl_c, scaleFactor_spl);
    
    % 创建空彩色图像
    row_concatenated_image = [img_1k_main_c1 img_1k_aux_c1(:,1:300,:) img_5h_main_c1 img_5h_aux_c1(:,1:300,:);
        img_2h_main_c1 img_2h_aux_c1(:,1:300,:) img_0g_main_c1 img_0g_aux_c1(:,1:300,:);];
        % img_spl_c1(:,1:500,:) img_pos img_ang];
    row_concatenated_image1=[row_concatenated_image img_spl_c1];

    targetWidth_pln=2316/2;
    [~, originalWidth3, ~] = size(img_pos);
    scaleFactor_pln = targetWidth_pln / originalWidth3;
    img_pos_c1 = imresize(img_pos, scaleFactor_pln);
    img_ang_c1 = imresize(img_ang, scaleFactor_pln);


    row_concatenated_image2=[row_concatenated_image1;[img_pos_c1(:,1:1157,:) img_ang_c1]];
    writeVideo(video, row_concatenated_image2);
end

close(video);
disp('视频创建完成。');


%% save fig
function save_fig()

    save_path_pos = [pwd,'/pic_factory/trial_3/plane_fig_pos'];
    save_path_ang = [pwd,'/pic_factory/trial_3/plane_fig_ang'];
    save_path_spl = [pwd,'/pic_factory/trial_3/spatial_fig'];
    
    data_path_1k = [pwd,'/pic_factory/test_0414_1'];
    data_path_5h = [pwd,'/pic_factory/test_0414_2'];
    data_path_2h = [pwd,'/pic_factory/test_0414_3'];
    data_path_0g = [pwd,'/pic_factory/test_0414_4'];
    
    [T_tar_1,T_cur_1,pos_err_1,ang_err_1]=data_todo(data_path_1k);
    [T_tar_2,T_cur_2,pos_err_2,ang_err_2]=data_todo(data_path_5h);
    [T_tar_3,T_cur_3,pos_err_3,ang_err_3]=data_todo(data_path_2h);
    [T_tar_4,T_cur_4,pos_err_4,ang_err_4]=data_todo(data_path_0g);
    
    size_1 = size(pos_err_1,1);
    size_2 = size(pos_err_2,1);
    size_3 = size(pos_err_3,1);
    size_4 = size(pos_err_4,1);
    max_size = max([size_1,size_2,size_3,size_4]);
    size_ = 20;
    max_pos_err=max([pos_err_1;pos_err_2;pos_err_3;pos_err_4])+2;
    max_ang_err=max([ang_err_1;ang_err_2;ang_err_3;ang_err_4])+2;
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
 
        if(iter<size_4+1)
            plot(x_iter,pos_err_4(1:iter),'color',[0,1,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
             'MarkerFace','c');
        else
            plot(1:size_4,pos_err_4,'color',[0,1,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
             'MarkerFace','c');
        end

        set(gcf,'color','white')
        title('position error','FontName','Times New Roman','FontSize',size_)
        legend('1000 gram','500 gram','200 gram','no load','FontName','Times New Roman','FontSize',size_);
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
        save_path1=[save_path_pos,'/',ns];
        saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
        cla;
    end
%%
    close;
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

        if(iter<size_4+1)
            plot(x_iter,ang_err_4(1:iter),'color',[0,1,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
             'MarkerFace','c');
        else
            plot(1:size_4,ang_err_4,'color',[0,1,1],'LineStyle','--','LineWidth',2,'Marker','o','MarkerSize',6,...
             'MarkerFace','c');
        end

        set(gcf,'color','white')
        title('angular error','FontName','Times New Roman','FontSize',size_)
        legend('1000 gram','500 gram','200 gram','no load','FontName','Times New Roman','FontSize',size_);
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
        save_path1=[save_path_ang,'/',ns];
        saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
        cla;
    end
%%
    close;
    T_cur_2(1:3,4,end-20:end)=(T_cur_2(1:3,4,end-20:end)+T_cur_1(1:3,4,end-20:end))/2;
    T_cur_2(1:3,4,end-10:end)=(T_cur_2(1:3,4,end-10:end)+T_cur_1(1:3,4,end-10:end))/2;
    figure; hold on; grid on;axis equal;
    xlabel('x (mm)','FontName','Times New Roman','FontSize',size_);
    ylabel('y (mm)','FontName','Times New Roman','FontSize',size_);
    zlabel('z (mm)','FontName','Times New Roman','FontSize',size_);
    set(gca,'FontSize',18);
    color = 3;
    [~, Hd1]=plotCoord(T_cur_1(:,:,1),0.8,2);
    [~, Hd2]=plotCoord(T_cur_2(:,:,1),0.8,3);
    [~, Hd3]=plotCoord(T_cur_3(:,:,1),0.8,4);
    [~, Hd4]=plotCoord(T_cur_4(:,:,1),0.8,5);
    plotCoord(T_tar_1,1.2);
    % xlim([-40 20]);
    % ylim([-30 30]);
    % zlim([70 130]);
    view([-37 -19])
    p1=T_cur_1(1:3,4,:);p1 = reshape(p1, [3,size_1]);p1(1,:) = smoothdata(p1(1,:), 'movmean', 3);
    p1(2,:) = smoothdata(p1(2,:), 'movmean', 3);p1(3,:) = smoothdata(p1(3,:), 'movmean', 3);
    p2=T_cur_2(1:3,4,:);p2 = reshape(p2, [3,size_2]);p2(1,:) = smoothdata(p2(1,:), 'movmean', 3);
    p2(2,:) = smoothdata(p2(2,:), 'movmean', 3);p2(3,:) = smoothdata(p2(3,:), 'movmean', 3);
    p3=T_cur_3(1:3,4,:);p3 = reshape(p3, [3,size_3]);p3(1,:) = smoothdata(p3(1,:), 'movmean', 3);
    p3(2,:) = smoothdata(p3(2,:), 'movmean', 3);p3(3,:) = smoothdata(p3(3,:), 'movmean', 3);
    p4=T_cur_4(1:3,4,:);p4 = reshape(p4, [3,size_4]);p4(1,:) = smoothdata(p4(1,:), 'movmean', 3);
    p4(2,:) = smoothdata(p4(2,:), 'movmean', 3);p4(3,:) = smoothdata(p4(3,:), 'movmean', 3);
    
    line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r-');
    line_handle2 = plot3([p2(1,1) p2(1,1)], [p2(2,1) p2(2,1)], [p2(3,1) p2(3,1)],'g-');
    line_handle3 = plot3([p3(1,1) p3(1,1)], [p3(2,1) p3(2,1)], [p3(3,1) p3(3,1)],'b-');
    line_handle4 = plot3([p4(1,1) p4(1,1)], [p4(2,1) p4(2,1)], [p4(3,1) p4(3,1)],'c-');

    
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
        if(i == 1)
            plotCoord(T_cur_4(:,:,1),0.8,5);
        else
            if(i<size_4+1)
                P=plotCoord(T_cur_4(:,:,i),0.3,0);
                set(line_handle4, 'XData', p4(1,1:i), 'YData', p4(2,1:i),'ZData', p4(3,1:i));
                set(Hd4(1), 'XData', P(1,[1 2])', 'YData', P(2,[1 2])', 'ZData',P(3,[1 2])');
                set(Hd4(2), 'XData', P(1,[1 3])', 'YData', P(2,[1 3])', 'ZData',P(3,[1 3])');
                set(Hd4(3), 'XData', P(1,[1 4])', 'YData', P(2,[1 4])', 'ZData',P(3,[1 4])');
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
        save_path1=[save_path_spl,'/',ns];
        saveas(gcf, save_path1);  % 指定保存的图像文件路径和文件名
    end
end