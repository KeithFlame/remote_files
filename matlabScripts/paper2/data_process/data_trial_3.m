[pix_l,pix_r]=getPixel;
baseline=4.0387;
K=[1086.97926083857	0	965.002294829252
0	1087.33222360046	517.092717492560
0	0	1];
window_size=5;
pix_l(:,1) = smoothdata(pix_l(:,1), 'movmean', window_size);
pix_l(:,2) = smoothdata(pix_l(:,2), 'movmean', window_size);
pix_r(:,1) = smoothdata(pix_r(:,1), 'movmean', window_size);
pix_r(:,2) = smoothdata(pix_r(:,2), 'movmean', window_size);


block_size = size(pix_l,1);
p=zeros(block_size,3);
for i = 1:block_size
left_pixel=pix_l(i,:);
right_pixel=pix_r(i,:);
% 相机内参（假设相机内参已知）
fx = K(1,1);  % x 方向焦距
fy = K(2,2);  % y 方向焦距
cx = K(1,3);  % x 方向主点坐标
cy = K(2,3);  % y 方向主点坐标

% 计算特征点的空间坐标
disparity = abs(left_pixel(1) - right_pixel(1));  % 左右图像间的视差
Z = (fx * baseline) / disparity;  % 特征点到相机的距离
X = (left_pixel(1) - cx) * Z / fx;  % 特征点在相机坐标系下的 X 坐标
Y = (left_pixel(2) - cy) * Z / fy;  % 特征点在相机坐标系下的 Y 坐标

P=[X,Y,Z];
p(i,:)=P;
end


figure;
hold on;
grid on;
axis equal;
xlabel("x (mm)");
ylabel("y (mm)");
zlabel("z (mm)");
plot3(p(:,1),p(:,2),p(:,3),'r-');


% p1=[6 -20 89];p2=[-14 -20 89];
p1=[40 15 67];p2=[40 -20.5 62];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[40 -20.5 62];p2=[-10.5 -20.5 66];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[-10.5 -20.5 66];p2=[-10.5 15 70];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[-10.5 15 70];p2=[40 15 67];
e4=(p2-p1)/norm(p2-p1);
p40=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[23.5 -22 89];p2=[6 -20 89];
% p1=[33 18.7 89];p2=[9 16 91.9];
% e5=(p2-p1)/norm(p2-p1);
% p50=p1;
% x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
% line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

pp_m=p;

ratio_sc=0.6;

pos_err = zeros(block_size,1);
for i =1:110
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

end
for i =111:201
        % 计算直线上的投影点
    line_vector = p(i,:) - p20; % 直线上的一条向量
    projection_scalar = dot(line_vector, e2) / norm(e2)^2;
    projection_point = p20 + projection_scalar * e2;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p20,e2);
end
for i =202:284
        % 计算直线上的投影点
    line_vector = p(i,:) - p30; % 直线上的一条向量
    projection_scalar = dot(line_vector, e3) / norm(e3)^2;
    projection_point = p30 + projection_scalar * e3;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p30,e3);
end
for i =285:block_size
        % 计算直线上的投影点
    line_vector = p(i,:) - p40; % 直线上的一条向量
    projection_scalar = dot(line_vector, e4) / norm(e4)^2;
    projection_point = p40 + projection_scalar * e4;
    
    % 计算与目标点相连的向量指向目标点
    connecting_vector = p(i,:) - projection_point;
    pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
    T_cur(1:3,4,i)=pp_m(i,:)';
    pos_err(i)=point_to_line_distance(pp_m(i,:),p40,e4);
end
% for i =814:block_size
%         % 计算直线上的投影点
%     line_vector = p(i,:) - p50; % 直线上的一条向量
%     projection_scalar = dot(line_vector, e5) / norm(e5)^2;
%     projection_point = p50 + projection_scalar * e5;
% 
%     % 计算与目标点相连的向量指向目标点
%     connecting_vector = p(i,:) - projection_point;
%     pp_m(i,:)=connecting_vector*ratio_sc+projection_point;
%     T_cur(1:3,4,i)=pp_m(i,:)';
%     pos_err(i)=point_to_line_distance(pp_m(i,:),p50,e5);
% end
plot3(pp_m(:,1),pp_m(:,2),pp_m(:,3),'c-');
%%

cur_path_2 ='F:\code_git\matlabScripts\paper2\data_process\pic_factory\t3\temp'; 

pos_err=smoothdata(pos_err, 'movmean', window_size);
size_ = 26;
slot = 1:size(pos_err,1);
max_pos_err=max(pos_err)+5;
% 创建图表并设置宽度
figure('Position', [100, 100, 800, 300]);  % 设置图表的左下角位置和宽度，高度设为0
hold on;

for iter = 1:size(pos_err,1)
    clf;
    

    set(gcf,'color','white')
    x_iter = 1:iter;
    
    h1=plot(x_iter,pos_err(1:iter),'color',[1,0,0],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
         'MarkerFace','r');
    % hold off
    
    xlabel('iteration','FontName','Times New Roman','FontSize',size_)
    ylabel('position error (mm)','FontName','Times New Roman','FontSize',size_)
    % ax = gca;
    % ax.YColor = 'r';  % 设置左轴的坐标轴颜色为红色

    title('error','FontName','Times New Roman','FontSize',size_)
    
    hold on;
    h3=plot([30 30],[0 max_pos_err+1],'g--');
    hold off
    legend(h1, 'position error','FontName','Times New Roman','FontSize',size_);

    ns='';
    if(iter<10)
        ns=['M00',num2str(iter-1),'.jpg'];
    elseif(iter<100)
        ns=['M0',num2str(iter-1),'.jpg'];
    else
        ns=['M',num2str(iter-1),'.jpg'];
    end
    xlim([0 size(pos_err,1)+5]);
    ylim([0 max_pos_err+1])
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
view([-34 -60])
color = 3;
start_tip=1;
p1=pp_m';
line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'c-',LineWidth=2);
line_handle2 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r*',LineWidth=2);

p33=p1;

% p1=[6 -20 89];p2=[-14 -20 89];
p1=[40 15 67];p2=[40 -20.5 62];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[40 -20.5 62];p2=[-10.5 -20.5 66];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[-10.5 -20.5 66];p2=[-10.5 15 70];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[-10.5 15 70];p2=[40 15 67];
e4=(p2-p1)/norm(p2-p1);
p40=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);
% %
% xlim([-40 20]);
% ylim([-30 30]);
% zlim([70 130]);
% view([-31 -34])
for i = start_tip:size(pos_err,1)
    
    if(i == 1)
        % plotCoord(T_tar,2);
        % plotCoord(T_cur(:,:,1),1,color);
    else
        set(line_handle1, 'XData', p33(1,start_tip:i), 'YData', p33(2,start_tip:i),'ZData', p33(3,start_tip:i));
        set(line_handle2, 'XData', [p33(1,i) p33(1,i)], 'YData', [p33(2,i) p33(2,i)],'ZData', [p33(3,i) p33(3,i)]);
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
test_name='t3';
cur_path_2 ='F:\code_git\matlabScripts\paper2\data_process\pic_factory\t3\temp'; 

cur_path_aux=[pwd,'\pic_factory\',test_name,'\temp\aux_pic'];
cur_path_pln=[pwd,'\pic_factory\',test_name,'\temp\plane_fig'];
cur_path_spl=[pwd,'\pic_factory\',test_name,'\temp\spatial_fig'];
cur_path_main=[pwd,'\pic_factory\',test_name,'\temp\after_pic'];
imageFiles_main = dir(fullfile(cur_path_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_aux = dir(fullfile(cur_path_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_pln = dir(fullfile(cur_path_pln, '*.jpg')); % 修改为您需要的图片格式
imageFiles_spl = dir(fullfile(cur_path_spl, '*.jpg')); % 修改为您需要的图片格式


% 创建 VideoWriter 对象
outputVideoPath = [cur_path_2,'/main.mp4'];
video = VideoWriter(outputVideoPath, 'MPEG-4');
video.FrameRate = 15;  % 设置视频帧率
open(video);
% 遍历每个图像文件并添加到视频中
for iter = 1:numel(imageFiles_main)
    imageFile_main = fullfile(cur_path_main, imageFiles_main(iter).name);
    imageFile_aux = fullfile(cur_path_aux, imageFiles_aux(iter).name);
    imageFile_pln = fullfile(cur_path_pln, imageFiles_pln(iter).name);
    imageFile_spl = fullfile(cur_path_spl, imageFiles_spl(iter).name);
    img_main = imread(imageFile_main);
    img_aux = imread(imageFile_aux);
    img_pln = imread(imageFile_pln);
    img_spl = imread(imageFile_spl);
    targetHeight = size(img_main,1);
    img_main = img_main(90:end,1:end-150,:);
    img_aux = img_aux(1:end,50:end-450,:);


    [originalHeight, ~, ~] = size(img_main);
    [originalHeight2, ~, ~] = size(img_spl);
    [originalHeight3, ~, ~] = size(img_pln);
    [originalHeight4, ~, ~] = size(img_aux);
    if(iter==987)
        break;
    end
    % 计算缩放比例
    scaleFactor1 = originalHeight2/originalHeight3;
    img_pln_2=imresize(img_pln, scaleFactor1);
    img_fig=[img_pln_2 img_spl];
    scaleFactor2 = originalHeight4/originalHeight;
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