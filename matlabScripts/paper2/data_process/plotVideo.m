
test_name = 't1';
% 获取当前路径
currentPath = pwd;


% 指定要检查和创建的文件夹路径
cur_path_main=[currentPath,'\pic_factory\',test_name,'\after_pic'];

% 检查文件夹是否存在
if ~exist(cur_path_main, 'dir')
    % 文件夹不存在，创建新的文件夹
    mkdir(cur_path_main);
    disp('文件夹已创建。');
else
    disp('文件夹已存在。');
end

% 指定要检查和创建的文件夹路径
cur_path_pln=[currentPath,'\pic_factory\',test_name,'\plane_fig'];

% 检查文件夹是否存在
if ~exist(cur_path_pln, 'dir')
    % 文件夹不存在，创建新的文件夹
    mkdir(cur_path_pln);
    disp('文件夹已创建。');
end

% 指定要检查和创建的文件夹路径
cur_path_spl=[currentPath,'\pic_factory\',test_name,'\spatial_fig'];

% 检查文件夹是否存在
if ~exist(cur_path_spl, 'dir')
    % 文件夹不存在，创建新的文件夹
    mkdir(cur_path_spl);
    disp('文件夹已创建。');
end

% 指定要检查和创建的文件夹路径
cur_path_spl=[currentPath,'\pic_factory\',test_name,'\final_pic'];

% 检查文件夹是否存在
if ~exist(cur_path_spl, 'dir')
    % 文件夹不存在，创建新的文件夹
    mkdir(cur_path_spl);
    disp('文件夹已创建。');
end

cur_path_2=[currentPath,'\pic_factory\',test_name];

%% plot cur_pose

python_path = 'F:\code_git\pythonScripts\endsAndOdds\plotCoord2.py ';
pythonScriptPath = [python_path,cur_path_2];
system(['python ', pythonScriptPath]);



%% plot cur_tar

python_path = 'F:\code_git\pythonScripts\endsAndOdds\plot_cur_tar.py ';
pythonScriptPath = [python_path,cur_path_2];
system(['python ', pythonScriptPath]);

%% data

% [T_tar,T_cur,pos_err,ang_err]=data_todo(cur_path_2);

window_size = 5;
% pos_err = smoothdata(pos_err, 'movmean', window_size);
pos_err = pos_err;
pos_err(1:70)=pos_err(1:70);
ang_err=ang_err2;
ang_err(1:70)=ang_err(1:70);
ang_err(106:end)=ang_err(106:end)*1;
% ang_err=abs(ang_err);
% ang_err = smoothdata(ang_err, 'movmean', window_size);

size_ = 26;
slot = 1:size(pos_err,1);
max_pos_err=max(pos_err)+5;
max_ang_err=max(ang_err)+5;
% 创建图表并设置宽度
figure('Position', [100, 100, 800, 300]);  % 设置图表的左下角位置和宽度，高度设为0
% hold on;


for iter = 1:size(pos_err,1)
    % cla;
    xlim([0 size(pos_err,1)+1]);
    
    set(gcf,'color','white')
    x_iter = 1:iter;
    
    
    yyaxis left
    % hold on;
    % plot([70 70],[0 max_pos_err],'g--');
    plot(x_iter,pos_err(1:iter),'color',[1,0,0],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
         'MarkerFace','r');
    % hold off
    ylim([0 max_pos_err])
    yyaxis right
    plot(x_iter,ang_err(1:iter),'color',[0,0,1],'LineStyle','--','LineWidth',1,'Marker','o','MarkerSize',3,...
        'MarkerFace','b');
    ylim([0 max_ang_err]);
    yyaxis left
    xlabel('iteration','FontName','Times New Roman','FontSize',size_)
    ylabel('position error (mm)','FontName','Times New Roman','FontSize',size_)
    ax = gca;
    ax.YColor = 'r';  % 设置左轴的坐标轴颜色为红色
    yyaxis right
    ylabel('angular error (°)','FontName','Times New Roman','FontSize',size_)
    ax = gca;
    ax.YColor = 'b';  % 设置左轴的坐标轴颜色为红色
    title('error','FontName','Times New Roman','FontSize',size_)
    legend('position error','angular error','FontName','Times New Roman','FontSize',size_);
    
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
view([-34 -60])
color = 3;
start_tip=1;

[P, Hd]=plotCoord(T_cur(:,:,start_tip),0.5,color);

p1=T_cur(1:3,4,:);p1 = reshape(p1, [3,size(T_cur,3)]);
line_handle1 = plot3([p1(1,1) p1(1,1)], [p1(2,1) p1(2,1)], [p1(3,1) p1(3,1)],'r-');
p33=p1;
ylabel("y (mm)");
zlabel("z (mm)");
% plot3(p(:,1),p(:,2),p(:,3),'r-');

% p1=[6 -20 89];p2=[-14 -20 89];
p1=[-44 15.5 94];p2=[-46 -25.5 90];
e1=(p2-p1)/norm(p2-p1);
p10=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-14 -20 89];p2=[-12 17 89];
p1=[-46 -25.5 90];p2=[0 -27 91];
e2=(p2-p1)/norm(p2-p1);
p20=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[-12 17 89];p2=[27 14 90];
p1=[0 -27 91];p2=[-0 14.5 93];
e3=(p2-p1)/norm(p2-p1);
p30=p1;
x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

% p1=[27 14 90];p2=[23.5 -22 89];
p1=[-0 14.5 93];p2=[-44 15.5 94];
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
        plotCoord(T_cur(:,:,1),1,color);
    else
        P=plotCoord(T_cur(:,:,i),0.5,0);
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
cur_path_aux=[currentPath,'\pic_factory\',test_name,'\aux_pic'];
cur_path_pln=[currentPath,'\pic_factory\',test_name,'\plane_fig'];
cur_path_spl=[currentPath,'\pic_factory\',test_name,'\spatial_fig'];
imageFiles_main = dir(fullfile(cur_path_main, '*.jpg')); % 修改为您需要的图片格式
imageFiles_aux = dir(fullfile(cur_path_aux, '*.jpg')); % 修改为您需要的图片格式
imageFiles_pln = dir(fullfile(cur_path_pln, '*.jpg')); % 修改为您需要的图片格式
imageFiles_spl = dir(fullfile(cur_path_spl, '*.jpg')); % 修改为您需要的图片格式


% 创建 VideoWriter 对象
outputVideoPath = [cur_path_2,'/main.mp4'];
video = VideoWriter(outputVideoPath, 'MPEG-4');
video.FrameRate = 21;  % 设置视频帧率
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
    img_main = img_main(80:end,1:end-127-427,:);
    img_aux = img_aux(50:end-50,50:end-280,:);

    % [originalHeight, originalWidth, ~] = size(img_aux);
    if(iter==987)
        break;
    end
    % 计算缩放比例
    scaleFactor1 = 1094/1250;
    img_pln_2=imresize(img_pln, scaleFactor1);
    img_fig=[img_spl;img_pln_2];
    scaleFactor2 = 669/1505;
    img_fig_2=imresize(img_fig, scaleFactor2);
    scaleFactor3 = 669/1001;
    img_main_2=imresize(img_main, scaleFactor3);
    % 根据目标高度进行缩放
    % img_aux = imresize(img_aux, scaleFactor);
    img=[img_aux img_main_2 img_fig_2];
    % tiledImg = imtile({img_main, img_spl,img_pln,img_aux});%, 'GridSize', [1, 2]});%
    scaledImage = imresize(img, 0.75);
    
    writeVideo(video, scaledImage);
end

% 关闭 VideoWriter 对象
close(video);
disp('视频创建完成。');

