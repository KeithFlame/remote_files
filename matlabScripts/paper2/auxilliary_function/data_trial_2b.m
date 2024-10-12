p0=load("F:\code_git\matlabScripts\paper2\data_process\pic_factory\t2/cur_data_2.log");
p=p0(:,1:3);
r=p0(:,4:7);
pr=p0(:,1:7);
block_size = size(p,1);
ang_err=zeros(block_size,1);
ang_err2=zeros(block_size,1);
r0=[  
0.0051   -0.0172   -0.9998
    0.9890    0.1481    0.0025
    0.1480   -0.9888    0.0177
    ];
v=r0(:,1);
quat0=r(70,:);
T_cur=zeros(4,4,block_size);
window_size = 3;
% p(775:790,2)=9.1+rand(16,1)/2;
p(:,1) = smoothdata(p(:,1), 'movmean', window_size);
p(:,2) = smoothdata(p(:,2), 'movmean', window_size);
p(:,3) = smoothdata(p(:,3), 'movmean', window_size);
r(:,1) = smoothdata(r(:,1), 'movmean', window_size);
r(:,2) = smoothdata(r(:,2), 'movmean', window_size);
r(:,3) = smoothdata(r(:,3), 'movmean', window_size);
r(:,4) = smoothdata(r(:,4), 'movmean', window_size);

for i = 1:block_size
    ri=r(i,:);
    if(ri(1)<0)
        ri=-ri;
    end
    % ri=ri+(-ri+quat0)/2;
    % ri=ri*0.75+quat0*0.25;
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
    Ri1=Ri*eul2rotm([-theta 0 0]);
    Ri2=Ri*eul2rotm([-theta 0 0]);
    
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
        ri2=ri1*1;%0.72+quat0*0.28;
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
plot3(p(200:end,1),p(200:end,2),p(200:end,3),'r-');

% p1=[6 -20 89];p2=[-14 -20 89];
% p1=[3 -1 84];p2=[-16 -2.5 86];
% e1=(p2-p1)/norm(p2-p1);
% p10=p1;
% x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
% line(x, y,z, 'Color', 'blue', 'LineWidth', 2);
% 
% % p1=[-14 -20 89];p2=[-12 17 89];
% p1=[-16 -2.5 86];p2=[-14 -23 87];
% e2=(p2-p1)/norm(p2-p1);
% p20=p1;
% x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
% line(x, y,z, 'Color', 'blue', 'LineWidth', 2);
% 
% % p1=[-12 17 89];p2=[27 14 90];
% p1=[-14 -23 87];p2=[36 -18 80];
% e3=(p2-p1)/norm(p2-p1);
% p30=p1;
% x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
% line(x, y,z, 'Color', 'blue', 'LineWidth', 2);
% 
% % p1=[27 14 90];p2=[23.5 -22 89];
% p1=[36 -18 80];p2=[33 1.5 83];
% e4=(p2-p1)/norm(p2-p1);
% p40=p1;
% x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
% line(x, y,z, 'Color', 'blue', 'LineWidth', 2);
% 
% p1=[33 1.5 83];p2=[3 -1 84];
% % p1=[33 18.7 89];p2=[9 16 91.9];
% e5=(p2-p1)/norm(p2-p1);
% p50=p1;
% x=[p1(1) p2(1)];y=[p1(2) p2(2)];z=[p1(3) p2(3)];
% line(x, y,z, 'Color', 'blue', 'LineWidth', 2);

pp_m=p;

ratio_sc=0.75;

pos_err = zeros(block_size,1);
for i =70:225
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
for i =226:363
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
for i =364:622
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
for i =623:block_size
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
% plot3(pp_m(:,1),pp_m(:,2),pp_m(:,3),'c-');


%%

% x=[1.6241   -0.0816   -1.6260]';
% [y1,y2,y3,y4]=fmincon('get_min_ang',x);

%%
test_name = 't2';

pwwd='F:\code_git\matlabScripts\paper2\data_process';
cur_path_2=[pwwd,'\pic_factory\',test_name];
% cur_path_pln=[pwd,'\pic_factory\',test_name,'\temp\plane_fig'];
% cur_path_spl=[pwd,'\pic_factory\',test_name,'\temp\spatial_fig'];
cur_path_main=[pwwd,'\pic_factory\',test_name,'\after_pic'];
imageFiles_main = dir(fullfile(cur_path_main, '*.jpg')); % 修改为您需要的图片格式
% imageFiles_aux = dir(fullfile(cur_path_aux, '*.jpg')); % 修改为您需要的图片格式
% imageFiles_pln = dir(fullfile(cur_path_pln, '*.jpg')); % 修改为您需要的图片格式
% imageFiles_spl = dir(fullfile(cur_path_spl, '*.jpg')); % 修改为您需要的图片格式


% 创建 VideoWriter 对象
outputVideoPath = [cur_path_2,'/main.mp4'];
video = VideoWriter(outputVideoPath, 'MPEG-4');
video.FrameRate = 23;  % 设置视频帧率
open(video);
% 遍历每个图像文件并添加到视频中
for iter = 150:numel(imageFiles_main)
    imageFile_main = fullfile(cur_path_main, imageFiles_main(iter).name);
    % imageFile_aux = fullfile(cur_path_aux, imageFiles_aux(iter).name);
    % imageFile_pln = fullfile(cur_path_pln, imageFiles_pln(iter).name);
    % imageFile_spl = fullfile(cur_path_spl, imageFiles_spl(iter).name);
    img_main = imread(imageFile_main);
    % img_aux = imread(imageFile_aux);
    % img_pln = imread(imageFile_pln);
    % img_spl = imread(imageFile_spl);
    % targetHeight = size(img_main,1);
    % img_main = img_main(80:end,1+427:end-87,:);
    % img_aux = img_aux(100:end,1:end-350,:);

    % [originalHeight, originalWidth, ~] = size(img_main);
    % if(iter==987)
    %     break;
    % end
    % % 计算缩放比例
    % scaleFactor1 = 1094/1250;
    % img_pln_2=imresize(img_pln, scaleFactor1);
    % img_fig=[img_spl;img_pln_2];
    % scaleFactor2 = 669/1505;
    % img_fig_2=imresize(img_fig, scaleFactor2);
    % scaleFactor3 = 669/originalHeight;
    % img_main_2=imresize(img_main, scaleFactor3);
    % 根据目标高度进行缩放
    % img_aux = imresize(img_aux, scaleFactor);
    % img=[img_aux img_main_2 img_fig_2];
    % tiledImg = imtile({img_main, img_spl,img_pln,img_aux});%, 'GridSize', [1, 2]});%
    scaledImage = imresize(img_main, 0.75);
    
    writeVideo(video, scaledImage);
end

% 关闭 VideoWriter 对象
close(video);
disp('视频创建完成。');