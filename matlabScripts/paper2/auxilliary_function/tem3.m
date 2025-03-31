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
quat0=r(70,:);
T_cur=zeros(4,4,block_size);
window_size = 5;
p(:,1) = smoothdata(p(:,1), 'movmean', window_size);
p(:,2) = smoothdata(p(:,2), 'movmean', window_size);
p(:,3) = smoothdata(p(:,3), 'movmean', window_size);
position = p;
%% get tar_orientation
x=[1.6029    0.0143   -1.4871]';
[y1,y2,y3,y4]=fmincon('get_min_ang',x);

%% get tar_position

x = rand(9,1);
x =x+ [-3.1097   15.7548   37.8010   52.6679   41.5699   24.8859   21.8915   83.9668    0.3897]';
[y1,y2,y3,y4]=fmincon('get_min_position',x);

figure;
hold on; axis equal;
plot3(position(:,1),position(:,2),position(:,3),'r.');

R = eul2rotm(y1(1:3)');
width = y1(4);
height = y1(5);
P1 = y1(6:8);
P2 = P1 - R(:,1)*width;
P3 = P2 + R(:,2)*height;
P4 = P3 + R(:,1)*width;
P_tar = [P1 P2 P3 P4 P1]';
plot3(P_tar(:,1),P_tar(:,2),P_tar(:,3),'b-');
xlabel('x (mm)')
ylabel('y (mm)')
zlabel('z (mm)')

%%

folderPath = 'F:\code_git\matlabScripts\paper2\data_process\pic_factory\trial_51\final_pic\';%after_pic
videoFileName = 'output';
imagesToVideo(folderPath, videoFileName);
function imagesToVideo(folderPath, videoFileName)
    % 输入参数：
    % folderPath - 包含图片的文件夹路径
    % videoFileName - 输出视频文件的名称（包括扩展名，如 'output.avi'）

    % 获取文件夹中的所有图像文件
    imageFiles = dir(fullfile(folderPath, '*.jpg')); % 可根据需要修改文件类型
    % 如果图像格式不同，可以添加其他格式，如 '*.png', '*.jpeg' 等
    
    % 创建视频写入对象
    videoWriter = VideoWriter(videoFileName);
    videoWriter.FrameRate = 20; % 设置帧率
    open(videoWriter);
    
    % 循环读取每张图片并写入视频
    for i = 1:length(imageFiles)
        % 读取图像
        img = imread(fullfile(folderPath, imageFiles(i).name));
        
        % 写入视频帧
        writeVideo(videoWriter, img);
    end
    
    % 关闭视频写入对象
    close(videoWriter);
    
    fprintf('视频已保存到: %s\n', videoFileName);
end