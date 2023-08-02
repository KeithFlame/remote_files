function [] = sr_rectify(image_folder)
% SR_RECTIFY 立体校正
% 输入参数：
%   image_folder   ---  棋盘格图像路径,文件夹下需要包含left,right图像文件夹，以及
% 对应于图像的stereoParams的参数路径
% 例如： image_folder / left
%                    / right
%                    / stereoParams.mat
%
% Date: 2019.6.25 - 6.29, by WangLf.
% Revise the ROI with Denglf's approach.
% 2021.2.24. Revise the method as a function and optimize the codes, by Wanglf.
% 2021.8.24. Fixed an error; change the precision of the output cam_params.sr.
% Version 2.2.

% Define a global variable to store the useful parameters of the stereo-camera
global params
warning off;

time_point = tic;
fprintf('\n矫畸变程序启动，图像正在处理请等待...\n');
%% 1 Load parameters
data = load([image_folder,'/stereoParams.mat']);
% -----------------------------------------------------------------------------
% Retrieving parameters of Camera 1
% 1 Retrieve intrinsic matrix
params.intrinsic1 = data.stereoParams.CameraParameters1.IntrinsicMatrix';
% 1.1 Retrieve scale-factors of u-axe and v-axe of image, in pixel
fx_1 = params.intrinsic1(1,1);
fy_1 = params.intrinsic1(2,2);
% 1.2 Retrieve optical center (u,v)or(cx,cy) of image, in pixel
cx_1 = params.intrinsic1(1,3);
cy_1 = params.intrinsic1(2,3);

% 2 Retrieve radial and tangential distortion
params.dist_coeffs1 = zeros(1,5);
params.dist_coeffs1(1:length(data.stereoParams.CameraParameters1.RadialDistortion))...
    = data.stereoParams.CameraParameters1.RadialDistortion;
params.dist_coeffs1(4:5) = data.stereoParams.CameraParameters1.TangentialDistortion;
% 2.1 Distribute distortion
k1 = params.dist_coeffs1(1:3);
p1 = params.dist_coeffs1(4:5);
% -----------------------------------------------------------------------------
% Retrieving parameters of Camera 2
% 1 Retrieve intrinsic matrix
params.intrinsic2 = data.stereoParams.CameraParameters2.IntrinsicMatrix';
% 1.1 Retrieve scale-factors of u-axe and v-axe of image, in pixel
fx_2 = params.intrinsic2(1,1);
fy_2 = params.intrinsic2(2,2);
% 1.2 Retrieve optical center (u,v)or(cx,cy) of image, in pixel
cx_2 = params.intrinsic2(1,3);
cy_2 = params.intrinsic2(2,3);

% 2 Retrieve radial and tangential distortion
params.dist_coeffs2 = zeros(1,5);
params.dist_coeffs2(1:length(data.stereoParams.CameraParameters2.RadialDistortion))...
    = data.stereoParams.CameraParameters2.RadialDistortion;
params.dist_coeffs2(4:5) = data.stereoParams.CameraParameters2.TangentialDistortion;
% 2.1 Distribute distortion
k2 = params.dist_coeffs2(1:3);
p2 = params.dist_coeffs1(4:5);
% -----------------------------------------------------------------------------
% Calculateing rectifying parameters
% Get the rotation and translation relationship between camera 1 and 2
R_2to1 = data.stereoParams.RotationOfCamera2;
T_2to1 = data.stereoParams.TranslationOfCamera2;
params.t = sqrt(T_2to1*T_2to1');

% Calculate the relation which can rectify the left and right image to 
% vertical status respectively.
v1 = -T_2to1/norm(T_2to1);
v2 = cross([0,0,1],v1);
v3 = cross(v1,v2);

R_rectify_1 = [v1' v2' v3']; 
R_rectify_2 = R_rectify_1*R_2to1';
params.rectify1 = R_rectify_1;
params.rectify2 = R_rectify_2;
% Note: Above rotations mean the final pose of image, which also represent
% the output image. 
%   In a detailed explanation, we will first give a output image, and we 
% use the undistorted pixel in output image to calculate the corresponding 
% distorted pixel in the input image. What we need to focus on is that the
% output image can be not parallel to the input image, there would be a
% rotation between them at this moment. ...
% -----------------------------------------------------------------------------
% new scale-factor for both camera
% fx = (fx_1 + fx_2)/2; % ax = focal_lenght*sx
% fy = (fy_1 + fy_2)/2; % ay = focal_length*sy
% Revise. 2020.7.25. To get a same scale image pair, the left's and right's
% focal length in width and height direction should be identical. In addition,
% to ensure a bigger map, the focal length should be large rathar than small;
% Revise. 2020.8.12. To ensure a FOV by 90-degrees, we should verify the P.
% --------
P = 1;%1.055;
% --------
fx_new = max([fx_1, fx_2, fy_1, fy_2]) * P;
fy_new = fx_new;

% [sx,sy] are the number of pixels per world unit in x and y direction...

% Revise. 2020.11.5. To keep the principle point coinciding to the center of the
% rectified area, we set the (cx, cy) to (1920,1080)
% cx = (cx_1 + cx_2)/2 * scale;
% cy = (cy_1 + cy_2)/2 * scale;
cx_new = 1920;
cy_new = 1080;

%% 3 Load a image and show it
test_image_id = 1;
test_data_path = image_folder;

left_images = dir(fullfile([test_data_path,'/left/'],'*.png'));
right_images = dir(fullfile([test_data_path,'/right/'],'*.png'));
if isempty(left_images)
    left_images = dir(fullfile([test_data_path,'/left/'],'*.bmp'));
    right_images = dir(fullfile([test_data_path,'/right/'],'*.bmp'));
end
if isempty(left_images)
    left_images = dir(fullfile([test_data_path,'/left/'],'*.jpg'));
    right_images = dir(fullfile([test_data_path,'/right/'],'*.jpg'));
end
left_image_names = {left_images.name};
right_image_names = {right_images.name};
I1 = imread([test_data_path,'/left/',left_image_names{test_image_id}]);
I2 = imread([test_data_path,'/right/',right_image_names{test_image_id}]);
% -----------------------------------------------------------------------------
[height,width,~] = size(I1);
params.width = width;
params.height = height;
% the size of the output image relative to origin image
scale = 2;
width_rect = width*scale;
height_rect = height*scale;
params.roi1 = [width/2,height/2,width,height];
params.roi2 = [width/2,height/2,width,height];
% -----------------------------------------------------------------------------
% Before do the rectification, check the P first
testL = uint8(255*ones(height,width));
testR = uint8(255*ones(height,width));
while true
    % deal with left
    I1_rect = rectify(testL,R_rectify_1,fx_1,fy_1,cx_1,cy_1,k1,p1);
    % deal with right
    I2_rect = rectify(testR,R_rectify_2,fx_2,fy_2,cx_2,cy_2,k2,p2);
    % checkP
    [need2changeP,retP] = checkP(I1_rect,I2_rect);
    if need2changeP
        fprintf('检查完P参数，正在更新P参数...\n');
        % Display
        display(I1_rect,I2_rect,'图I-待更正的畸变校正区域','图I-截取后效果');
        % Update P
        P = retP;
        fx_new = max([fx_1, fx_2, fy_1, fy_2]) * P;
        fy_new = fx_new;
    else
        % Display
        display(I1_rect,I2_rect,'图II-畸变校正区域','图II-截取后效果');
        break;
    end
end

fprintf('正在对实际图像进行畸变矫正...\n');
% show the original image
figure('Name','图III-测试图像原图')
hold on;
title('Left & Right');
imshow([I1,I2]);
for j = 1:(size(I1,1)/20):size(I1,1)
    line([1,2*size(I1,2)],[j,j],'color','g');
end
hold off;

% deal with image-1
I1_rect = rectify(I1,R_rectify_1,fx_1,fy_1,cx_1,cy_1,k1,p1);
% deal with image-2
I2_rect = rectify(I2,R_rectify_2,fx_2,fy_2,cx_2,cy_2,k2,p2);
% Display
display(I1_rect,I2_rect,'图III-测试图像畸变矫正区域','图III-测试图像矫畸变后效果');

params.fov = rad2deg(2*atan(sqrt((width/2)^2 + (height/2)^2)/fx_new));

%% Save params
params.new_intrinsic = [fx_new, 0, cx_new/2; 0, fy_new, cy_new/2; 0, 0, 1];
fprintf('畸变校正结束！耗时%.3f秒.\n',toc(time_point));

params_path = [image_folder,'/cam_params'];
writeParams(params_path);
fprintf('相机参数已存储在"%s"文件夹下.\n\n',params_path);

% -----------------------------------------------------------------------------
    function [I_out] = rectify(I_in,R_rectify,fx,fy,cx,cy,k,p)
        % I_in  --- input image
        % R_rectify --- rectfication mat
        % k  ---  radial distortion
        % p  ---  tangential distortion
        I_out = uint8(zeros(height_rect,width_rect,size(I_in,3))); 
        mapx = zeros(height_rect,width_rect);
        mapy = zeros(height_rect,width_rect);
        parfor v = 1:height_rect
            for u = 1:width_rect
                % 1) convert pixel coordinate of image to physical coordinate(mm)
                %    of image
                vc = (v - cy_new)/fy_new;
                uc = (u - cx_new)/fx_new;
                % 2) rotate coordinate above to camera frame. Notes, camera frame
                %    is a translation by focal_length frome image physical frame
                p_under_cam = [uc; vc; 1];
                p_under_cam = R_rectify * p_under_cam;
                xc = p_under_cam(1)/p_under_cam(3); % normalize
                yc = p_under_cam(2)/p_under_cam(3); % normalize
                % 3) rectify distortion
                r = xc^2 + yc^2;
                kr = 1 + r * (k(1) + r * (k(2) + r * k(3)));
                ud = xc * kr + p(1) * 2*xc*yc + p(2)*(r + 2*xc^2);
                vd = yc * kr + p(2) * 2*xc*yc + p(1)*(r + 2*yc^2);
                % 4) convert the rectified coordinate(mm) to pixel unit
                xxx = ud * fx + cx;
                yyy = vd * fy + cy;
                mapx(v,u) = xxx;
                mapy(v,u) = yyy;
                if(xxx>=1 && xxx<=width && yyy>=1 && yyy<=height)
                    w = xxx;
                    h = yyy;
                    % bilinear interpolation
                    I_out(v,u,:) = ...
                        I_in(floor(h),floor(w),:)*(floor(w+1)-w)*(floor(h+1)-h) + ...
                        I_in(floor(h+1),floor(w),:)*(floor(w+1)-w)*(h-floor(h)) + ...
                        I_in(floor(h+1),floor(w+1),:)*(w-floor(w))*(h-floor(h)) + ...
                        I_in(floor(h),floor(w+1),:)*(w-floor(w))*(floor(h+1)-h);
                end
            end
        end
    end
end

% ----------------------------------------------------------------------------
function [] = display(I1_rect,I2_rect,f1_name,f2_name)
global params;
left_start = [params.roi1(2),params.roi1(1)];
right_start = [params.roi2(2),params.roi2(1)];
width = params.width;
height = params.height;

figure('Name',f1_name);
hold on;
title('Left & Right');
imshow([I1_rect,I2_rect]);
line([left_start(2),left_start(2)],...
    [left_start(1),left_start(1) + height], 'color','g');
line([left_start(2) + width, left_start(2) + width],...
    [left_start(1),left_start(1) + height],'color','g');
line([left_start(2),left_start(2) + width],...
    [left_start(1),left_start(1)], 'color','g');
line([left_start(2), left_start(2) + width],...
    [left_start(1) + height,left_start(1) + height],'color','g');
r_start = [right_start(1),right_start(2) + 2*width];
line([r_start(2),r_start(2)],...
    [r_start(1),r_start(1) + height], 'color','g');
line([r_start(2) + width, r_start(2) + width],...
    [r_start(1),r_start(1) + height],'color','g');
line([r_start(2),r_start(2) + width],...
    [r_start(1),r_start(1)], 'color','g');
line([r_start(2), r_start(2) + width],...
    [r_start(1) + height,r_start(1) + height],'color','g');
hold on;
scatter([width,width*3],[height, height],'filled','r');
hold off;
set(gcf,'color','w');
clear r_start
% -----------------------------------------------------------------------------
I1_crop = I1_rect(left_start(1):left_start(1)+height-1,...
    left_start(2):left_start(2)+width-1,:);
I2_crop = I2_rect(right_start(1):right_start(1)+height-1,...
    right_start(2):right_start(2)+width-1,:);
figure('Name',f2_name);
hold on;
title('Left & Right');
imshow([I1_crop,I2_crop]);
for j = 1:(size(I1_crop,1)/20):size(I1_crop,1)
    line([1,2*size(I1_crop,2)],[j,j],'color','g');
end
hold off;
clear j
end

function [ret,P] = checkP(I1_rect,I2_rect)
global params;
left_start = [params.roi1(2),params.roi1(1)];
right_start = [params.roi2(2),params.roi2(1)];
width = params.width;
height = params.height;

I1_crop = I1_rect(left_start(1):left_start(1)+height-1,...
    left_start(2):left_start(2)+width-1,:);
I2_crop = I2_rect(right_start(1):right_start(1)+height-1,...
    right_start(2):right_start(2)+width-1,:);

[ret1,P1] = findBlackSide(I1_crop);
[ret2,P2] = findBlackSide(I2_crop);
ret = ret1|ret2;
P = max([P1,P2]);

    function [ret,P] = findBlackSide(I)
        param = polyfit([1,540],[1,960],1);
        k = param(1); b = param(2);
        for ii = 1:height/2
            jj = round(k*ii+b);
            % get a four side
            l_up = I(ii:height-ii+1,jj,:);
            l_down = I(ii:height-ii+1,width-jj+1,:);
            l_left = I(ii,jj:width-jj+1,:);
            l_right = I(height-ii+1,jj:width-jj+1,:);
            pts = [l_up;l_down;l_left';l_right'];
            has_black = pts~=255;
            if(sum(has_black(:)) == 0)
                break;
            end
        end
        if ii == 1 && jj == 1
            ret = false;
            P = [];
        else
            ret = true;
            P = max([height/(height-2*(ii+1)), width/(width-2*(jj+1))]);
        end
    end
end

function [] = writeParams(path)
global params;
if ~exist(path,'dir')
    mkdir(path);
end
A1 = params.intrinsic1;
R1 = params.rectify1;
D1 = params.dist_coeffs1;
A2 = params.intrinsic2;
R2 = params.rectify2;
D2 = params.dist_coeffs2;

Anew = params.new_intrinsic;
roi1 = params.roi1;
roi2 = params.roi2;

width = params.width;
height = params.height;
t = params.t;
fov = params.fov;
if 0
    %% Save as texts
    save([path,'/A1.txt'],'A1','-ascii');
    save([path,'/R1.txt'],'R1','-ascii');
    save([path,'/D1.txt'],'D1','-ascii');
    
    save([path,'/A2.txt'],'A2','-ascii');
    save([path,'/R2.txt'],'R2','-ascii');
    save([path,'/D2.txt'],'D2','-ascii');
    
    save([path,'/roi1.txt'],'roi1','-ascii');
    save([path,'/roi2.txt'],'roi2','-ascii');
    
    save([path,'/Anew.txt'],'Anew','-ascii');
    
    save([path,'/t.txt'],'t','-ascii');
    save([path,'/fov.txt'],'fov','-ascii');
    
    widtheight = [width, height];
    save([path,'/widthheight.txt'],'widtheight','-ascii');
else
    %% Save as csv
    filename = [path,'/cam_params.sr'];
    fid = fopen(filename, 'w');
    time_str = datestr(datetime('now','Format','yyyy-MM-dd HH:mm:SS'));
    %fprintf(fid, ['%s','\n'],sprintf('%s',time_str));
    fprintf(fid, ['%s',',','%d','\n'],'width',width);
    fprintf(fid, ['%s',',','%d','\n'],'height',height);
    fprintf(fid, ['%s',',','%.4f','\n'],'fov',fov);
    fprintf(fid, ['%s',',','%.4f','\n'],'t',t);
    writeMatInCSV(fid,'A1',A1);
    writeMatInCSV(fid,'R1',R1);
    writeMatInCSV(fid,'D1',D1);
    writeMatInCSV(fid,'A2',A2);
    writeMatInCSV(fid,'R2',R2);
    writeMatInCSV(fid,'D2',D2);
    writeMatInCSV(fid,'roi1',roi1);
    writeMatInCSV(fid,'roi2',roi2);
    writeMatInCSV(fid,'Anew',Anew);    
    fclose(fid);
end
    function [] = writeMatInCSV(fid,name,mat)
        [row,col] = size(mat);
        fprintf(fid, ['%s','\n'],name);
        for i = 1:row
            for j = 1:col
                fprintf(fid,'%.8f',mat(i,j));
                if(j == col)
                    fprintf(fid,'\n');
                else
                    fprintf(fid,',');
                end
            end
        end
    end
end