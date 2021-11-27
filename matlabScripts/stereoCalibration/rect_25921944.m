%% Rectifying the stereo image pairs
% clear
% close all
% Date: 2019.6.25 - 6.29, by WangLf.
% Recording:
%  Calibration error of NO.12: about 0.7-0.8 mm
%  Calibration error of NO.24: about 0.6 mm

% Define a global variable to store the useful parameters of the stereo-camera
global params;

tic
datetime('now','Format','yyyy-MM-dd HH:mm:SS')
%% 1 Load parameters
cam_no = 1;
partialPath=pwd;
folderPath = [partialPath,'/D1125_',num2str(cam_no)];
load([folderPath,'/stereoParams.mat']);

% test_image_id = 7;
% IS_SAVE_TEST_IMAGE = false;
% test_data_path = [folder_path];

%% 2.1 Retrieving parameters of Camera 1
% 1 Retrieve intrinsic matrix
params.intrinsic1 = stereoParams.CameraParameters1.IntrinsicMatrix';

% 1.1 Retrieve scale-factors of u-axe and v-axe of image, in pixel
ax_1 = params.intrinsic1(1,1);
ay_1 = params.intrinsic1(2,2);

% 1.2 Retrieve optical center (u,v)or(cx,cy) of image, in pixel
u0_1 = params.intrinsic1(1,3);
v0_1 = params.intrinsic1(2,3);

% 2 Retrieve radial and tangential distortion
params.dist_coeffs1 = zeros(1,5);
params.dist_coeffs1(1:length(stereoParams.CameraParameters1.RadialDistortion))...
    = stereoParams.CameraParameters1.RadialDistortion;
params.dist_coeffs1(4:5) = stereoParams.CameraParameters1.TangentialDistortion;



% 2.1 Distribute distortion
k1_1 = params.dist_coeffs1(1);
k2_1 = params.dist_coeffs1(2);
k3_1 = params.dist_coeffs1(3);
p1_1 = params.dist_coeffs1(4);
p2_1 = params.dist_coeffs1(5);

%% 2.2 Retrieving parameters of Camera 2
% 1 Retrieve intrinsic matrix
params.intrinsic2 = stereoParams.CameraParameters2.IntrinsicMatrix';

% 1.1 Retrieve scale-factors of u-axe and v-axe of image, in pixel
ax_2 = params.intrinsic2(1,1);
ay_2 = params.intrinsic2(2,2);

% 1.2 Retrieve optical center (u,v)or(cx,cy) of image, in pixel
u0_2 = params.intrinsic2(1,3);
v0_2 = params.intrinsic2(2,3);

% 2 Retrieve radial and tangential distortion
params.dist_coeffs2 = zeros(1,5);
params.dist_coeffs2(1:length(stereoParams.CameraParameters2.RadialDistortion))...
    = stereoParams.CameraParameters2.RadialDistortion;
params.dist_coeffs2(4:5) = stereoParams.CameraParameters2.TangentialDistortion;

% 2.1 Distribute distortion
k1_2 = params.dist_coeffs2(1);
k2_2 = params.dist_coeffs2(2);
k3_2 = params.dist_coeffs2(3);
p1_2 = params.dist_coeffs2(4);
p2_2 = params.dist_coeffs2(5);

%% 2.3 Calculateing rectifying parameters
% Get the rotation and translation relationship between camera 1 and 2
R_2to1 = stereoParams.RotationOfCamera2;
T_2to1 = stereoParams.TranslationOfCamera2;

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

%% 3 Load a image and show it
% left_images = dir(fullfile([test_data_path,'/left/'],'*.png'));
% right_images = dir(fullfile([test_data_path,'/right/'],'*.png'));
% if isempty(left_images)
%     left_images = dir(fullfile([test_data_path,'/left/'],'*.bmp'));
%     right_images = dir(fullfile([test_data_path,'/right/'],'*.bmp'));
% end
% left_image_names = {left_images.name};
% right_image_names = {right_images.name};
% I1 = imread([test_data_path,'/left/',left_image_names{test_image_id}]);
% I2 = imread([test_data_path,'/right/',right_image_names{test_image_id}]);
% I1=imread('E:/术锐/ROI选取/roi/NO24_wlf/FindROI.png');
% I2=imread('E:/术锐/ROI选取/roi/NO24_wlf/FindROI.png');
I1 = imread([folderPath, '\left\left06.jpg']);
I2 = imread([folderPath, '\right\right06.jpg']);

figure('Name','Image before rectification')
hold on;
title('Left & Right');
imshow([I1,I2]);
for j = 1:(size(I1,1)/20):size(I1,1)
    line([1,2*size(I1,2)],[j,j],'color','g');
end
hold off;

[height,width,channel] = size(I1);
params.width = width;
params.height = height;
params.ratio=1/params.height*params.width;
%% 4 Rectifying image by build-in function
% [J1,J2] = rectifyStereoImages(I1,I2,stereoParams);
% figure('Name','Image after rectification by build-in function')
% hold on;
% imshow([J1,J2]);
% for j = 1:(size(J1,1)/20):size(J1,1)
%     line([1,2*size(J1,2)],[j,j],'color','g');
% end
% title('Left & Right');
% hold off;
% % figure('Name','Red-cyan image by build-in function')
% % imshow(stereoAnaglyph(J1,J2));

%% 5 Rectifying image by manual calculating
% new scale-factor for both camera
ax = (ax_1 + ax_2)/2; % ax = focal_lenght*sx
ay = (ay_1 + ay_2)/2; % ay = focal_length*sy
% Revise. 2020.7.25. To get a same scale image pair, the left's and right's
% focal length in width and height direction should be identical. In addition,
% to ensure a bigger map, the focal length should be large rathar than small;
% Revise. 2020.8.12. To ensure a FOV by 90-degrees, we should verify the P.
% --------
% P = 1.0942;
P=1.0;%第一�?
% --------
ax = max([ax_1, ax_2, ay_1, ay_2]) * P;
ay = ax;

% [sx,sy] are the number of pixels per world unit in x and y direction...

% the size of the output image relative to origin image
scale = 2;
width_rect = width*scale;
height_rect = height*scale;

% u0 = (u0_1 + u0_2)/2 * scale;
% v0 = (v0_1 + v0_2)/2 * scale;
u0=params.width/2*scale;
v0=params.height/2*scale;

params.new_intrinsic = [ax, 0, u0/2; 0, ay, v0/2; 0, 0, 1];
params.P=P;
%成像平面放大两�?�，�?以光心在u0，v0处，则left_start为（u0-960,v0-540�?;
%18�?23之后的模组将光心定为960�?540，因此left_start为（960,540�?;

mapx1 = zeros(height_rect,width_rect);
mapy1 = zeros(height_rect,width_rect);
tic
% deal with image-1
I1_rect = uint8(zeros(height_rect,width_rect,channel));
for v = 1:height_rect
    for u = 1:width_rect
        % 1) convert pixel coordinate of image to physical coordinate(mm)
        %    of image

        y = (v - params.height)/ay; 
        x = (u - params.width)/ax;
%         y = (v - v0)/ay; 
%         x = (u - u0)/ax;


        % 2) rotate coordinate above to camera frame. Notes, camera frame
        %    is a translation by focal_length from image physical frame
        p_under_cam = [x; y; 1];
        p_under_cam = R_rectify_1*p_under_cam;
        xc = p_under_cam(1)/p_under_cam(3); % normalize
        yc = p_under_cam(2)/p_under_cam(3); % normalize
        % 3) rectify distortion
        r = xc^2 + yc^2;
        xx = xc*(1 + k1_1*r + k2_1*r^2 + k3_1*r^3) + ...
            2*p1_1*yc + p2_1*(r + 2*xc^2);
        yy = yc*(1 + k1_1*r + k2_1*r^2 + k3_1*r^3) + ...
            2*p2_1*xc + p1_1*(r + 2*yc^2);
        % 4) convert the rectified coordinate(mm) to pixel unit
        
        xxx = xx*ax_1 + u0_1;%DLF- u0_1*scale?
        yyy = yy*ay_1 + v0_1;
        if (-1<(xxx-u0_1)&&(xxx-u0_1)<1)&&(-1<(yyy-v0_1)&&(yyy-v0_1)<1)
            ROI_l_x=u;
            ROI_l_y=v;
        end
        
        mapx1(v,u) = xxx;
        mapy1(v,u) = yyy;
        if(xxx>=1 && xxx<=width && yyy>=1 && yyy<=height)
           w = xxx;
           h = yyy;
           % bilinear interpolation
           I1_rect(v,u,:) = ...
               I1(floor(h),floor(w),:)*(floor(w+1)-w)*(floor(h+1)-h) + ...
               I1(floor(h+1),floor(w),:)*(floor(w+1)-w)*(h-floor(h)) + ...
               I1(floor(h+1),floor(w+1),:)*(w-floor(w))*(h-floor(h)) + ...
               I1(floor(h),floor(w+1),:)*(w-floor(w))*(floor(h+1)-h);        
        end
    end
end
% figure();
% imshow(I1_rect);
%DLF-calculate optical axis location/ROI
% ROI_l_temp=R_rectify_1*[0;0;1];
% 
%         xc = ROI_l_temp(1)/ROI_l_temp(3); % normalize
%         yc = ROI_l_temp(2)/ROI_l_temp(3); % normalize
%         % 3) rectify distortion
%         r = xc^2 + yc^2;
%         xx = xc*(1 + k1_1*r + k2_1*r^2 + k3_1*r^3) + ...
%             2*p1_1*yc + p2_1*(r + 2*xc^2);
%         yy = yc*(1 + k1_1*r + k2_1*r^2 + k3_1*r^3) + ...
%             2*p2_1*xc + p1_1*(r + 2*yc^2);
%         % 4) convert the rectified coordinate(mm) to pixel unit
%         ROI_l_x = xx*ax_1*scale + u0_1*scale;
%         ROI_l_y = yy*ay_1*scale + v0_1*scale;
%         
% ROI_r_temp=R_rectify_2*[0;0;1];
% 
%         xc = ROI_r_temp(1)/ROI_r_temp(3); % normalize
%         yc = ROI_r_temp(2)/ROI_r_temp(3); % normalize
%         % 3) rectify distortion
%         r = xc^2 + yc^2;
%         xx = xc*(1 + k1_2*r + k2_2*r^2 + k3_2*r^3) + ...
%             2*p1_2*yc + p2_2*(r + 2*xc^2);
%         yy = yc*(1 + k1_2*r + k2_2*r^2 + k3_2*r^3) + ...
%             2*p2_2*xc + p1_2*(r + 2*yc^2);
%         % 4) convert the rectified coordinate(mm) to pixel unit
%         ROI_r_x = xx*ax_1*scale + u0_2*scale;
%         ROI_r_y = yy*ay_1*scale + v0_2*scale;



% SHOW Map
% figure('Name','Mapx and Mapy of Camera 1');
% imshow([mapx1,mapy1]);
% imshow(I1_rect);
% for j = 1:(size(I1_rect,1)/20):size(I1_rect,1)
%     line([1,size(I1_rect,2)],[j,j],'color','g');
% end
% deal with image-2
I2_rect = uint8(zeros(height_rect,width_rect,channel));

mapx2 = zeros(height_rect,width_rect);
mapy2 = zeros(height_rect,width_rect);
for v = 1:height_rect
    for u = 1:width_rect
        % 1) convert pixel coordinate of image to physical coordinate(mm)
        %    of image
%         y = (v - v0)/ay; 
%         x = (u - u0)/ax;
        y = (v - params.height)/ay; 
        x = (u - params.width)/ax;


        % 2) rotate coordinate above to camera frame. Notes, camera frame
        %    is a translation by focal_length frome image physical frame
        p_under_cam = [x; y; 1];
        p_under_cam = R_rectify_2*p_under_cam;
        xc = p_under_cam(1)/p_under_cam(3); % normalize
        yc = p_under_cam(2)/p_under_cam(3); % normalize
        % 3) rectify distortion
        r = xc^2 + yc^2;
        xx = xc*(1 + k1_2*r + k2_2*r^2 + k3_2*r^3) + ...
            2*p1_2*yc + p2_2*(r + 2*xc^2);
        yy = yc*(1 + k1_2*r + k2_2*r^2 + k3_2*r^3) + ...
            2*p2_2*xc + p1_2*(r + 2*yc^2);
        % 4) convert the rectified coordinate(mm) to pixel unit
        xxx = xx*ax_2 + u0_2;
        yyy = yy*ay_2 + v0_2;
        if (-0.5<(xxx-u0_2)&&(xxx-u0_2)<0.5)&&(-0.5<(yyy-v0_2)&&(yyy-v0_2)<0.5)
            ROI_r_x=u;
            ROI_r_y=v;
        end
        mapx2(v,u) = xxx;
        mapy2(v,u) = yyy;
        if(xxx>=1 && xxx<=width && yyy>=1 && yyy<=height)
           w = xxx;
           h = yyy;
           % bilinear interpolation
           I2_rect(v,u,:) = ...
               I2(floor(h),floor(w),:)*(floor(w+1)-w)*(floor(h+1)-h) + ...
               I2(floor(h+1),floor(w),:)*(floor(w+1)-w)*(h-floor(h)) + ...
               I2(floor(h+1),floor(w+1),:)*(w-floor(w))*(h-floor(h)) + ...
               I2(floor(h),floor(w+1),:)*(w-floor(w))*(floor(h+1)-h);        
        end
    end
end
toc
% SHOW Map
% figure('Name','Mapx and Mapy of Camera 2');
% imshow([mapx2,mapy2]);

%% 6 Manual
% --------
% left_start = [542,950]; % (row, col)

% left_start = [int16(v0) - 540, int16(u0) - 950];
% right_start = [left_start(1),left_start(2)+0];
for y=params.height:-1:900
    w=int16(y/params.height*params.width/2);
    left_start =[params.height - int16(y/2), params.width - w];%left ROI is equal to right one

    correct=1;
    %up board
    for i=left_start(2):1:(left_start(2)+w*2)
        correct=correct*(I1_rect(left_start(1),i,1)+I1_rect(left_start(1),i,2)+I1_rect(left_start(1),i,3))*(I2_rect(left_start(1),i,1)+I2_rect(left_start(1),i,2)+I2_rect(left_start(1),i,3));
        if(correct==0)
            break;
        end
    end
    if(correct==0)
        continue;
    end
    
    %right board
    for i=left_start(1):1:(left_start(1)+y)
        correct=correct*(I1_rect(i,left_start(2)+w*2,1)+I1_rect(i,left_start(2)+w*2,2)+I1_rect(i,left_start(2)+w*2,3))*(I2_rect(i,left_start(2)+w*2,1)+I2_rect(i,left_start(2)+w*2,2)+I2_rect(i,left_start(2)+w*2,3));
        if(correct==0)
            break;
        end
    end
    if(correct==0)
        continue;
    end
        
    %down board
    for i=left_start(2):1:(left_start(2)+w*2)
        correct=correct*(I1_rect(left_start(1)+y,i,1)+I1_rect(left_start(1)+y,i,2)+I1_rect(left_start(1)+y,i,3))*(I2_rect(left_start(1)+y,i,1)+I2_rect(left_start(1)+y,i,2)+I2_rect(left_start(1)+y,i,3));
        if(correct==0)
            break;
        end
    end
    if(correct==0)
        continue;
    end
    
    %left board
    for i=left_start(1):1:(left_start(1)+y)
        correct=correct*(I1_rect(i,left_start(2),1)+I1_rect(i,left_start(2),2)+I1_rect(i,left_start(2),3))*(I2_rect(i,left_start(2),1)+I2_rect(i,left_start(2),2)+I2_rect(i,left_start(2),3));
        if(correct==0)
            break;
        end
    end
    if(correct==0)
        continue;
    else
        break;
    end
    
end
%  y=1080;
left_start =[params.height - int16(y/2), params.width - int16(y*params.ratio/2)];
right_start =[params.height - int16(y/2), params.width - int16(y*params.ratio/2)];

 left_start1 =[int16(v0) - params.height/2, int16(u0) - params.width/2];
 right_start1 =[int16(v0) - params.height/2, int16(u0) - params.width/2];
% left_new_start=[int16(ROI_l_y) - 540, int16(ROI_l_x) - 960];
% right_new_start=[int16(ROI_r_y) - 540, int16(ROI_r_x) - 960];
% left_new_start=[1080 - 540, 1920 - 960];
% right_new_start=[1080 - 540, 1920 - 960];
% --------

I1_crop = I1_rect(left_start(1):left_start(1)+y-1,...
    left_start(2):left_start(2)+int16(y*params.ratio)-1,:);
I2_crop = I2_rect(right_start(1):right_start(1)+y-1,...
    right_start(2):right_start(2)+int16(y*params.ratio)-1,:);

% I1_new_crop=I1_rect(left_new_start(1):left_new_start(1)+height-1,...
%     left_new_start(2):left_new_start(2)+width-1,:);
% I2_new_crop=I2_rect(right_new_start(1):right_new_start(1)+height-1,...
%     right_new_start(2):right_new_start(2)+width-1,:);
figure('Name','Rectifying by manual calculating');
hold on;
title('Left & Right');
imshow([I1_rect,I2_rect]);

% hold on;
% plot(u0_1*scale,(height-v0_1)*scale,'r*');
% hold on;
% plot(u0_2*scale+width*scale,(height-v0_2)*scale,'r*');
% hold on;
line([left_start(2),left_start(2)],...
    [left_start(1),left_start(1) + y], 'color','g');
line([left_start(2) + int16(y*params.ratio), left_start(2) + int16(y*params.ratio)],...
    [left_start(1),left_start(1) + y],'color','g');
line([left_start(2),left_start(2) + int16(y*params.ratio)],...
    [left_start(1),left_start(1)], 'color','g');
line([left_start(2), left_start(2) + int16(y*params.ratio)],...
    [left_start(1) + y,left_start(1) + y],'color','g');
r_start = [right_start(1),right_start(2) + 2*width];
line([r_start(2),r_start(2)],...
    [r_start(1),r_start(1) + y], 'color','g');
line([r_start(2) + int16(y*params.ratio), r_start(2) + int16(y*params.ratio)],...
    [r_start(1),r_start(1) + y],'color','g');
line([r_start(2),r_start(2) + int16(y*params.ratio)],...
    [r_start(1),r_start(1)], 'color','g');
line([r_start(2), r_start(2) + int16(y*params.ratio)],...
    [r_start(1) + y,r_start(1) + y],'color','g');
%======
% line([left_new_start(2),left_new_start(2)],...
%     [left_new_start(1),left_new_start(1) + height], 'color','r');
% line([left_new_start(2) + width, left_new_start(2) + width],...
%     [left_new_start(1),left_new_start(1) + height],'color','r');
% line([left_new_start(2),left_new_start(2) + width],...
%     [left_new_start(1),left_new_start(1)], 'color','r');
% line([left_new_start(2), left_new_start(2) + width],...
%     [left_new_start(1) + height,left_new_start(1) + height],'color','r');
% r_new_start = [right_new_start(1),right_new_start(2) + 2*width];
% line([r_new_start(2),r_new_start(2)],...
%     [r_new_start(1),r_new_start(1) + height], 'color','r');
% line([r_new_start(2) + width, r_new_start(2) + width],...
%     [r_new_start(1),r_new_start(1) + height],'color','r');
% line([r_new_start(2),r_new_start(2) + width],...
%     [r_new_start(1),r_new_start(1)], 'color','r');
% line([r_new_start(2), r_new_start(2) + width],...
%     [r_new_start(1) + height,r_new_start(1) + height],'color','r');
hold off;
clear r_start;
% clear r_new_start;

figure('Name','Rectifying by manual calculating');
hold on;
title('Left & Right');
imshow([I1_crop,I2_crop]);
for j = 1:(size(I1_crop,1)/20):size(I1_crop,1)
    line([1,2*size(I1_crop,2)],[j,j],'color','g');
end
hold off;

figure('Name','NEW ROI Rectifying by manual calculating');
hold on;
title('Left & Right');
% imshow([I1_new_crop,I2_new_crop]);
% for j = 1:(size(I1_new_crop,1)/20):size(I1_new_crop,1)
%     line([1,2*size(I1_new_crop,2)],[j,j],'color','g');
% end
hold off;

% if IS_SAVE_TEST_IMAGE
%     imwrite(I1_crop,[test_data_path,'/left/rectified_',...
%         left_image_names{test_image_id}]);
%     imwrite(I2_crop,[test_data_path,'/right/rectified_',...
%         right_image_names{test_image_id}]);
% end
% clear j

params.roi1 = [left_start(2),left_start(1),width,height];
params.roi2 = [right_start(2),right_start(1),width,height];
datetime('now','Format','yyyy-MM-dd HH:mm:SS')

if 0
%     Give it up.
%     writeParamsAsOpencvYml(folder_path,num2str(cam_no));
end
if 0
    writeParamsAsTxt([folderPath,'/tmp']);
end
toc


%% 
csvwrite('mapx1.csv',mapx1);
csvwrite('mapy1.csv',mapy1);
csvwrite('mapx2.csv',mapx2);
csvwrite('mapy2.csv',mapy2);