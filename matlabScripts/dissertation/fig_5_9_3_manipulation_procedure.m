file_path='F:\code_git\matlabScripts\paper2\data_process\pic_factory\test_0331_2\after_pic\';

I1 = imread([file_path,'M000.jpg']);
I2 = imread([file_path,'M031.jpg']);
I3 = imread([file_path,'M062.jpg']);
I4 = imread([file_path,'M093.jpg']);
I5 = imread([file_path,'M124.jpg']);
I6 = imread([file_path,'M155.jpg']);
I7 = imread([file_path,'M186.jpg']);
I8 = imread([file_path,'M217.jpg']);
I9 = imread([file_path,'M247.jpg']);

height_c=[80 0];
width_c=[650 0];

I1=I1(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I2=I2(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I3=I3(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I4=I4(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I5=I5(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I6=I6(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I7=I7(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I8=I8(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);
I9=I9(height_c(1):(end-height_c(2)),width_c(1):(end-width_c(2)),:);

I_com=[I1 I2 I3;I4 I5 I6;I7 I8 I9];
I_com=imresize(I_com,0.3);
imshow(I_com)