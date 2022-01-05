close,clear,clc;
%%
path_trocar='../test1206/59_2/otherTest/coord_59_2_1.log';
path_marker='../test1206/59_2/otherTest/round_1.log';
Tch_tr=getPortChannel('59_2');
Tch_tr(1:3,4)=Tch_tr(1:3,4)*1000;
[Tcamera_trocar_1,z_dev_1,ang_dev_1]=getTrocar1Camera(load(path_trocar));
fprintf("->_1 两轴夹角为：%f°\n",ang_dev_1);
fprintf("->_1 两轴间距为：%fmm\n",z_dev_1);
[Tcamera_marker_1, Tcamera_marker_p_1, Tcamera_marker_d_1, angle_block_1]=getMarker1Camera(load(path_marker));


block_size=max(size(Tcamera_marker_d_1));
M_d=zeros(block_size,3);
Ttrocar_marker_d=zeros(4,4,block_size);
M_p=zeros(block_size,3);
Ttrocar_marker_p=zeros(4,4,block_size);
for i =1:block_size
    tem=Tcamera_trocar_1\Tcamera_marker_d_1(:,:,i);
    Ttrocar_marker_d(:,:,i)=Tch_tr\tem;
    M_d(i,:)=Ttrocar_marker_d(1:3,4,i)';

    tem=Tcamera_trocar_1\Tcamera_marker_p_1(:,:,i);
    Ttrocar_marker_p(:,:,i)=Tch_tr\tem;
    M_p(i,:)=Ttrocar_marker_p(1:3,4,i)';
end


[p0_d,r_d]=spatialCircleFitV2(M_d(:,1:3));
[p0_p,r_p]=spatialCircleFitV2(M_p(:,1:3));
plot3(p0_d(1), p0_d(2), p0_d(3),'^r');
plot3(p0_d(1), p0_d(2), p0_d(3),'*b');
hold on;
for i = 1:block_size
    plot3([M_d(i,1) M_p(i,1)],[M_d(i,2) M_p(i,2)],[M_d(i,3) M_p(i,3)],'-r');
end
%% 验证tau
tau_m_b=180-acosd(dot(M_d(1,1:2),[-1 0])/norm(M_d(1,1:2)));
tau_m_e=180-acosd(dot(M_d(end,1:2),[-1 0])/norm(M_d(end,1:2)));
%% 图片合成
% is_assemblied=1;
% if(is_assemblied)
%     figure;
%     t_alpha=0.15;
%     I1=imread('../test1206/59_2/otherTest/testPrecision/left_1.jpg');
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_2.jpg');
%     I1=imadd(t_alpha*I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_3.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_4.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_5.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_6.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_7.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_8.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_9.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_10.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_11.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_12.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_13.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_14.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     I2=imread('../test1206/59_2/otherTest/testPrecision/left_15.jpg');
%     I1=imadd(I1,t_alpha*I2);
%     imshow(I1);
%     for i =2:2
%         p_path=['../test1206/59_2/otherTest/testPrecision/left_',num2str(i),'.jpg'];
%         I2=imread(p_path);
%         hold on;
%         I_tem=abs(I1-I2);
%         I_tem(all(I_tem<200,"all"),:,:)=0;
%         I_tem(all(I_tem>200,"all"),:,:)=0.2;
%         I1=imadd(I1,I_tem.*I2);
%         imshow(I1);
%     end
% end