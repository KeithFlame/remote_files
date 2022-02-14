clear ,close,clc;
[T1,T2,T3]=getData;

% test 1 marker2、3之间的夹角和距离
T_marker_2=T1(1:4,:,:);
T_marker_3=T1(5:8,:,:);
block_size=size(T1,3);
T_2_3=zeros(4,4,block_size);
ang_2_3=zeros(block_size,2); % 第一列是旋转矩阵夹角，第二列是Z轴夹角
dis_2_3=zeros(block_size,3);
for i = 1:block_size
    if(T_marker_3(3,3,i)==0||T_marker_2(3,3,i)==0)
        continue;
    end
    T_2_3(:,:,i)=T_marker_3(:,:,i)\T_marker_2(:,:,i);
    axang=rotm2axang(T_2_3(1:3,1:3,i));
    ang_2_3(i,1)=axang(4)*180/pi;
    ang_2_3(i,2)=acosd(dot(T_2_3(1:3,3,i),[0 0 1]'));
    dis_2_3(i,:)=T_2_3(1:3,4,i)';
end

% test 2 marker4、3之间的夹角和距离
T_marker_3=T2(1:4,:,:);
T_marker_4=T2(5:8,:,:);
block_size=size(T2,3);
T_4_3=zeros(4,4,block_size);
ang_4_3=zeros(block_size,2); % 第一列是旋转矩阵夹角，第二列是Z轴夹角
dis_4_3=zeros(block_size,3);
for i = 1:block_size
    T_4_3(:,:,i)=T_marker_3(:,:,i)\T_marker_4(:,:,i);
    axang=rotm2axang(T_4_3(1:3,1:3,i));
    ang_4_3(i,1)=axang(4)*180/pi;
    ang_4_3(i,2)=acosd(dot(T_4_3(1:3,3,i),[0 0 1]'));
    dis_4_3(i,:)=T_4_3(1:3,4,i)';
end
% [p0,r]=spatialCircleFitV2(dis_4_3);




%% 2e下的marker
T_trocar_camera=T_marker_3(:,:,1)*[1 0 0 0;0 1 0 0;0 0 1 0; -4.9157  -42.4688  -55.8442 1]'*[1 0 0 0; 0 -1 0 0;0 0 -1 0;0 0 0 1];
% T_trocar_camera(1:3,4)=[-4.9157  -42.4688  -55.8442]';
figure;hold on;axis equal;
% plotCoord(T_trocar_camera);
position_4_trocar=zeros(block_size,3);
T_4_trocar=zeros(4,4,block_size);
for i =1 :block_size
    ang=180-(i-1)*20;
    temT=T_trocar_camera\T_marker_4(:,:,i);
    position_4_trocar(i,:)=temT(1:3,4)';
    T_4_trocar(:,:,i)=rotZ(ang)'*temT;
end
[p0,r]=spatialCircleFitV2(position_4_trocar);
%%
% test 3 marker1、3、5之间的夹角和距离
T_marker_1=T3(1:4,:,:);
T_marker_3=T3(5:8,:,:);
T_marker_5=T3(9:12,:,:);
block_size=size(T3,3);
T_1_5=zeros(4,4,block_size);
T_3_5=zeros(4,4,block_size);
T_3_1=zeros(4,4,block_size);
ang_1_5=zeros(block_size,2); % 第一列是旋转矩阵夹角，第二列是Z轴夹角
dis_1_5=zeros(block_size,3);
ang_3_5=zeros(block_size,2); % 第一列是旋转矩阵夹角，第二列是Z轴夹角
dis_3_5=zeros(block_size,3);
ang_3_1=zeros(block_size,2); % 第一列是旋转矩阵夹角，第二列是Z轴夹角
dis_3_1=zeros(block_size,3);
for i = 1:block_size
    T_1_5(:,:,i)=T_marker_5(:,:,i)\T_marker_1(:,:,i);
    axang=rotm2axang(T_1_5(1:3,1:3,i));
    ang_1_5(i,1)=axang(4)*180/pi;
    ang_1_5(i,2)=acosd(dot(T_1_5(1:3,3,i),[0 0 1]'));
    dis_1_5(i,:)=T_1_5(1:3,4,i)';
    
    tem_rotZ=[-1 0 0 0;0 -1 0 0;0 0 1 0;0 0 0 1];
    T_3_5(:,:,i)=T_marker_5(:,:,i)\(T_marker_3(:,:,i)*tem_rotZ);
    axang=rotm2axang(T_3_5(1:3,1:3,i));
    ang_3_5(i,1)=axang(4)*180/pi;
%     ang_3_5(i,1)=acosd(dot(T_marker_5(1:3,3,i),T_marker_3(1:3,2,i)));
    ang_3_5(i,2)=acosd(dot(T_3_5(1:3,3,i),[0 0 1]'));
    dis_3_5(i,:)=T_3_5(1:3,4,i)';   

    T_3_1(:,:,i)=T_marker_1(:,:,i)\T_marker_3(:,:,i);
    axang=rotm2axang(T_3_1(1:3,1:3,i));
    ang_3_1(i,1)=axang(4)*180/pi;
    ang_3_1(i,2)=acosd(dot(T_3_1(1:3,3,i),[0 0 1]'));
    dis_3_1(i,:)=T_3_1(1:3,4,i)';  
end
