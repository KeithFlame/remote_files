%--------- link the measured pose--------------------%
% 
% ----Info
% By Keith W.
% Date 20201110
% Ver c1.0
% input: 同组不同marker采集到的数据
% output: 不同marker位姿的偏差
%-------------------------------------------------------------------------%

name='59_2';
%% pose
n1=['./test1102/pose_new/Ttrocar_marker_p_data_',name,'.mat'];
n2=['./test1102/pose_new/Ttrocar_marker_r_data_',name,'.mat'];
p=['./test1102/psi_new/Psi_actual_',name,'.mat'];
t=load(n1);
Ttrocar_marker_1=t.Ttrocar_marker;
t=load(n2);
Ttrocar_marker_2=t.Ttrocar_marker;
t=load(p);
Psi=t.Psi;

i=100;
block_size=max(size(Ttrocar_marker_2));
angle_block=zeros(block_size,1);
psi=zeros(1,6);
while(1)
    if((~(Ttrocar_marker_1(3,4,i)==0))&&(~(Ttrocar_marker_2(3,4,i)==0)))
        r1=Ttrocar_marker_1(1:3,1:3,i);
        r2=Ttrocar_marker_2(1:3,1:3,i);
        axang=rotm2axang(r1'*r2);
        angle_block(i)=axang(4)*180/pi;
    else

    end
    if(i==block_size)
        break;
    else
        i=i-1;
    end
end
angle_block(all(angle_block==0,2),:)=[];
% % % % figure;
% % % % mean_angle=mean(angle_block);
% % % % stem(angle_block-mean_angle);

% figure;
hold on; grid on; axis equal;
% PlotAxis(0.005,Ttrocar_marker_1(:,:,6));

% psi=;
selected=54;
T=plot_snake(Ttrocar_marker_1(:,:,selected),Psi(:,selected)',name);


%%
psi2=zeros(6,block_size*2);
for i =1:block_size
    psi2(:,i*2-1)=Psi(:,i);
    psi2(1:2,i*2)=Psi(1:2,i);
end
fileConfig=fopen('batch_psi_2.data','w');
for i =1:block_size*2
    finalValue=psi2(:,i);
    fprintf(fileConfig,'%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,%6.4f,\n',finalValue);
end
fclose(fileConfig);