%--------- stereo-camera coordinate marker's pose--------------------%
% to get stereo-camera marker's pose
% ----Info
% By Keith W.
% Date 20201110
% Ver c1.0
% input: 不同组采集到的数据
% output: 不同组的位姿文件，按照输入的名字进行保存
%-------------------------------------------------------------------------%
name='70_4_1';
fpath=['./1102/',name,'/data.log'];
M=load(fpath);
coord_path=['./test1102/trocar/Tcamera_trocar_data_',name,'.mat'];
t=load(coord_path);
Tcamera_trocar=t.Tcamera_trocar;
block_size=100;
Tcamera_marker=zeros(4,4,block_size);
Ttrocar_marker=zeros(4,4,block_size);
for i =1:block_size
    P_tem_1=M(i*2-1,[1 2 3]);
    P_tem_2=M(i*2,[1 2 3]);
    if(P_tem_1(3)<50||P_tem_2(3)<50)
        Tcamera_marker(:,:,i)=eye(4);
        Ttrocar_marker(:,:,i)=Tcamera_marker(:,:,i);
        continue;
    end
    z=(P_tem_2-P_tem_1)/norm(P_tem_2-P_tem_1);
    z=reshape(z,[3 1]);
    x=[1 0 0]';
    y=cross(z,x)/norm(cross(z,x));
    x=cross(y,z)/norm(cross(y,z));
    R=[x y z];
    Tcamera_marker(1:3,1:3,i)=R;
    P=reshape(P_tem_1,[3 1]);
    Tcamera_marker(1:3,4,i)=P;
    Tcamera_marker(4,4,i)=1;
    Ttrocar_marker(:,:,i)=inv(Tcamera_trocar)*Tcamera_marker(:,:,i);
end
fname=['./test1102/pose/Ttrocar_marker_data_',name];
save(fname,'Ttrocar_marker');


