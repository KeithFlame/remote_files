%--------- link the measured pose--------------------%
% 
% ----Info
% By Keith W.
% Date 20201110
% Ver c1.0
% input: 不同组采集到的数据
% output: 不同组的位姿文件，将其拼接并按照输入的名字进行保存
%-------------------------------------------------------------------------%

name='33_1';
%% pose
n1=['./test1102/pose/Ttrocar_marker_data_',name,'_1.mat'];
n2=['./test1102/pose/Ttrocar_marker_data_',name,'_2.mat'];
t=load(n1);
Ttrocar_marker_1=t.Ttrocar_marker;
t=load(n2);
Ttrocar_marker_2=t.Ttrocar_marker;
block_size = size(Ttrocar_marker_1,3);
Ttrocar_marker=zeros(4,4,block_size);
count=0;
for i =1:block_size
    if(Ttrocar_marker_1(3,4,i)==0&&Ttrocar_marker_2(3,4,i)~=0)
        Ttrocar_marker(:,:,i)=Ttrocar_marker_2(:,:,i);
    else
        Ttrocar_marker(:,:,i)=Ttrocar_marker_1(:,:,i);
    end
    if(Ttrocar_marker(3,4,i)==0)
        count=count+1;
    end
end
fname=['./test1102/pose/Ttrocar_marker_data_',name];
save(fname,'Ttrocar_marker');


%% psi
n1=['./test1102/psi/Psi_actual_',name,'_1.mat'];
n2=['./test1102/psi/Psi_actual_',name,'_2.mat'];
t=load(n1);
Psi_1=t.Psi;
t=load(n2);
Psi_2=t.Psi;
count_psi=0;
for i =1:block_size
    if(norm(Psi_2(:,i)-Psi_1(:,i))>1e-6)
        count_psi=count_psi+1;
    end
end
if(count_psi==0)
    Psi=Psi_1;
    fname=['./test1102/psi/Psi_actual_',name];
    save(fname,'Psi');
else
    fprintf("错误的Psi");
end