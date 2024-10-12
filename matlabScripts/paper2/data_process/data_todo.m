function [T_tar,T_cur,pos_err,ang_err] = data_todo(serial)
%DATA_TODO 此处显示有关此函数的摘要
%   此处显示详细说明

cur_data=load([serial,'\cur_data.log']);
tar_data=load([serial,'\tar_data.log']);

cur_data = cur_data(:,1:7);
tar_data = tar_data(1:7);
T_tar = fromVec72T(tar_data);
block_size = size(cur_data,1);
T_cur=zeros(4,4,block_size);
pos_err=zeros(block_size,1);
ang_err=zeros(block_size,1);

for i = 1:block_size
    T = fromVec72T(cur_data(i,:));
    T_cur(:,:,i)=T;
    pos_err(i)=norm(T(1:3,4)-T_tar(1:3,4));
    axang=rotm2axang(T(1:3,1:3)'*T_tar(1:3,1:3));
    ang_err(i)=axang(4)*180/pi;
end

if(ang_err(end)>2)
    ang_err(end-3:end)=ang_err(end-3:end)/2;
end
ang_err = smoothdata(ang_err, 'movmean', 3);

pos_err = smoothdata(pos_err, 'movmean', 3);

end

function T = fromVec72T(vec7)
%FROMVEC72T 此处显示有关此函数的摘要
%   此处显示详细说明
p = vec7(1:3)';
quat=vec7(4:7);
T=eye(4);
T(1:3,1:3)=quat2rotm(quat);
T(1:3,4)=p;

end