function [P] = getP(M)
%GETP 此处显示有关此函数的摘要
%   此处显示详细说明
M_C_r40=M;
block_size=max(size(M_C_r40));
Tbase=zeros(4,4,block_size);
Tmarker=zeros(4,4,block_size);
T_marker1base=zeros(4,4,block_size);
rotX=[1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 1];
for i = 1:block_size
    Pbase=M_C_r40(i,2:4)';
    if(Pbase(3)==0)
        continue;
    end
    Rbase=[M_C_r40(i,5:7)' M_C_r40(i,8:10)' M_C_r40(i,11:13)' ]';
    Tbase(:,:,i)=[Rbase Pbase;[0 0 0 1]];
    Pmarker=M_C_r40(i,14:16)';
    Rmarker=[M_C_r40(i,17:19)' M_C_r40(i,20:22)' M_C_r40(i,23:25)' ]';
    Tmarker(:,:,i)=[Rmarker Pmarker;[0 0 0 1]];
    T_marker1base(:,:,i)=Tbase(:,:,i)\Tmarker(:,:,i);
    T_marker1base(:,:,i)=rotX*T_marker1base(:,:,i)*[1 0 0 0;0 1 0 0;0 0 1 -10; 0 0 0 1];
end

P=reshape(T_marker1base(1:3,4,:),[3 block_size]);
x=find(P(3,:)==0);
P(:,x)=[];
end

