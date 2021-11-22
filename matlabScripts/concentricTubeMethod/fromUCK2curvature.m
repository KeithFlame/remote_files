function uc=fromUCK2curvature(U,R,P,K,C)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% input1: clearance
% input2: every tube curvature inital guess
% input3: stiffness matrix
% output1: final curvature
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% step 1 
if (nargin==3)
    %calc K
    K=getK(structure_para);
    C=1;
end

%% step 2 计算初始曲率与猜测曲率之间的差异
% g=K(u1-u2)
U0=getU0;
g=K(U-U0);
%% step 3 计算雅克比
% Jp
columns_num=size(U,1)/3;
u=reshape(U,[3 columns_num]);
P=reshape(P,[3 columns_num]);
Jp=getJp(u,P,R,ds);

%% step 4 计算 间隙对应关系
% calc qij **********
    qk=getQk(Pi2i1,C);

%% 准备小循环
% lamda=[]

% calc Q h


