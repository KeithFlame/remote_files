function K= getK(structure_para)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
%
% By Keith W.
% date 11.18.2021
% Ver. 1.0
% output1: stiffness
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
%% 
persistent K_res;
if(~isempty(K_res)&&nargin==0)
    K=K_res;
    return;
end

%% used para

zeta=structure_para(100);
k1=structure_para(100);
k2=structure_para(100);
pR=100; % pure rigid material stiffness / K2_Ls
MP.E=50e9;                          %Rod Young's modules
MP.mu=0.25;                         %Rod Poisson rate
MP.d1=0.95e-3;                      %Rod diameter seg1
MP.d2=0.4e-3;                       %Rod diameter seg2
MP.I1=pi*MP.d1^4/64;                %Rod inertia moment seg1
MP.A1=pi*MP.d1^2/4;                 %Rod cross area seg1
MP.I2=pi*MP.d2^4/64;                %Rod inertia moment seg2
MP.A2=pi*MP.d2^2/4;                 %Rod cross area seg1
MP.G=MP.E/2/(1+MP.mu);
MP.Kb1=diag([MP.E*MP.I1 MP.E*MP.I1 2*MP.G*MP.I1]);
MP.Kb2=diag([MP.E*MP.I2 MP.E*MP.I2 2*MP.G*MP.I2]);
%% inner tube stiffness matrix
% K2 is a variable, [K2_Ls K2_L1 K2_Lr K2_L2 K2_Lg] 
K2_L2=16*MP.Kb2+k2*MP.Kb1;
K2_L1=(4+k1)*MP.Kb1+16*MP.Kb2;
K2_Ls=K2_L1/zeta-K2_L2;
K2_Lr=K2_Ls*pR;
K2_Lg=K2_Ls*pR;
K2=zeros(15,15);
K2(1:3,1:3)=K2_Ls;
K2(4:6,4:6)=K2_L1;
K2(7:9,7:9)=K2_Lr;
K2(10:12,10:12)=K2_L2;
K2(13:15,13:15)=K2_Lg;
%% outer tube stiffness matrix
K1=K2_Ls*pR;
%% result
K=eye(size(K1,1)+size(K2,1));
K(1:size(K1,1),1:size(K1,1))=K1;
K(size(K1,1)+1:end,size(K1,1)+1:end)=K2;
K_res=K;
end

