function T=getPortChannel(name)

%--------- On trocar, marker's pose--------------------%
% to get  marker's pose
% ----Info
% By Keith W.
% Date 20201207
% Ver c1.0
% input: 不同组采集到的数据文件关键字
% output: 不同的trocar出口相对于trocar的坐标转换矩阵
%------------------------------------------------------%

persistent T0;
if(nargin==0)
    T=T0;
    return;
end
split_name = regexp(name, '_', 'split');

%% trocar channel positions
 pc1= [0.0  -7.85  0.0 ]'*1e-3;
 pc2= [7.66 -1.7   0.0 ]'*1e-3;
 pc3= [0.0   5.8 -15.0 ]'*1e-3;
 pc4=[-7.66 -1.7   0.0 ]'*1e-3;
 Pc=[pc1 pc2 pc3 pc4];
 pc=Pc(:,str2double(char(split_name(2))));
 %% history tau_s
 tau1 = -0.934;
 tau2 = -0.489;
 tau4 = 0.268;
 Tau=[tau1 tau2 0 tau4];
 tau=Tau(str2double(char(split_name(2))));
 %% trocar outport w.r.t trocar world
T_ch_tr = [Expm([0 0 tau+(11/180*pi)]') ...
    pc;0 0 0 1];%2nd arm of 03 surgical
T=T_ch_tr;
T0=T;
end